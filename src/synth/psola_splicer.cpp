#include "psola_splicer.hpp"
#include "kelly_lochbaum.hpp"
#include "source_gen.hpp"
#include "analysis/lpc_analyzer.hpp"
#include "util/math_util.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace resamp::synth {

// ── 분석 프레임에서 LPC 선형 보간 ────────────────────────────────────────
static analysis::LpcCoeffs interp_lpc(
    const std::vector<analysis::AnalysisFrame>& frames,
    double src_sample_pos)
{
    if (frames.empty()) return {};

    // src_sample_pos 기준으로 가장 가까운 두 프레임 찾기
    int n = static_cast<int>(frames.size());
    int idx = 0;
    for (int i = 0; i < n; ++i) {
        if (frames[i].center_sample <= static_cast<int>(src_sample_pos))
            idx = i;
        else
            break;
    }

    if (idx >= n - 1) return frames[n - 1].lpc;

    const auto& fa = frames[idx];
    const auto& fb = frames[idx + 1];
    double span = fb.center_sample - fa.center_sample;
    if (span < 1.0) return fa.lpc;
    double t = (src_sample_pos - fa.center_sample) / span;
    t = math::clamp(t, 0.0, 1.0);

    // 계수 보간
    analysis::LpcCoeffs out;
    int P = std::min(static_cast<int>(fa.lpc.a.size()),
                     static_cast<int>(fb.lpc.a.size()));
    out.order = P;
    out.a.resize(P);
    out.k.resize(P);
    for (int i = 0; i < P; ++i) {
        out.a[i] = static_cast<float>(
            fa.lpc.a[i] * (1.0 - t) + fb.lpc.a[i] * t);
    }
    out.gain = static_cast<float>(
        fa.lpc.gain * (1.0 - t) + fb.lpc.gain * t);
    return out;
}

// ── 원본 위치 계산 (자음 속도 + 모음 루프) ───────────────────────────────
static double compute_src_position(
    int out_sample,
    int /*output_samples*/,
    const RenderParams& params,
    int N_src,
    int sample_rate)
{
    double consonant_scale = params.velocity / 100.0;
    if (consonant_scale < 0.01) consonant_scale = 0.01;

    int consonant_src_smp = static_cast<int>(
        params.consonant_ms * sample_rate / 1000.0);
    int consonant_tgt_smp = static_cast<int>(
        consonant_src_smp * consonant_scale);

    if (out_sample <= consonant_tgt_smp) {
        // 자음 영역: 역스케일
        return out_sample / consonant_scale;
    } else {
        // 모음 영역: consonant 이후를 루프
        int vowel_offset = out_sample - consonant_tgt_smp;
        int vowel_src_len = N_src - consonant_src_smp;
        if (vowel_src_len <= 0) return consonant_src_smp;
        int vowel_loop = vowel_offset % vowel_src_len;
        return consonant_src_smp + vowel_loop;
    }
}

// ── 메인 합성 ─────────────────────────────────────────────────────────────
// 샘플 단위 합성:
//   1. 출력 위치 → 원본 대응 위치 계산
//   2. 해당 위치의 LPC 보간
//   3. KL 필터에 LPC 설정
//   4. 타깃 F0 소스 샘플 → KL 필터 통과 → 출력
std::vector<float> psola_splice(
    const std::vector<float>&              signal,
    const std::vector<analysis::AnalysisFrame>& frames,
    const std::vector<double>&             f0_contour,
    const RenderParams&                    params,
    const SynthParams&                     sp,
    int                                    sample_rate,
    int                                    output_samples)
{
    int N_src = static_cast<int>(signal.size());

    // 타깃 F0 소스 신호 생성 (출력 길이)
    auto source = generate_source(f0_contour, sample_rate, sp);

    // KL 필터 초기화
    KellyLochbaumFilter kl(64);

    // 원본 RMS (볼륨 기준)
    double src_rms = math::rms(signal.data(), N_src);
    if (src_rms < 1e-6) src_rms = 0.1;  // 무음 소스 방어

    std::cerr << "[Resamp] N_src=" << N_src
              << " frames=" << frames.size()
              << " src_rms=" << src_rms
              << " output_samples=" << output_samples << '\n';

    // LPC 업데이트 간격
    // 256→128: 더 촘촘한 포먼트 추적, smooth=true로 전환 시 클릭 제거
    const int UPDATE_INTERVAL = 128;
    int prev_frame_idx = -1;

    std::vector<float> output(output_samples, 0.0f);

    for (int i = 0; i < output_samples; ++i) {
        // 원본 대응 위치
        double src_pos = compute_src_position(i, output_samples, params, N_src, sample_rate);
        src_pos = math::clamp(src_pos, 0.0, static_cast<double>(N_src - 1));

        // LPC 업데이트 (매 UPDATE_INTERVAL 샘플마다 또는 첫 샘플)
        if (i % UPDATE_INTERVAL == 0 || prev_frame_idx < 0) {
            auto lpc = interp_lpc(frames, src_pos);
            if (lpc.order > 0) {
                // smooth=true: bandwidth expansion(γ=0.997) 덕분에 필터 안정
                // → 전환 구간 64샘플 크로스페이드로 클릭/기계음 제거
                // apply_gain=false: 새 소스 사용 시 gain 무시 (에너지 제어는 별도)
                kl.set_coeffs(lpc, /*smooth=*/true, /*apply_gain=*/false);
            }
            prev_frame_idx = 0;
        }

        // 소스 샘플 가져오기
        float src_samp = (i < static_cast<int>(source.size())) ? source[i] : 0.0f;

        // KL 필터 통과 (성도 합성)
        output[i] = kl.process(src_samp);
    }

    // 출력 RMS를 원본 RMS에 맞춰 스케일 (자연스러운 음량)
    double out_rms = math::rms(output.data(), output_samples);
    std::cerr << "[Resamp] out_rms=" << out_rms << '\n';
    if (out_rms > 1e-10) {
        float scale = static_cast<float>(src_rms / out_rms);
        // 상한만 제한: 무음에 가까운 출력을 과도하게 증폭하지 않음
        // 하한 없음: 필터 게인이 높아도 필요한 만큼 scale-down 허용
        if (scale > 5.0f) scale = 5.0f;
        for (auto& s : output) s *= scale;
    } else {
        // 필터가 무음을 출력한 경우 — 소스를 직접 출력 (폴백)
        std::cerr << "[Resamp] Warning: KL filter produced silence! Falling back to source passthrough.\n";
        for (int i = 0; i < output_samples; ++i)
            output[i] = (i < static_cast<int>(source.size())) ? source[i] : 0.0f;
        // 소스 RMS로 정규화
        double s_rms = math::rms(output.data(), output_samples);
        if (s_rms > 1e-10) {
            float scale = static_cast<float>(src_rms / s_rms);
            for (auto& s : output) s *= scale;
        }
    }

    // 소프트 클리핑 (-0.9 ~ +0.9 범위로 제한)
    for (auto& s : output) {
        if (s >  1.0f) s =  1.0f;
        if (s < -1.0f) s = -1.0f;
    }

    return output;
}

} // namespace resamp::synth
