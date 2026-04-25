#include "psola_splicer.hpp"
#include "analysis/frame_slicer.hpp"
#include "analysis/lpc_analyzer.hpp"
#include "util/math_util.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace resamp::synth {

static double compute_src_position(
    int out_sample, const RenderParams& params, int N_src, int sample_rate)
{
    double consonant_scale = params.velocity / 100.0;
    if (consonant_scale < 0.01) consonant_scale = 0.01;
    int consonant_src_smp = static_cast<int>(params.consonant_ms * sample_rate / 1000.0);
    int consonant_tgt_smp = static_cast<int>(consonant_src_smp * consonant_scale);
    if (out_sample <= consonant_tgt_smp) {
        return out_sample / consonant_scale;
    } else {
        int vowel_offset  = out_sample - consonant_tgt_smp;
        int vowel_src_len = N_src - consonant_src_smp;
        if (vowel_src_len <= 0) return consonant_src_smp;
        return consonant_src_smp + vowel_offset % vowel_src_len;
    }
}

static double get_f0_at_sample(
    const std::vector<analysis::AnalysisFrame>& frames, int sample)
{
    if (frames.empty()) return 0.0;
    int lo = 0, hi = static_cast<int>(frames.size()) - 1;
    while (lo < hi) {
        int mid = (lo + hi) >> 1;
        if (frames[mid].center_sample < sample) lo = mid + 1;
        else hi = mid;
    }
    if (lo > 0 && std::abs(frames[lo-1].center_sample - sample) <=
                  std::abs(frames[lo  ].center_sample - sample)) --lo;
    return frames[lo].f0;
}

// ── Simple TD-PSOLA ──────────────────────────────────────────────────────
//
// 원본 신호 grain을 직접 OLA. 리샘플링·LPC 합성 필터 모두 사용 안 함.
// → 포먼트는 원본 파형이 자연 반영되어 보존됨 (Mickey-Mouse 효과 없음)
// → IIR 없으므로 ringing/buzzing 아티팩트 없음
//
// Pitch UP  (f_tgt > f_src): grain = 2·T_src (소스 1주기 포함)
// Pitch DOWN (f_tgt ≤ f_src): grain = 2·T_tgt (50% COLA)
// → wsum 정규화가 비 50% 오버랩 자동 보정
//
// Gender 플래그: per-grain LPC 분석 + 워핑 (raw grain에 직접 적용,
//   리샘플링 없으므로 스펙트럼 불일치로 인한 IIR 발산 위험 낮음)
std::vector<float> psola_splice(
    const std::vector<float>&                   signal,
    const std::vector<analysis::AnalysisFrame>& frames,
    const std::vector<double>&                  f0_contour,
    const RenderParams&                         params,
    const SynthParams&                          sp,
    int                                         sample_rate,
    int                                         output_samples)
{
    if (params.target_hz <= 0.0)
        return std::vector<float>(output_samples, 0.0f);

    int    N_src   = static_cast<int>(signal.size());
    double src_rms = math::rms(signal.data(), N_src);
    if (src_rms < 1e-6) src_rms = 0.1;

    float warp_lambda = -sp.gender / 300.0f;
    bool  do_warp     = std::fabs(warp_lambda) > 0.005f;

    std::cerr << "[Resamp] TD-PSOLA N_src=" << N_src
              << " output=" << output_samples
              << " warp=" << warp_lambda << '\n';

    // ── 소스 피치 마크 구성 ───────────────────────────────────────────────
    std::vector<int> src_marks;
    {
        const int n_frames = static_cast<int>(frames.size());
        int fi = 0;
        double pos = 0.0;
        while (static_cast<int>(std::round(pos)) < N_src) {
            src_marks.push_back(static_cast<int>(std::round(pos)));
            while (fi + 1 < n_frames &&
                   std::abs(frames[fi+1].center_sample - (int)pos) <
                   std::abs(frames[fi  ].center_sample - (int)pos)) ++fi;
            double f0h = (fi < n_frames && frames[fi].f0 >= 50.0) ? frames[fi].f0 : 0.0;
            pos += (f0h > 0.0) ? (sample_rate / f0h) : (sample_rate * 0.005);
        }
    }
    std::cerr << "[Resamp] src_marks=" << src_marks.size() << '\n';

    std::vector<float> output(output_samples, 0.0f);
    std::vector<float> wsum  (output_samples, 0.0f);

    const int LPC_ORDER = 16;
    double phase = 0.5;

    for (int i = 0; i < output_samples; ++i) {
        double f0 = (i < (int)f0_contour.size()) ? f0_contour[i] : params.target_hz;
        if (f0 < 10.0) f0 = params.target_hz;
        if (f0 < 20.0) { phase = 0.5; continue; }

        phase += f0 / sample_rate;
        if (phase < 1.0) continue;
        phase -= 1.0;

        double src_d = compute_src_position(i, params, N_src, sample_rate);
        src_d = math::clamp(src_d, 0.0, (double)(N_src - 1));
        int src_c = (int)std::round(src_d);

        // 소스 피치 마크 스냅
        if (!src_marks.empty()) {
            auto it = std::lower_bound(src_marks.begin(), src_marks.end(), src_c);
            if      (it == src_marks.end())   src_c = src_marks.back();
            else if (it == src_marks.begin()) src_c = *it;
            else {
                auto pi = std::prev(it);
                src_c = (std::abs(*it - src_c) < std::abs(*pi - src_c)) ? *it : *pi;
            }
            src_c = (int)math::clamp((double)src_c, 0.0, (double)(N_src-1));
        }

        // 윈도우 크기 결정
        int hw_tgt = std::max(32, std::min(4096, (int)std::round(sample_rate / f0)));
        double f0_src = get_f0_at_sample(frames, src_c);
        bool voiced = (f0_src >= 50.0);
        int hw_src = voiced
            ? std::max(32, std::min(4096, (int)std::round(sample_rate / f0_src)))
            : hw_tgt;
        int hw_g   = std::max(hw_tgt, hw_src);
        int win_lg = 2 * hw_g;

        // ── 일반 경로: 원본 샘플 직접 OLA (포먼트 자연 보존) ───────────
        if (!do_warp) {
            for (int d = -hw_g; d <= hw_g; ++d) {
                int oi = i + d;
                int si = src_c + d;
                if (oi < 0 || oi >= output_samples) continue;
                if (si < 0 || si >= N_src)          continue;
                float w = (float)(0.5*(1.0-std::cos(2.0*math::PI*(d+hw_g)/win_lg)));
                output[oi] += signal[si] * w;
                wsum[oi]   += w;
            }
            continue;
        }

        // ── 젠더 플래그: per-grain LPC 워핑 ──────────────────────────────
        int grain_len = win_lg + 1;
        std::vector<float> grain(grain_len, 0.0f);
        for (int d = -hw_g; d <= hw_g; ++d) {
            int si = src_c + d;
            if (si >= 0 && si < N_src) grain[d + hw_g] = signal[si];
        }

        std::vector<float> grain_w(grain_len);
        for (int k = 0; k < grain_len; ++k)
            grain_w[k] = grain[k] * (float)(0.5*(1.0-std::cos(2.0*math::PI*k/win_lg)));

        auto lpc_g = analysis::analyze_lpc(grain_w.data(), grain_len, LPC_ORDER, sample_rate);

        if (lpc_g.order > 0 && lpc_g.gain > 1e-10f) {
            auto lpc_w = analysis::warp_lpc(lpc_g, warp_lambda);
            int P = lpc_g.order;

            std::vector<float> residual(grain_len, 0.0f);
            for (int n = 0; n < grain_len; ++n) {
                float e = grain[n];
                for (int k = 0; k < P && n-1-k >= 0; ++k)
                    e += lpc_g.a[k] * grain[n-1-k];
                residual[n] = e;
            }

            std::vector<float> grain_out(grain_len, 0.0f);
            for (int n = 0; n < grain_len; ++n) {
                float s = residual[n];
                for (int k = 0; k < P && n-1-k >= 0; ++k)
                    s -= lpc_w.a[k] * grain_out[n-1-k];
                grain_out[n] = std::isfinite(s) ? s : grain[n];
            }

            for (int d = -hw_g; d <= hw_g; ++d) {
                int oi = i + d;
                if (oi < 0 || oi >= output_samples) continue;
                int gi = d + hw_g;
                float w = (float)(0.5*(1.0-std::cos(2.0*math::PI*gi/win_lg)));
                output[oi] += grain_out[gi] * w;
                wsum[oi]   += w;
            }
        } else {
            // LPC 실패 시 raw grain 직접 OLA
            for (int d = -hw_g; d <= hw_g; ++d) {
                int oi = i + d;
                if (oi < 0 || oi >= output_samples) continue;
                int gi = d + hw_g;
                float w = (float)(0.5*(1.0-std::cos(2.0*math::PI*gi/win_lg)));
                output[oi] += grain[gi] * w;
                wsum[oi]   += w;
            }
        }
    }

    // OLA 정규화
    for (int i = 0; i < output_samples; ++i)
        if (wsum[i] > 1e-10f) output[i] /= wsum[i];

    // RMS 맞춤
    double out_rms = math::rms(output.data(), output_samples);
    std::cerr << "[Resamp] PSOLA out_rms=" << out_rms << " src_rms=" << src_rms << '\n';
    if (out_rms > 1e-10) {
        float scale = (float)std::min(5.0, src_rms / out_rms);
        for (auto& s : output) s *= scale;
    } else {
        std::cerr << "[Resamp] Warning: silence fallback\n";
        for (int i = 0; i < output_samples; ++i) {
            double sd = compute_src_position(i, params, N_src, sample_rate);
            output[i] = signal[(int)math::clamp(sd, 0.0, (double)(N_src-1))];
        }
    }
    return output;
}

} // namespace resamp::synth
