// Resamp — (Open)UTAU Source-Filter 리샘플러
// Kelly-Lochbaum 물리 성도 모델 기반
// UTAU 표준 CLI 프로토콜 호환
//
// 파이프라인:
//   CLI 파싱 → WAV/FRQ 읽기 → 트리밍 → LPC 분석
//   → 타깃 F0 컨투어 → 소스 생성 → KL 합성 → PSOLA 접합
//   → 볼륨/fade → WAV 출력

#include "args/arg_parser.hpp"
#include "io/wav_reader.hpp"
#include "io/wav_writer.hpp"
#include "io/frq_reader.hpp"
#include "flags/flag_parser.hpp"
#include "analysis/frame_slicer.hpp"
#include "analysis/lpc_analyzer.hpp"
#include "synth/pitch_mapper.hpp"
#include "synth/source_gen.hpp"
#include "synth/psola_splicer.hpp"
#include "post/volume.hpp"
#include "post/fade.hpp"
#include "util/math_util.hpp"

#include <iostream>
#include <random>
#include <stdexcept>
#include <algorithm>
#include <cmath>

int main(int argc, char** argv) {
    try {
        // ── 1. CLI 파싱 ────────────────────────────────────────────────
        resamp::RenderParams params = resamp::parse_args(argc, argv);

        int sample_rate = 44100; // 기본값; WAV 로드 후 갱신

        // ── 2. 소스 WAV 읽기 ───────────────────────────────────────────
        std::cerr << "[Resamp] in=" << params.input_wav
                  << " pitch=" << params.pitch_str
                  << " vel=" << params.velocity
                  << " flags=" << params.flags
                  << " off=" << params.offset_ms
                  << " len=" << params.length_ms
                  << " con=" << params.consonant_ms
                  << " cut=" << params.cutoff_ms
                  << " vol=" << params.volume << '\n';
        resamp::io::WavInfo wav_info;
        auto signal = resamp::io::load_wav(params.input_wav, wav_info);
        sample_rate = static_cast<int>(wav_info.sample_rate);
        std::cerr << "[Resamp] WAV: sr=" << sample_rate
                  << " ch=" << wav_info.num_channels
                  << " bits=" << wav_info.bits_per_sample
                  << " samples=" << wav_info.num_samples << '\n';

        // ── 3. FRQ 읽기 ────────────────────────────────────────────────
        auto frq = resamp::io::load_frq(params.input_wav);

        // ── 4. 소스 트리밍 (offset_ms, cutoff_ms 적용) ──────────────
        int src_start = static_cast<int>(params.offset_ms * sample_rate / 1000.0);
        src_start = std::max(0, std::min(src_start, static_cast<int>(signal.size())));

        int src_end;
        if (params.cutoff_ms < 0.0) {
            // 음수: 끝에서부터
            src_end = static_cast<int>(signal.size())
                    + static_cast<int>(params.cutoff_ms * sample_rate / 1000.0);
        } else if (params.cutoff_ms > 0.0) {
            src_end = src_start + static_cast<int>(params.cutoff_ms * sample_rate / 1000.0);
        } else {
            src_end = static_cast<int>(signal.size());
        }
        src_end = std::max(src_start + 1, std::min(src_end, static_cast<int>(signal.size())));

        std::vector<float> trimmed(signal.begin() + src_start,
                                   signal.begin() + src_end);
        if (trimmed.empty()) trimmed.push_back(0.0f);

        // FRQ 프레임 오프셋 조정
        // (frq_get_f0는 트리밍 전 원본 인덱스를 받으므로 오프셋을 더함)
        // 간단히 frq를 그대로 사용하되, 오프셋 보정은 frame_slicer에서 처리

        // ── 5. 출력 길이 결정 ─────────────────────────────────────────
        int output_samples = static_cast<int>(params.length_ms * sample_rate / 1000.0);
        if (output_samples < 1) output_samples = 1;

        // ── 6. 플래그 파싱 ────────────────────────────────────────────
        resamp::SynthParams sp = resamp::parse_flags(params.flags);

        // ── 7. LPC 분석 + 프레임 분할 ─────────────────────────────────
        // 포먼트 워핑 (g 플래그) 적용을 위한 warp_lambda 계산
        // g=-100 → +0.33 (성도 길어짐=포먼트 하강), g=+100 → -0.33 (포먼트 상승)
        float warp_lambda = -sp.gender / 300.0f;

        auto frames = resamp::analysis::slice_and_analyze(
            trimmed, frq,
            sample_rate,
            1024,  // frame_size
            256,   // hop_size
            0      // lpc_order=자동
        );

        // g 플래그: 모든 프레임에 포먼트 워핑 적용
        if (std::fabs(warp_lambda) > 0.005f) {
            for (auto& f : frames)
                f.lpc = resamp::analysis::warp_lpc(f.lpc, warp_lambda);
        }

        // ── 8. 타깃 F0 컨투어 생성 ────────────────────────────────────
        auto f0_contour = resamp::synth::make_f0_contour(params, output_samples, sample_rate);

        // ── 9. PSOLA 합성 ─────────────────────────────────────────────
        auto output = resamp::synth::psola_splice(
            trimmed, frames, f0_contour,
            params, sp,
            sample_rate, output_samples
        );

        // ── 10. 후처리 ────────────────────────────────────────────────
        // N 플래그: 배경 노이즈 혼합
        if (sp.noise_level > 0) {
            float noise_gain = sp.noise_level / 200.0f;
            float out_rms = static_cast<float>(
                resamp::math::rms(output.data(), static_cast<int>(output.size())));
            std::mt19937 rng_n(12345);
            std::uniform_real_distribution<float> ud(-1.0f, 1.0f);
            for (auto& s : output)
                s += ud(rng_n) * out_rms * noise_gain;
        }

        // B 플래그 (brightness): 1차 IIR으로 스펙트럼 기울기 조정
        if (sp.brightness != 50) {
            float alpha = (sp.brightness > 50)
                ? 0.85f + (sp.brightness - 50) / 500.0f  // HF 부스트 → 더 통과
                : 0.5f + sp.brightness / 100.0f;          // LF 부스트 → 더 스무딩
            alpha = resamp::math::clamp(alpha, 0.1f, 0.99f);
            // HF 강조: 원본 - 스무딩
            if (sp.brightness > 50) {
                std::vector<float> smooth(output.size());
                float prev = 0.0f;
                for (size_t i = 0; i < output.size(); ++i) {
                    smooth[i] = prev = prev + (1.0f - alpha) * (output[i] - prev);
                }
                float boost = (sp.brightness - 50) / 50.0f * 0.5f;
                for (size_t i = 0; i < output.size(); ++i)
                    output[i] += (output[i] - smooth[i]) * boost;
            }
        }

        // 볼륨 스케일 + 피크 제한
        resamp::post::apply_volume(output, params.volume, sp);

        // Fade in/out (5ms)
        resamp::post::apply_fades(output, 5.0, 5.0, sample_rate);

        // ── 11. WAV 저장 ──────────────────────────────────────────────
        resamp::io::save_wav(params.output_wav, output,
                             static_cast<uint32_t>(sample_rate));

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "[Resamp] Error: " << e.what() << '\n';
        return 1;
    }
}
