// Resamp — (Open)UTAU WORLD-기반 리샘플러
// 성도 시뮬레이션 정체성: spectral envelope = 성도 전달 함수
//
// 파이프라인:
//   CLI → WAV 로드 → 트리밍 → WORLD 분석 (DIO/StoneMask/CheapTrick/D4C)
//   → 타겟 F0 컨투어 → WORLD 합성 (시간 매핑 + 플래그 변환) → 후처리 → WAV 출력

#include "args/arg_parser.hpp"
#include "io/wav_reader.hpp"
#include "io/wav_writer.hpp"
#include "flags/flag_parser.hpp"
#include "synth/pitch_mapper.hpp"
#include "synth/world_synth.hpp"
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
        int sample_rate = 44100;

        // ── 2. WAV 읽기 ────────────────────────────────────────────────
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
                  << " samples=" << wav_info.num_samples << '\n';

        // ── 3. 소스 트리밍 (offset_ms, cutoff_ms) ─────────────────────
        int src_start = static_cast<int>(params.offset_ms * sample_rate / 1000.0);
        src_start = std::max(0, std::min(src_start, static_cast<int>(signal.size())));

        int src_end;
        if (params.cutoff_ms < 0.0) {
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
        if (trimmed.size() < 32) trimmed.resize(32, 0.0f);

        // ── 4. 출력 길이 ──────────────────────────────────────────────
        int output_samples = static_cast<int>(params.length_ms * sample_rate / 1000.0);
        if (output_samples < 1) output_samples = 1;

        // ── 5. 플래그 파싱 ────────────────────────────────────────────
        resamp::SynthParams sp = resamp::parse_flags(params.flags);
        std::cerr << "[Resamp] flags parsed:"
                  << " g=" << sp.gender
                  << " Bi=" << sp.brightness
                  << " Hu=" << sp.husky_tone
                  << " Mo=" << sp.mouth_open
                  << " t=" << sp.tension
                  << " H=" << sp.harmonics
                  << " N=" << sp.noise_level
                  << " Bh=" << sp.breathiness
                  << " Tr=" << sp.transition_length
                  << " Cs=" << sp.consonant_stability
                  << " At=" << sp.attack
                  << " Rl=" << sp.release_air
                  << " Ns=" << sp.noise_color
                  << " P=" << sp.peak_comp
                  << " c=" << sp.voice_color << '\n';

        // ── 6. WORLD 분석 (raw Harvest F0 + envelope/AP 추출) ─────────
        auto wa = resamp::synth::world_analyze(trimmed, sample_rate);

        // ── 7. 타겟 F0 컨투어 ─────────────────────────────────────────
        auto f0_contour = resamp::synth::make_f0_contour(
            params, output_samples, sample_rate);

        // ── 8. WORLD 합성 ─────────────────────────────────────────────
        auto output = resamp::synth::world_render(
            wa, f0_contour, params, sp, output_samples);

        // ── 9. 후처리 ────────────────────────────────────────────────
        // B 플래그는 envelope에서 처리됨 (world_synth 내부)
        // 노이즈 추가 (N 플래그)는 envelope 단계에서 AP로 처리됨

        // 볼륨 스케일 + 피크 제한 (P 플래그)
        resamp::post::apply_volume(output, params.volume, sp);

        // Fade in/out:
        // 과도한 fade-in은 어두 자음 attack을 깎아 "툭 끊기는" 인상을 줄 수 있어 축소.
        resamp::post::apply_fades(output, 1.0, 4.0, sample_rate);

        // ── 10. WAV 저장 ──────────────────────────────────────────────
        resamp::io::save_wav(params.output_wav, output,
                             static_cast<uint32_t>(sample_rate));
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "[Resamp] Error: " << e.what() << '\n';
        return 1;
    }
}
