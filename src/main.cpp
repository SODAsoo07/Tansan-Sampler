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
#include <fstream>
#include <cstdlib>

int main(int argc, char** argv) {
    try {
        auto env_enabled = [](const char* name, bool default_value) {
            const char* v = std::getenv(name);
            if (v == nullptr || *v == '\0') return default_value;
            std::string s(v);
            std::transform(s.begin(), s.end(), s.begin(),
                           [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            if (s == "0" || s == "false" || s == "off" || s == "no") return false;
            if (s == "1" || s == "true" || s == "on" || s == "yes") return true;
            return default_value;
        };
        const bool verbose_log = env_enabled("RESAMP_VERBOSE", false);
        auto append_debug_log = [&](const std::string& line) {
            const char* path = std::getenv("RESAMP_DEBUG_LOG");
            if (path == nullptr || *path == '\0') return;
            std::ofstream ofs(path, std::ios::out | std::ios::app);
            if (!ofs) return;
            ofs << line << '\n';
        };

        // ── 1. CLI 파싱 ────────────────────────────────────────────────
        resamp::RenderParams params = resamp::parse_args(argc, argv);
        int sample_rate = 44100;

        {
            std::string argv_joined;
            for (int i = 0; i < argc; ++i) {
                if (i > 0) argv_joined += " | ";
                argv_joined += argv[i] ? argv[i] : "";
            }
            append_debug_log("[ARGS] argc=" + std::to_string(argc) +
                             " argv=" + argv_joined);
            append_debug_log("[PARSED] flags=\"" + params.flags +
                             "\" off=" + std::to_string(params.offset_ms) +
                             " len=" + std::to_string(params.length_ms) +
                             " con=" + std::to_string(params.consonant_ms) +
                             " cut=" + std::to_string(params.cutoff_ms));
        }

        // ── 2. WAV 읽기 ────────────────────────────────────────────────
        if (verbose_log) {
            std::cerr << "[Resamp] in=" << params.input_wav
                      << " pitch=" << params.pitch_str
                      << " vel=" << params.velocity
                      << " flags=" << params.flags
                      << " off=" << params.offset_ms
                      << " len=" << params.length_ms
                      << " con=" << params.consonant_ms
                      << " cut=" << params.cutoff_ms
                      << " vol=" << params.volume << '\n';
        }
        resamp::io::WavInfo wav_info;
        auto signal = resamp::io::load_wav(params.input_wav, wav_info);
        sample_rate = static_cast<int>(wav_info.sample_rate);
        if (verbose_log) {
            std::cerr << "[Resamp] WAV: sr=" << sample_rate
                      << " ch=" << wav_info.num_channels
                      << " samples=" << wav_info.num_samples << '\n';
        }

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
        const std::string expanded_flags =
            resamp::expand_flag_presets(params.flags, argc > 0 && argv[0] ? argv[0] : "");
        if (expanded_flags != params.flags) {
            append_debug_log("[FLAGS_EXPANDED] raw=\"" + params.flags +
                             "\" expanded=\"" + expanded_flags + "\"");
            if (verbose_log) {
                std::cerr << "[Resamp] flags expanded: raw=" << params.flags
                          << " expanded=" << expanded_flags << '\n';
            }
        }
        resamp::SynthParams sp = resamp::parse_flags(expanded_flags);
        if (verbose_log) {
            std::cerr << "[Resamp] flags parsed:"
                      << " g=" << sp.gender
                      << " Bi=" << sp.brightness
                      << " Hu=" << sp.husky_tone
                      << " Mo=" << sp.mouth_open
                      << " Tn=" << sp.tension
                      << " t=" << sp.pitch_cents
                      << " Gr=" << sp.growl
                      << " Vg=" << sp.voiced_growl
                      << " Vtl=" << sp.tract_length
                      << " Vtr=" << sp.tract_resonance
                      << " Vtw=" << sp.tract_focus
                      << " Vc=" << sp.tract_constriction
                      << " Nn=" << sp.nasal_coupling
                      << " Hr=" << sp.harmonics
                      << " N=" << sp.noise_level
                      << " Bh=" << sp.breathiness
                      << " Cs=" << sp.consonant_stability
                      << " At=" << sp.attack
                      << " Ns=" << sp.noise_color
                      << " P=" << sp.peak_comp
                      << " Ln=" << sp.loud_norm
                      << " Lp=" << sp.loop_mode
                      << " Cw=" << sp.consonant_power
                      << " Vw=" << sp.vowel_power
                      << " Rv=" << sp.reverse_mode
                      << " Vo=" << sp.voicing
                      << " Fc=" << sp.final_filter
                      << " Eb=" << sp.end_breath
                      << " Fh=" << sp.fry_head
                      << " Ft=" << sp.fry_tail
                      << " Tm=" << sp.tremolo
                      << " Ds=" << sp.distortion
                      << " Bc=" << sp.bitcrusher
                      << " Vz=" << sp.vocalizer
                      << " VzS=" << sp.vocalizer_strength << '\n';
        }
        append_debug_log("[FLAGS] Vtl=" + std::to_string(sp.tract_length) +
                         " Vtr=" + std::to_string(sp.tract_resonance) +
                         " Vtw=" + std::to_string(sp.tract_focus) +
                         " Vc=" + std::to_string(sp.tract_constriction) +
                         " Nn=" + std::to_string(sp.nasal_coupling) +
                         " Mo=" + std::to_string(sp.mouth_open) +
                         " Tn=" + std::to_string(sp.tension) +
                         " Vz=" + std::to_string(sp.vocalizer) +
                         " VzS=" + std::to_string(sp.vocalizer_strength));

        // ── 6. WORLD 분석 (raw Harvest F0 + envelope/AP 추출) ─────────
        // 디스크 캐시를 사용해 반복 렌더 시 분석 비용을 절감.
        const bool track_formants =
            std::abs(sp.mouth_open) > 0 ||
            std::abs(sp.tract_length) > 0 ||
            std::abs(sp.tract_resonance) > 0 ||
            std::abs(sp.tract_focus) > 0 ||
            (sp.vocalizer > 0 && sp.vocalizer_strength > 0);
        append_debug_log(std::string("[ANALYSIS] formants=") +
                         (track_formants ? "tracked" : "default"));
        if (verbose_log) {
            std::cerr << "[Resamp] analysis formants="
                      << (track_formants ? "tracked" : "default") << '\n';
        }
        auto wa = resamp::synth::world_analyze_cached(
            trimmed, sample_rate, params.input_wav, src_start, src_end, track_formants);

        // ── 7. 타겟 F0 컨투어 ─────────────────────────────────────────
        auto f0_contour = resamp::synth::make_f0_contour(
            params, sp, output_samples, sample_rate);

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

        // 역재생/디스토션/비트크러셔/최종 컷 필터.
        // Fc 하이컷/로우컷은 apply_flag_post_effects 내부에서 항상 마지막에 적용된다.
        double consonant_tgt_ms = params.consonant_ms *
            std::clamp(100.0 / static_cast<double>(std::max(1, params.velocity)), 0.25, 4.0);
        resamp::post::apply_flag_post_effects(output, sample_rate, consonant_tgt_ms, sp);

        // ── 10. WAV 저장 ──────────────────────────────────────────────
        resamp::io::save_wav(params.output_wav, output,
                             static_cast<uint32_t>(sample_rate));
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "[Resamp] Error: " << e.what() << '\n';
        return 1;
    }
}
