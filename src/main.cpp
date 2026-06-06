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
#include <cctype>
#include <cstdint>
#include <fstream>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#ifdef _WIN32
#include <filesystem>
#include <windows.h>
#include <shellapi.h>
#endif

namespace {

#ifdef _WIN32
std::string wide_to_utf8(const wchar_t* ws) {
    if (ws == nullptr || *ws == L'\0') return {};
    int size = WideCharToMultiByte(CP_UTF8, 0, ws, -1, nullptr, 0, nullptr, nullptr);
    if (size <= 1) return {};
    std::string out(static_cast<size_t>(size - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, ws, -1, out.data(), size, nullptr, nullptr);
    return out;
}

std::vector<std::string> get_utf8_command_line_args(int argc, char** argv) {
    int wide_argc = 0;
    LPWSTR* wide_argv = CommandLineToArgvW(GetCommandLineW(), &wide_argc);
    if (wide_argv != nullptr && wide_argc > 0) {
        std::vector<std::string> args;
        args.reserve(static_cast<size_t>(wide_argc));
        for (int i = 0; i < wide_argc; ++i) {
            args.push_back(wide_to_utf8(wide_argv[i]));
        }
        LocalFree(wide_argv);
        return args;
    }

    std::vector<std::string> args;
    args.reserve(static_cast<size_t>(std::max(0, argc)));
    for (int i = 0; i < argc; ++i) args.emplace_back(argv[i] ? argv[i] : "");
    return args;
}

std::filesystem::path path_from_utf8(const std::string& path) {
    return std::filesystem::u8path(path);
}
#else
std::vector<std::string> get_utf8_command_line_args(int argc, char** argv) {
    std::vector<std::string> args;
    args.reserve(static_cast<size_t>(std::max(0, argc)));
    for (int i = 0; i < argc; ++i) args.emplace_back(argv[i] ? argv[i] : "");
    return args;
}
#endif

bool env_enabled(const char* name, bool default_value) {
    const char* v = std::getenv(name);
    if (v == nullptr || *v == '\0') return default_value;
    std::string s(v);
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (s == "0" || s == "false" || s == "off" || s == "no") return false;
    if (s == "1" || s == "true" || s == "on" || s == "yes") return true;
    return default_value;
}

double env_double_clamped(const char* name, double default_value, double lo, double hi) {
    const char* v = std::getenv(name);
    if (v == nullptr || *v == '\0') return default_value;
    try {
        double parsed = std::stod(v);
        if (!std::isfinite(parsed)) return default_value;
        return std::clamp(parsed, lo, hi);
    } catch (...) {
        return default_value;
    }
}

void append_debug_log(const std::string& line) {
    const char* path = std::getenv("RESAMP_DEBUG_LOG");
    if (path == nullptr || *path == '\0') return;
#ifdef _WIN32
    std::ofstream ofs(path_from_utf8(path), std::ios::out | std::ios::app);
#else
    std::ofstream ofs(path, std::ios::out | std::ios::app);
#endif
    if (!ofs) return;
    ofs << line << '\n';
}

void write_u16(std::ofstream& f, uint16_t v) {
    uint8_t b[2] = { static_cast<uint8_t>(v & 0xFF),
                     static_cast<uint8_t>((v >> 8) & 0xFF) };
    f.write(reinterpret_cast<char*>(b), 2);
}

void write_u32(std::ofstream& f, uint32_t v) {
    uint8_t b[4] = { static_cast<uint8_t>(v & 0xFF),
                     static_cast<uint8_t>((v >> 8)  & 0xFF),
                     static_cast<uint8_t>((v >> 16) & 0xFF),
                     static_cast<uint8_t>((v >> 24) & 0xFF) };
    f.write(reinterpret_cast<char*>(b), 4);
}

bool write_silence_wav_streamed(const std::string& path, int64_t requested_samples, uint32_t sample_rate) {
    if (path.empty()) return false;
    int64_t max_samples = (static_cast<int64_t>(std::numeric_limits<uint32_t>::max()) - 36) / 2;
    int64_t num_samples_i64 = std::clamp<int64_t>(requested_samples, 1, max_samples);
    uint32_t num_samples = static_cast<uint32_t>(num_samples_i64);
    uint16_t num_channels = 1;
    uint16_t bits_per_sample = 16;
    uint32_t byte_rate = sample_rate * num_channels * bits_per_sample / 8;
    uint16_t block_align = static_cast<uint16_t>(num_channels * bits_per_sample / 8);
    uint32_t data_size = num_samples * block_align;

#ifdef _WIN32
    std::ofstream f(path_from_utf8(path), std::ios::binary | std::ios::trunc);
#else
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
#endif
    if (!f) return false;
    f.write("RIFF", 4);
    write_u32(f, 36 + data_size);
    f.write("WAVE", 4);
    f.write("fmt ", 4);
    write_u32(f, 16);
    write_u16(f, 1);
    write_u16(f, num_channels);
    write_u32(f, sample_rate);
    write_u32(f, byte_rate);
    write_u16(f, block_align);
    write_u16(f, bits_per_sample);
    f.write("data", 4);
    write_u32(f, data_size);

    char zeros[8192] = {};
    uint32_t remaining = data_size;
    while (remaining > 0) {
        uint32_t chunk = std::min<uint32_t>(remaining, static_cast<uint32_t>(sizeof(zeros)));
        f.write(zeros, chunk);
        remaining -= chunk;
    }
    return f.good();
}

int fail_with_silence(const resamp::RenderParams& params,
                      int64_t output_samples,
                      uint32_t sample_rate,
                      const std::string& reason) {
    std::cerr << "[Resamp] " << reason << '\n';
    append_debug_log("[FALLBACK_SILENCE] " + reason);
    if (write_silence_wav_streamed(params.output_wav, output_samples, sample_rate)) {
        return 0;
    }
    std::cerr << "[Resamp] Failed to write fallback silence WAV\n";
    return 1;
}

bool render_memory_budget_exceeded(int output_samples,
                                   int sample_rate,
                                   int analysis_frames,
                                   int fft_size,
                                   double max_mb,
                                   std::string& reason) {
    if (output_samples < 1 || sample_rate <= 0 || analysis_frames < 1 || fft_size <= 0) return false;
    int64_t spec_dim = static_cast<int64_t>(fft_size / 2 + 1);
    double worst_frame_period_ms = 0.5; // 품질 저하는 하지 않고, 최악 경로 기준으로만 예산 산정.
    int64_t out_frames = static_cast<int64_t>(
        std::ceil(static_cast<double>(output_samples) * 1000.0 /
                  (worst_frame_period_ms * static_cast<double>(sample_rate)))) + 2;
    long double analysis_bytes = 2.0L * analysis_frames * spec_dim * sizeof(double);
    long double render_bytes = 2.0L * out_frames * spec_dim * sizeof(double);
    long double output_bytes = static_cast<long double>(output_samples) * (sizeof(double) + sizeof(float));
    long double scratch_bytes = static_cast<long double>(spec_dim) * sizeof(double) * 24.0L;
    long double total_mb = (analysis_bytes + render_bytes + output_bytes + scratch_bytes) / (1024.0L * 1024.0L);
    if (total_mb > max_mb) {
        reason = "OOM guard: estimated render memory " + std::to_string(static_cast<int>(total_mb)) +
                 " MB exceeds limit " + std::to_string(static_cast<int>(max_mb)) + " MB";
        return true;
    }
    return false;
}

} // namespace

int main(int argc, char** argv) {
    resamp::RenderParams params;
    int sample_rate = 44100;
    int64_t requested_output_samples = 1;
    try {
        const bool verbose_log = env_enabled("RESAMP_VERBOSE", false);
        std::vector<std::string> utf8_args = get_utf8_command_line_args(argc, argv);
        std::vector<char*> utf8_argv;
        utf8_argv.reserve(utf8_args.size());
        for (std::string& arg : utf8_args) {
            utf8_argv.push_back(const_cast<char*>(arg.c_str()));
        }
        argc = static_cast<int>(utf8_argv.size());
        argv = utf8_argv.data();

        // ── 1. CLI 파싱 ────────────────────────────────────────────────
        params = resamp::parse_args(argc, argv);
        if (!std::isfinite(params.length_ms) || params.length_ms <= 0.0 ||
            !std::isfinite(params.offset_ms) || !std::isfinite(params.consonant_ms) ||
            !std::isfinite(params.cutoff_ms)) {
            return fail_with_silence(params, requested_output_samples, static_cast<uint32_t>(sample_rate),
                                     "Invalid timing parameter; wrote fallback silence");
        }

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

        double max_note_seconds = env_double_clamped("RESAMP_MAX_NOTE_SECONDS", 45.0, 1.0, 600.0);
        if (params.length_ms > max_note_seconds * 1000.0) {
            requested_output_samples = static_cast<int64_t>(
                std::ceil(max_note_seconds * static_cast<double>(sample_rate)));
            return fail_with_silence(params, requested_output_samples, static_cast<uint32_t>(sample_rate),
                                     "Render length guard: note length exceeds RESAMP_MAX_NOTE_SECONDS");
        }
        requested_output_samples = static_cast<int64_t>(
            std::ceil(params.length_ms * static_cast<double>(sample_rate) / 1000.0));
        if (requested_output_samples > static_cast<int64_t>(std::numeric_limits<int>::max())) {
            return fail_with_silence(params, static_cast<int64_t>(sample_rate), static_cast<uint32_t>(sample_rate),
                                     "Render length guard: output sample count exceeds int range");
        }

        // ── 3. 소스 트리밍 (offset_ms, cutoff_ms) ─────────────────────
        // UTAU식 offset..cutoff 전체가 너무 길면 VCV/CVVC 녹음의 다음 발음까지
        // WORLD 분석에 들어간다. 실제 합성 안정 구간만 분석하도록 하드캡하되,
        // CheapTrick/D4C 경계 안정성을 위해 offset 앞과 cap 뒤에 작은 분석 여유를 둔다.
        int playback_start = static_cast<int>(params.offset_ms * sample_rate / 1000.0);
        playback_start = std::max(0, std::min(playback_start, static_cast<int>(signal.size())));

        int playback_end;
        if (params.cutoff_ms < 0.0) {
            playback_end = static_cast<int>(signal.size())
                    + static_cast<int>(params.cutoff_ms * sample_rate / 1000.0);
        } else if (params.cutoff_ms > 0.0) {
            playback_end = playback_start + static_cast<int>(params.cutoff_ms * sample_rate / 1000.0);
        } else {
            playback_end = static_cast<int>(signal.size());
        }
        playback_end = std::max(playback_start + 1, std::min(playback_end, static_cast<int>(signal.size())));

        double analysis_preroll_ms = env_double_clamped("RESAMP_ANALYSIS_PREROLL_MS", 5.0, 0.0, 30.0);
        double analysis_tail_cap_ms = env_double_clamped("RESAMP_ANALYSIS_TAIL_CAP_MS", 360.0, 80.0, 800.0);
        double analysis_postroll_ms = env_double_clamped("RESAMP_ANALYSIS_POSTROLL_MS", 24.0, 0.0, 90.0);

        double fixed_end_ms = std::max(0.0, params.offset_ms) + std::max(0.0, params.consonant_ms);
        double hard_end_ms = fixed_end_ms + analysis_tail_cap_ms + analysis_postroll_ms;
        int hard_end = static_cast<int>(std::ceil(hard_end_ms * sample_rate / 1000.0));
        int src_end = std::min(playback_end, std::max(playback_start + 32, hard_end));
        src_end = std::max(playback_start + 1, std::min(src_end, static_cast<int>(signal.size())));

        int preroll = static_cast<int>(std::round(analysis_preroll_ms * sample_rate / 1000.0));
        int src_start = std::max(0, playback_start - preroll);
        params.source_origin_ms =
            (playback_start - src_start) * 1000.0 / static_cast<double>(std::max(1, sample_rate));

        double max_source_seconds = env_double_clamped("RESAMP_MAX_SOURCE_SECONDS", 30.0, 1.0, 600.0);
        int64_t source_range_samples = static_cast<int64_t>(src_end) - static_cast<int64_t>(src_start);
        if (source_range_samples > static_cast<int64_t>(max_source_seconds * sample_rate)) {
            return fail_with_silence(params, requested_output_samples, static_cast<uint32_t>(sample_rate),
                                     "Source length guard: trimmed source exceeds RESAMP_MAX_SOURCE_SECONDS");
        }

        std::vector<float> trimmed(signal.begin() + src_start,
                                   signal.begin() + src_end);
        if (trimmed.size() < 32) trimmed.resize(32, 0.0f);
        append_debug_log("[TRIM] playback_start=" + std::to_string(playback_start) +
                         " playback_end=" + std::to_string(playback_end) +
                         " analysis_start=" + std::to_string(src_start) +
                         " analysis_end=" + std::to_string(src_end) +
                         " origin_ms=" + std::to_string(params.source_origin_ms) +
                         " hard_tail_cap_ms=" + std::to_string(analysis_tail_cap_ms));
        if (verbose_log) {
            std::cerr << "[Resamp] trim playback=[" << playback_start << "," << playback_end
                      << ") analysis=[" << src_start << "," << src_end
                      << ") origin_ms=" << params.source_origin_ms << '\n';
        }

        // ── 4. 출력 길이 ──────────────────────────────────────────────
        int output_samples = static_cast<int>(requested_output_samples);
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
                      << " Rs=" << sp.reverb_suppression
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
	                      << " VzS=" << sp.vocalizer_strength
	                      << " Fm=" << sp.fast_mode << '\n';
	        }
        append_debug_log("[FLAGS] Vtl=" + std::to_string(sp.tract_length) +
                         " Vtr=" + std::to_string(sp.tract_resonance) +
                         " Vtw=" + std::to_string(sp.tract_focus) +
                         " Vc=" + std::to_string(sp.tract_constriction) +
                         " Nn=" + std::to_string(sp.nasal_coupling) +
                         " Mo=" + std::to_string(sp.mouth_open) +
                         " Tn=" + std::to_string(sp.tension) +
                         " Rs=" + std::to_string(sp.reverb_suppression) +
                         " Vz=" + std::to_string(sp.vocalizer) +
                         " VzS=" + std::to_string(sp.vocalizer_strength));

        // ── 6. WORLD 분석 (raw Harvest F0 + envelope/AP 추출) ─────────
        // 디스크 캐시를 사용해 반복 렌더 시 분석 비용을 절감.
	        const int fast_mode_level = std::clamp(sp.fast_mode, 0, 2);
	        const bool formant_flags_requested =
	            std::abs(sp.mouth_open) > 0 ||
	            std::abs(sp.tract_resonance) > 0 ||
	            std::abs(sp.tract_focus) > 0 ||
	            (sp.vocalizer > 0 && sp.vocalizer_strength > 0);
	        const bool strong_formant_flags =
	            std::abs(sp.mouth_open) >= 45 ||
	            std::abs(sp.tract_resonance) >= 45 ||
	            std::abs(sp.tract_focus) >= 45 ||
	            (sp.vocalizer > 0 && sp.vocalizer_strength >= 35);
	        const bool critical_formant_flags =
	            std::abs(sp.mouth_open) >= 70 ||
	            std::abs(sp.tract_resonance) >= 70 ||
	            std::abs(sp.tract_focus) >= 70 ||
	            (sp.vocalizer > 0 && sp.vocalizer_strength >= 60);
	        const bool track_formants = formant_flags_requested &&
	            (fast_mode_level == 0 ||
	             (fast_mode_level == 1 && strong_formant_flags) ||
	             (fast_mode_level >= 2 && critical_formant_flags));
	        const double analysis_frame_period_ms =
	            (fast_mode_level == 0) ? 2.5 : ((fast_mode_level == 1) ? 3.25 : 4.5);
	        append_debug_log(std::string("[ANALYSIS] formants=") +
	                         (track_formants ? "tracked" : "default") +
	                         " frame_ms=" + std::to_string(analysis_frame_period_ms));
	        if (verbose_log) {
	            std::cerr << "[Resamp] analysis formants="
	                      << (track_formants ? "tracked" : "default")
	                      << " frame_ms=" << analysis_frame_period_ms << '\n';
	        }
	        auto wa = resamp::synth::world_analyze_cached(
	            trimmed, sample_rate, params.input_wav, src_start, src_end,
	            track_formants, analysis_frame_period_ms);

        std::string budget_reason;
        double max_render_mb = env_double_clamped("RESAMP_MAX_RENDER_MB", 768.0, 64.0, 8192.0);
        if (render_memory_budget_exceeded(output_samples, sample_rate, wa.n_frames, wa.fft_size,
                                          max_render_mb, budget_reason)) {
            return fail_with_silence(params, output_samples, static_cast<uint32_t>(sample_rate), budget_reason);
        }

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
        resamp::post::apply_boundary_level_guard(output, sample_rate);

        // Fade in/out:
        // 과도한 fade-in은 어두 자음 attack을 깎아 "툭 끊기는" 인상을 줄 수 있어 축소.
        resamp::post::apply_fades(output, 1.6, 4.0, sample_rate);

        // 역재생/디스토션/비트크러셔/최종 컷 필터.
        // Fc 하이컷/로우컷은 apply_flag_post_effects 내부에서 항상 마지막에 적용된다.
        double consonant_tgt_ms = params.consonant_ms *
            std::clamp(100.0 / static_cast<double>(std::max(1, params.velocity)), 0.25, 4.0);
        resamp::post::apply_flag_post_effects(output, sample_rate, consonant_tgt_ms, sp);

        // ── 10. WAV 저장 ──────────────────────────────────────────────
        resamp::io::save_wav(params.output_wav, output,
                             static_cast<uint32_t>(sample_rate));
        return 0;

    } catch (const std::bad_alloc&) {
        return fail_with_silence(params, requested_output_samples, static_cast<uint32_t>(std::max(1, sample_rate)),
                                 "Out of memory; wrote fallback silence");
    } catch (const std::length_error& e) {
        return fail_with_silence(params, requested_output_samples, static_cast<uint32_t>(std::max(1, sample_rate)),
                                 std::string("Allocation size error: ") + e.what() + "; wrote fallback silence");
    } catch (const std::exception& e) {
        std::cerr << "[Resamp] Error: " << e.what() << '\n';
        return 1;
    }
}
