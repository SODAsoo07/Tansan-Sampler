#include "io/wav_reader.hpp"
#include "io/wav_writer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr double kPi = 3.14159265358979323846;

struct WavtoolArgs {
    std::string output_path;
    std::string input_path;
    double skip_ms = 0.0;
    double duration_ms = 0.0;
    std::vector<double> envelope;
};

struct FastWavtoolArgs {
    std::string output_path;
    std::string input_path;
    double skip_ms = 0.0;
    double duration_ms = 0.0;
    double overlap_ms = 0.0;
};

struct JoinOptions {
    bool envelope = false;
    bool level_match = false;
    bool consonant_guard = false;
    bool phase = false;
};

struct MixShape {
    bool consonant_like = false;
    bool phase_mismatch = false;
    double level_gain = 1.0;
};

struct JoinAnalysis {
    bool voiced = false;
    int period = 0;
    double confidence = 0.0;
};

struct JoinDecision {
    int overlap = 0;
    int skip_adjust = 0;
    bool voiced = false;
    double confidence = 0.0;
};

struct WavDataInfo {
    uint32_t sample_rate = 44100;
    uint16_t channels = 1;
    uint16_t bits_per_sample = 16;
    uint16_t audio_format = 1;
    uint32_t data_offset = 0;
    uint32_t data_size = 0;
};

std::string read_env(const char* name) {
#ifdef _WIN32
    char* buf = nullptr;
    size_t len = 0;
    if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr) return {};
    std::string v(buf);
    std::free(buf);
    return v;
#else
    const char* v = std::getenv(name);
    return v ? std::string(v) : std::string();
#endif
}

std::string lower_ascii(std::string s) {
    for (char& c : s) {
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    return s;
}

bool env_truthy(const char* name) {
    std::string v = lower_ascii(read_env(name));
    return v == "1" || v == "true" || v == "yes" || v == "on";
}

std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 16);
    for (unsigned char ch : s) {
        switch (ch) {
        case '\"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (ch < 0x20) {
                std::ostringstream ss;
                ss << "\\u" << std::hex << std::uppercase
                   << std::setw(4) << std::setfill('0')
                   << static_cast<int>(ch);
                out += ss.str();
            } else {
                out.push_back(static_cast<char>(ch));
            }
            break;
        }
    }
    return out;
}

double parse_double_prefix(const std::string& s, double def = 0.0) {
    if (s.empty()) return def;
    try {
        size_t pos = 0;
        double v = std::stod(s, &pos);
        (void)pos;
        return std::isfinite(v) ? v : def;
    } catch (...) {
        return def;
    }
}

double parse_duration_ms(const std::string& s) {
    if (s.empty()) return 0.0;

    const size_t at = s.find('@');
    if (at == std::string::npos) {
        return std::max(0.0, parse_double_prefix(s, 0.0));
    }

    const double ticks = parse_double_prefix(s.substr(0, at), 0.0);
    std::string tempo_part = s.substr(at + 1);
    double correction_ms = 0.0;
    size_t correction_pos = tempo_part.find_first_of("+-", 1);
    if (correction_pos != std::string::npos) {
        correction_ms = parse_double_prefix(tempo_part.substr(correction_pos), 0.0);
        tempo_part = tempo_part.substr(0, correction_pos);
    }
    const double tempo = std::max(1.0, parse_double_prefix(tempo_part, 120.0));
    return std::max(0.0, ticks * 60000.0 / (tempo * 480.0) + correction_ms);
}

int ms_to_samples(double ms, uint32_t sr) {
    return static_cast<int>(std::llround(ms * static_cast<double>(sr) / 1000.0));
}

void write_u16(std::ostream& f, uint16_t v) {
    uint8_t b[2] = {
        static_cast<uint8_t>(v & 0xFF),
        static_cast<uint8_t>((v >> 8) & 0xFF),
    };
    f.write(reinterpret_cast<const char*>(b), 2);
}

void write_u32(std::ostream& f, uint32_t v) {
    uint8_t b[4] = {
        static_cast<uint8_t>(v & 0xFF),
        static_cast<uint8_t>((v >> 8) & 0xFF),
        static_cast<uint8_t>((v >> 16) & 0xFF),
        static_cast<uint8_t>((v >> 24) & 0xFF),
    };
    f.write(reinterpret_cast<const char*>(b), 4);
}

uint32_t read_u32_at(std::fstream& f, std::streamoff pos) {
    f.seekg(pos, std::ios::beg);
    uint8_t b[4] = {};
    f.read(reinterpret_cast<char*>(b), 4);
    return static_cast<uint32_t>(b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24));
}

uint16_t read_u16_stream(std::istream& f) {
    uint8_t b[2] = {};
    f.read(reinterpret_cast<char*>(b), 2);
    return static_cast<uint16_t>(b[0] | (b[1] << 8));
}

uint32_t read_u32_stream(std::istream& f) {
    uint8_t b[4] = {};
    f.read(reinterpret_cast<char*>(b), 4);
    return static_cast<uint32_t>(b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24));
}

bool probe_wav_data_stream(std::istream& f, WavDataInfo& info) {
    info = WavDataInfo{};
    char riff[4] = {};
    char wave[4] = {};
    f.read(riff, 4);
    read_u32_stream(f);
    f.read(wave, 4);
    if (std::string(riff, 4) != "RIFF" || std::string(wave, 4) != "WAVE") return false;

    bool found_fmt = false;
    bool found_data = false;
    while (f && !(found_fmt && found_data)) {
        char chunk_id[4] = {};
        f.read(chunk_id, 4);
        if (!f) break;
        const uint32_t chunk_size = read_u32_stream(f);
        const std::streamoff payload = f.tellg();
        if (std::string(chunk_id, 4) == "fmt ") {
            info.audio_format = read_u16_stream(f);
            info.channels = read_u16_stream(f);
            info.sample_rate = read_u32_stream(f);
            read_u32_stream(f);
            read_u16_stream(f);
            info.bits_per_sample = read_u16_stream(f);
            found_fmt = true;
        } else if (std::string(chunk_id, 4) == "data") {
            info.data_offset = static_cast<uint32_t>(payload);
            info.data_size = chunk_size;
            found_data = true;
        }
        f.seekg(payload + static_cast<std::streamoff>(chunk_size), std::ios::beg);
    }
    return found_fmt && found_data;
}

bool probe_wav_data(const std::string& path, WavDataInfo& info) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    return probe_wav_data_stream(f, info);
}

void write_pcm16_header(std::ostream& f, uint32_t sample_rate, uint32_t data_size) {
    const uint16_t channels = 1;
    const uint16_t bits = 16;
    const uint16_t block_align = channels * bits / 8;
    const uint32_t byte_rate = sample_rate * block_align;
    f.write("RIFF", 4);
    write_u32(f, 36 + data_size);
    f.write("WAVE", 4);
    f.write("fmt ", 4);
    write_u32(f, 16);
    write_u16(f, 1);
    write_u16(f, channels);
    write_u32(f, sample_rate);
    write_u32(f, byte_rate);
    write_u16(f, block_align);
    write_u16(f, bits);
    f.write("data", 4);
    write_u32(f, data_size);
}

void write_pcm16_samples(std::ostream& f, const std::vector<float>& samples) {
    for (float s : samples) {
        const float clamped = std::max(-1.0f, std::min(1.0f, s));
        const int16_t pcm = static_cast<int16_t>(
            clamped >= 0.0f ? clamped * 32767.0f : clamped * 32768.0f);
        uint8_t b[2] = {
            static_cast<uint8_t>(pcm & 0xFF),
            static_cast<uint8_t>((pcm >> 8) & 0xFF),
        };
        f.write(reinterpret_cast<const char*>(b), 2);
    }
}

std::vector<float> trim_input(std::vector<float> in,
                              int skip_samples,
                              int duration_samples,
                              int overlap_samples,
                              uint32_t sr);

std::vector<int16_t> load_pcm16_segment(const WavtoolArgs& args,
                                        uint32_t& sample_rate,
                                        int keep_samples_extra) {
    WavDataInfo wi{};
    if (probe_wav_data(args.input_path, wi) &&
        wi.audio_format == 1 &&
        wi.bits_per_sample == 16 &&
        wi.channels >= 1) {
        sample_rate = wi.sample_rate;
        const int skip_samples = std::max(0, ms_to_samples(args.skip_ms, sample_rate));
        const int duration_samples = std::max(0, ms_to_samples(args.duration_ms, sample_rate));
        const int total_frames = static_cast<int>(wi.data_size / (2 * wi.channels));
        const int available = std::max(0, total_frames - skip_samples);
        int wanted = duration_samples > 0 ? duration_samples + keep_samples_extra : available;
        wanted = std::max(0, std::min(wanted, available));

        std::vector<int16_t> out(static_cast<size_t>(wanted), 0);
        std::ifstream f(args.input_path, std::ios::binary);
        if (!f) throw std::runtime_error("Cannot open WAV: " + args.input_path);
        const std::streamoff start = static_cast<std::streamoff>(wi.data_offset) +
            static_cast<std::streamoff>(skip_samples) * wi.channels * 2;
        f.seekg(start, std::ios::beg);
        for (int i = 0; i < wanted; ++i) {
            int32_t sum = 0;
            for (uint16_t ch = 0; ch < wi.channels; ++ch) {
                uint8_t b[2] = {};
                f.read(reinterpret_cast<char*>(b), 2);
                const int16_t s = static_cast<int16_t>(b[0] | (b[1] << 8));
                sum += s;
            }
            out[i] = static_cast<int16_t>(std::clamp(
                sum / static_cast<int32_t>(wi.channels),
                static_cast<int32_t>(-32768),
                static_cast<int32_t>(32767)));
        }
        return out;
    }

    resamp::io::WavInfo in_info{};
    std::vector<float> input = resamp::io::load_wav(args.input_path, in_info);
    sample_rate = in_info.sample_rate;
    const int skip_samples = std::max(0, ms_to_samples(args.skip_ms, sample_rate));
    const int duration_samples = std::max(0, ms_to_samples(args.duration_ms, sample_rate));
    std::vector<float> segment = trim_input(std::move(input), skip_samples, duration_samples, keep_samples_extra, sample_rate);
    if (duration_samples > 0) segment.resize(static_cast<size_t>(duration_samples + keep_samples_extra), 0.0f);
    std::vector<int16_t> out(segment.size());
    for (size_t i = 0; i < segment.size(); ++i) {
        const float clamped = std::max(-1.0f, std::min(1.0f, segment[i]));
        out[i] = static_cast<int16_t>(clamped >= 0.0f ? clamped * 32767.0f : clamped * 32768.0f);
    }
    return out;
}

std::vector<int16_t> load_pcm16_segment_fast(const FastWavtoolArgs& args,
                                             uint32_t& sample_rate) {
    WavDataInfo wi{};
    std::ifstream f(args.input_path, std::ios::binary);
    if (f &&
        probe_wav_data_stream(f, wi) &&
        wi.audio_format == 1 &&
        wi.bits_per_sample == 16 &&
        wi.channels >= 1) {
        sample_rate = wi.sample_rate;
        const int skip_samples = std::max(0, ms_to_samples(args.skip_ms, sample_rate));
        const int duration_samples = std::max(0, ms_to_samples(args.duration_ms, sample_rate));
        const int total_frames = static_cast<int>(wi.data_size / (2 * wi.channels));
        const int available = std::max(0, total_frames - skip_samples);
        const int wanted = duration_samples > 0
            ? std::min(duration_samples, available)
            : available;

        std::vector<int16_t> out(static_cast<size_t>(wanted), 0);
        const std::streamoff start = static_cast<std::streamoff>(wi.data_offset) +
            static_cast<std::streamoff>(skip_samples) * wi.channels * 2;
        f.seekg(start, std::ios::beg);
        if (wi.channels == 1) {
            f.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(wanted * 2));
        } else {
            for (int i = 0; i < wanted; ++i) {
                int32_t sum = 0;
                for (uint16_t ch = 0; ch < wi.channels; ++ch) {
                    uint8_t b[2] = {};
                    f.read(reinterpret_cast<char*>(b), 2);
                    sum += static_cast<int16_t>(b[0] | (b[1] << 8));
                }
                out[i] = static_cast<int16_t>(std::clamp(
                    sum / static_cast<int32_t>(wi.channels),
                    static_cast<int32_t>(-32768),
                    static_cast<int32_t>(32767)));
            }
        }
        return out;
    }

    WavtoolArgs slow_args;
    slow_args.output_path = args.output_path;
    slow_args.input_path = args.input_path;
    slow_args.skip_ms = args.skip_ms;
    slow_args.duration_ms = args.duration_ms;
    return load_pcm16_segment(slow_args, sample_rate, 0);
}

std::vector<double> parse_envelope_args(int argc, char** argv) {
    std::vector<double> envelope;
    if (argc <= 5) return envelope;
    envelope.reserve(static_cast<size_t>(argc - 5));
    for (int i = 5; i < argc; ++i) {
        envelope.push_back(parse_double_prefix(argv[i] ? argv[i] : "", 0.0));
    }
    return envelope;
}

int16_t scale_pcm16(int16_t sample, double gain) {
    const int v = static_cast<int>(std::llround(static_cast<double>(sample) * gain));
    return static_cast<int16_t>(std::clamp(v, -32768, 32767));
}

void apply_gain_range_pcm16(std::vector<int16_t>& samples,
                            int begin,
                            int end,
                            double start_gain,
                            double end_gain) {
    begin = std::max(0, std::min(begin, static_cast<int>(samples.size())));
    end = std::max(begin, std::min(end, static_cast<int>(samples.size())));
    const int count = end - begin;
    if (count <= 0) return;
    if (std::abs(start_gain - end_gain) < 1e-9) {
        if (std::abs(start_gain - 1.0) < 1e-9) return;
        for (int i = begin; i < end; ++i) {
            samples[static_cast<size_t>(i)] = scale_pcm16(samples[static_cast<size_t>(i)], start_gain);
        }
        return;
    }
    const double denom = static_cast<double>(std::max(1, count - 1));
    const double step = (end_gain - start_gain) / denom;
    double gain = start_gain;
    for (int i = begin; i < end; ++i) {
        samples[static_cast<size_t>(i)] = scale_pcm16(samples[static_cast<size_t>(i)], gain);
        gain += step;
    }
}

void apply_envelope_pcm16(std::vector<int16_t>& samples,
                          const std::vector<double>& env,
                          double duration_ms,
                          uint32_t sample_rate) {
    if (env.size() < 7 || duration_ms <= 0.0 || samples.empty()) return;

    const double x1 = std::max(0.0, env[1]);
    const double x3 = std::max(x1, duration_ms - std::max(0.0, env[2]));
    const double x4 = duration_ms;
    const double x2 = std::max(x1, std::min(x3, x1 + (env.size() > 9 ? std::max(0.0, env[9]) : 0.0)));

    const double y0 = std::clamp(env[3] / 100.0, 0.0, 2.0);
    const double y1 = std::clamp(env[4] / 100.0, 0.0, 2.0);
    const double y3 = std::clamp(env[5] / 100.0, 0.0, 2.0);
    const double y4 = std::clamp(env[6] / 100.0, 0.0, 2.0);
    const double y2 = std::clamp((env.size() > 10 ? env[10] : env[4]) / 100.0, 0.0, 2.0);

    if (std::abs(y0 - 1.0) < 1e-9 &&
        std::abs(y1 - 1.0) < 1e-9 &&
        std::abs(y2 - 1.0) < 1e-9 &&
        std::abs(y3 - 1.0) < 1e-9 &&
        std::abs(y4 - 1.0) < 1e-9) {
        return;
    }

    const int n = static_cast<int>(samples.size());
    const int s1 = std::clamp(ms_to_samples(x1, sample_rate), 0, n);
    const int s2 = std::clamp(ms_to_samples(x2, sample_rate), s1, n);
    const int s3 = std::clamp(ms_to_samples(x3, sample_rate), s2, n);

    apply_gain_range_pcm16(samples, 0, s1, y0, y1);
    apply_gain_range_pcm16(samples, s1, s2, y1, y2);
    apply_gain_range_pcm16(samples, s2, s3, y2, y3);
    apply_gain_range_pcm16(samples, s3, n, y3, y4);
}

double rms_pcm16(const int16_t* samples, int count) {
    if (!samples || count <= 0) return 0.0;
    double sum = 0.0;
    for (int i = 0; i < count; ++i) {
        const double v = static_cast<double>(samples[i]) / 32768.0;
        sum += v * v;
    }
    return std::sqrt(sum / static_cast<double>(count));
}

double zero_crossing_rate_pcm16(const int16_t* samples, int count) {
    if (!samples || count <= 1) return 0.0;
    int crossings = 0;
    for (int i = 1; i < count; ++i) {
        if ((samples[i - 1] < 0 && samples[i] >= 0) ||
            (samples[i - 1] >= 0 && samples[i] < 0)) {
            crossings++;
        }
    }
    return static_cast<double>(crossings) / static_cast<double>(count - 1);
}

double correlation_pcm16(const int16_t* a, const int16_t* b, int count) {
    if (!a || !b || count <= 16) return 0.0;
    double num = 0.0;
    double da = 0.0;
    double db = 0.0;
    for (int i = 0; i < count; ++i) {
        const double x = static_cast<double>(a[i]);
        const double y = static_cast<double>(b[i]);
        num += x * y;
        da += x * x;
        db += y * y;
    }
    return (da > 1e-9 && db > 1e-9) ? num / std::sqrt(da * db) : 0.0;
}

int estimate_period_pcm16(const int16_t* a,
                          const int16_t* b,
                          int count,
                          uint32_t sample_rate,
                          double& confidence) {
    confidence = 0.0;
    if (!a || !b || count < 128) return 0;
    const int min_lag = std::max(24, static_cast<int>(sample_rate / 900));
    const int max_lag = std::min(count / 2, static_cast<int>(sample_rate / 70));
    if (max_lag <= min_lag) return 0;
    int best_lag = 0;
    double best = -1.0;
    for (int lag = min_lag; lag <= max_lag; ++lag) {
        const int n = count - lag;
        double num = 0.0;
        double da = 0.0;
        double db = 0.0;
        for (int i = 0; i < n; ++i) {
            const double x0 = 0.5 * (static_cast<double>(a[i]) + static_cast<double>(b[i]));
            const double x1 = 0.5 * (static_cast<double>(a[i + lag]) + static_cast<double>(b[i + lag]));
            num += x0 * x1;
            da += x0 * x0;
            db += x1 * x1;
        }
        const double corr = (da > 1e-9 && db > 1e-9) ? num / std::sqrt(da * db) : 0.0;
        if (corr > best) {
            best = corr;
            best_lag = lag;
        }
    }
    confidence = best;
    return best_lag;
}

int find_best_phase_shift_pcm16(const int16_t* old_tail,
                                const std::vector<int16_t>& samples,
                                int overlap,
                                int period) {
    if (!old_tail || overlap <= 32 || period <= 0) return 0;
    const int max_shift = std::min({period / 2, 96, static_cast<int>(samples.size()) - overlap - 1});
    if (max_shift <= 0) return 0;
    auto corr_at = [&](int shift) {
        double num = 0.0;
        double da = 0.0;
        double db = 0.0;
        for (int i = 0; i < overlap; ++i) {
            const double a = old_tail[i];
            const double b = samples[shift + i];
            num += a * b;
            da += a * a;
            db += b * b;
        }
        return (da > 1e-9 && db > 1e-9) ? num / std::sqrt(da * db) : -1.0;
    };
    int best_shift = 0;
    double best = corr_at(0);
    for (int shift = 1; shift <= max_shift; ++shift) {
        const double c = corr_at(shift);
        if (c > best) {
            best = c;
            best_shift = shift;
        }
    }
    return best_shift;
}

void apply_phase_shift_in_overlap(std::vector<int16_t>& samples, int shift, int overlap) {
    if (shift <= 0 || overlap <= 0 || static_cast<int>(samples.size()) <= overlap + shift) return;
    const int n = std::min(overlap, static_cast<int>(samples.size()) - shift);
    for (int i = 0; i < n; ++i) {
        samples[i] = samples[shift + i];
    }
}

WavtoolArgs parse_args(int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error("Usage: wavtool <output.wav> <input.wav> [skip_ms] [duration_ms|ticks@tempo] [envelope...]");
    }

    WavtoolArgs args;
    args.output_path = argv[1] ? argv[1] : "";
    args.input_path = argv[2] ? argv[2] : "";
    if (argc > 3) args.skip_ms = parse_double_prefix(argv[3] ? argv[3] : "", 0.0);
    if (argc > 4) args.duration_ms = parse_duration_ms(argv[4] ? argv[4] : "");
    for (int i = 5; i < argc; ++i) {
        args.envelope.push_back(parse_double_prefix(argv[i] ? argv[i] : "", 0.0));
    }
    return args;
}

FastWavtoolArgs parse_fast_args(int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error("Usage: V_wavtool <output.wav> <input.wav> [skip_ms] [duration_ms|ticks@tempo] [envelope...]");
    }
    FastWavtoolArgs args;
    args.output_path = argv[1] ? argv[1] : "";
    args.input_path = argv[2] ? argv[2] : "";
    if (argc > 3) args.skip_ms = parse_double_prefix(argv[3] ? argv[3] : "", 0.0);
    if (argc > 4) args.duration_ms = parse_duration_ms(argv[4] ? argv[4] : "");
    if (argc > 12) args.overlap_ms = std::max(0.0, parse_double_prefix(argv[12] ? argv[12] : "", 0.0));
    return args;
}

float clamp_sample(double v) {
    if (v > 1.0) return 1.0f;
    if (v < -1.0) return -1.0f;
    return static_cast<float>(v);
}

double rms_range(const std::vector<float>& x, int start, int len) {
    if (x.empty() || len <= 0) return 0.0;
    start = std::max(0, std::min(start, static_cast<int>(x.size())));
    int end = std::max(start, std::min(start + len, static_cast<int>(x.size())));
    if (end <= start) return 0.0;
    double sum = 0.0;
    for (int i = start; i < end; ++i) sum += static_cast<double>(x[i]) * x[i];
    return std::sqrt(sum / static_cast<double>(end - start));
}

double mean_range(const std::vector<float>& x, int start, int len) {
    if (x.empty() || len <= 0) return 0.0;
    start = std::max(0, std::min(start, static_cast<int>(x.size())));
    int end = std::max(start, std::min(start + len, static_cast<int>(x.size())));
    if (end <= start) return 0.0;
    double sum = 0.0;
    for (int i = start; i < end; ++i) sum += x[i];
    return sum / static_cast<double>(end - start);
}

void remove_dc(std::vector<float>& x, uint32_t sr) {
    if (x.empty()) return;
    const int n = std::min<int>(static_cast<int>(x.size()), std::max(64, ms_to_samples(80.0, sr)));
    const double m = mean_range(x, 0, n);
    if (std::abs(m) < 0.000005) return;
    for (float& s : x) s = clamp_sample(static_cast<double>(s) - m);
}

void apply_short_ramps(std::vector<float>& x, uint32_t sr) {
    const int n = std::min<int>(ms_to_samples(0.7, sr), static_cast<int>(x.size()) / 2);
    if (n <= 1) return;
    for (int i = 0; i < n; ++i) {
        const double w = 0.5 - 0.5 * std::cos(kPi * static_cast<double>(i) / static_cast<double>(n));
        x[i] = static_cast<float>(x[i] * w);
        x[x.size() - 1 - i] = static_cast<float>(x[x.size() - 1 - i] * w);
    }
}

double envelope_gain_at(const std::vector<double>& env, double t_ms, double duration_ms) {
    if (env.size() < 7 || duration_ms <= 0.0) return 1.0;

    const double x1 = std::max(0.0, env[1]);
    const double x3 = std::max(x1, duration_ms - std::max(0.0, env[2]));
    const double x4 = duration_ms;
    const double x2 = std::max(x1, std::min(x3, x1 + (env.size() > 9 ? std::max(0.0, env[9]) : 0.0)));

    const double y0 = std::clamp(env[3] / 100.0, 0.0, 2.0);
    const double y1 = std::clamp(env[4] / 100.0, 0.0, 2.0);
    const double y3 = std::clamp(env[5] / 100.0, 0.0, 2.0);
    const double y4 = std::clamp(env[6] / 100.0, 0.0, 2.0);
    const double y2 = std::clamp((env.size() > 10 ? env[10] : env[4]) / 100.0, 0.0, 2.0);

    auto lerp = [](double a, double b, double x) {
        return a + (b - a) * std::clamp(x, 0.0, 1.0);
    };

    if (t_ms <= x1) return lerp(y0, y1, x1 > 0.0 ? t_ms / x1 : 1.0);
    if (t_ms <= x2) return lerp(y1, y2, x2 > x1 ? (t_ms - x1) / (x2 - x1) : 1.0);
    if (t_ms <= x3) return lerp(y2, y3, x3 > x2 ? (t_ms - x2) / (x3 - x2) : 1.0);
    return lerp(y3, y4, x4 > x3 ? (t_ms - x3) / (x4 - x3) : 1.0);
}

void apply_envelope(std::vector<float>& x, const std::vector<double>& env, double duration_ms, uint32_t sr) {
    if (env.size() < 7 || duration_ms <= 0.0) return;
    for (size_t i = 0; i < x.size(); ++i) {
        const double t_ms = static_cast<double>(i) * 1000.0 / static_cast<double>(sr);
        x[i] = clamp_sample(static_cast<double>(x[i]) * envelope_gain_at(env, t_ms, duration_ms));
    }
}

void suppress_bright_tail(std::vector<float>& x, uint32_t sr) {
    if (x.size() < 8) return;
    const int tail = std::min<int>(ms_to_samples(18.0, sr), static_cast<int>(x.size()));
    const int start = static_cast<int>(x.size()) - tail;
    float prev = x[start];
    for (int i = start + 1; i < static_cast<int>(x.size()); ++i) {
        const double fade = static_cast<double>(i - start) / std::max(1, tail - 1);
        const float low = static_cast<float>(0.72f * x[i] + 0.28f * prev);
        x[i] = static_cast<float>(x[i] * (1.0 - 0.25 * fade) + low * (0.25 * fade));
        prev = low;
    }
}

JoinAnalysis analyze_voicing(const std::vector<float>& out,
                             const std::vector<float>& in,
                             int overlap,
                             uint32_t sr) {
    JoinAnalysis a;
    if (overlap < ms_to_samples(8.0, sr) || out.empty() || in.empty()) return a;

    const int min_lag = std::max(24, static_cast<int>(sr / 900));
    const int max_lag = std::min(overlap / 2, static_cast<int>(sr / 70));
    if (max_lag <= min_lag) return a;

    const int out_start = std::max(0, static_cast<int>(out.size()) - overlap);
    const int in_start = 0;
    double best = -1.0;
    int best_lag = 0;

    for (int lag = min_lag; lag <= max_lag; ++lag) {
        const int n = std::min(overlap - lag, static_cast<int>(in.size()) - lag);
        if (n <= lag) continue;
        double num = 0.0;
        double d0 = 0.0;
        double d1 = 0.0;
        for (int i = 0; i < n; ++i) {
            const double x0 = 0.5 * out[out_start + i] + 0.5 * in[in_start + i];
            const double x1 = 0.5 * out[out_start + i + lag] + 0.5 * in[in_start + i + lag];
            num += x0 * x1;
            d0 += x0 * x0;
            d1 += x1 * x1;
        }
        const double corr = (d0 > 1e-12 && d1 > 1e-12) ? num / std::sqrt(d0 * d1) : 0.0;
        if (corr > best) {
            best = corr;
            best_lag = lag;
        }
    }

    a.confidence = best;
    a.period = best_lag;
    a.voiced = best > 0.42 && best_lag > 0;
    return a;
}

double corr_at_shift(const std::vector<float>& out,
                     const std::vector<float>& in,
                     int overlap,
                     int shift) {
    if (out.empty() || in.empty()) return -1.0;
    const int out_start = static_cast<int>(out.size()) - overlap;
    const int n = std::min(overlap, static_cast<int>(in.size()) - shift);
    if (out_start < 0 || n <= 16) return -1.0;
    double num = 0.0;
    double d0 = 0.0;
    double d1 = 0.0;
    for (int i = 0; i < n; ++i) {
        const double a = out[out_start + i];
        const double b = in[shift + i];
        num += a * b;
        d0 += a * a;
        d1 += b * b;
    }
    if (d0 <= 1e-12 || d1 <= 1e-12) return -1.0;
    return num / std::sqrt(d0 * d1);
}

int find_phase_adjustment(const std::vector<float>& out,
                          const std::vector<float>& in,
                          int overlap,
                          int period,
                          uint32_t sr) {
    if (period <= 0 || in.size() <= static_cast<size_t>(overlap + 8)) return 0;
    const int max_shift = std::min<int>(
        std::max(1, period / 2),
        std::min(ms_to_samples(4.0, sr), static_cast<int>(in.size()) - overlap - 1));
    int best_shift = 0;
    double best_corr = corr_at_shift(out, in, overlap, 0);
    for (int shift = 1; shift <= max_shift; ++shift) {
        const double c = corr_at_shift(out, in, overlap, shift);
        if (c > best_corr) {
            best_corr = c;
            best_shift = shift;
        }
    }
    return best_shift;
}

int find_zero_crossing_adjustment(const std::vector<float>& out,
                                  const std::vector<float>& in,
                                  int base_shift,
                                  uint32_t sr) {
    if (out.empty() || in.empty()) return base_shift;
    const double target = out.back();
    const int radius = std::min(ms_to_samples(1.0, sr), static_cast<int>(in.size()) - base_shift - 1);
    int best = base_shift;
    double best_score = std::numeric_limits<double>::max();
    const int begin = std::max(0, base_shift - radius);
    const int end = std::min<int>(static_cast<int>(in.size()) - 1, base_shift + radius);
    for (int i = begin; i <= end; ++i) {
        const bool zc = (i > 0 && ((in[i - 1] <= 0.0f && in[i] >= 0.0f) || (in[i - 1] >= 0.0f && in[i] <= 0.0f)));
        const double score = std::abs(static_cast<double>(in[i]) - target) + (zc ? 0.0 : 0.015);
        if (score < best_score) {
            best_score = score;
            best = i;
        }
    }
    return best;
}

int choose_overlap_samples(const std::vector<float>& out,
                           const std::vector<float>& in,
                           const std::vector<double>& env,
                           uint32_t sr,
                           bool smart_mode) {
    double env_overlap_ms = env.size() > 7 ? env[7] : 0.0;
    if (env_overlap_ms <= 0.0) env_overlap_ms = 12.0;

    int overlap = ms_to_samples(std::clamp(env_overlap_ms, 3.0, 42.0), sr);
    if (smart_mode && !out.empty() && !in.empty()) {
        const int probe = std::min(
            ms_to_samples(30.0, sr),
            std::min(static_cast<int>(out.size()), static_cast<int>(in.size())));
        const double tail_rms = rms_range(out, static_cast<int>(out.size()) - probe, probe);
        const double head_rms = rms_range(in, 0, probe);
        const double activity = std::max(tail_rms, head_rms);
        double target_ms = env_overlap_ms;
        if (activity > 0.08) target_ms += 4.0;
        if (activity > 0.16) target_ms += 3.0;
        overlap = std::max(overlap, ms_to_samples(std::clamp(target_ms, 4.0, 42.0), sr));
    }

    overlap = std::min<int>(overlap, static_cast<int>(out.size()));
    overlap = std::min<int>(overlap, static_cast<int>(in.size()) / 2);
    return std::max(0, overlap);
}

JoinDecision decide_join(const std::vector<float>& out,
                         const std::vector<float>& in,
                         const std::vector<double>& env,
                         uint32_t sr,
                         bool smart_mode) {
    JoinDecision d;
    d.overlap = choose_overlap_samples(out, in, env, sr, smart_mode);
    if (!smart_mode || d.overlap <= 0 || out.empty() || in.empty()) return d;

    JoinAnalysis join = analyze_voicing(out, in, d.overlap, sr);
    d.voiced = join.voiced;
    d.confidence = join.confidence;
    if (join.voiced && env_truthy("WT_PHASE")) {
        d.skip_adjust = find_phase_adjustment(out, in, d.overlap, join.period, sr);
    } else {
        d.skip_adjust = 0;
    }
    return d;
}

std::vector<float> trim_input(std::vector<float> in,
                              int skip_samples,
                              int duration_samples,
                              int overlap_samples,
                              uint32_t sr) {
    skip_samples = std::max(0, std::min(skip_samples, static_cast<int>(in.size())));
    std::vector<float> seg(in.begin() + skip_samples, in.end());
    if (duration_samples > 0) {
        const int keep = duration_samples + overlap_samples + ms_to_samples(24.0, sr);
        if (keep > 32 && static_cast<int>(seg.size()) > keep) {
            seg.resize(keep);
        }
    }
    if (seg.empty()) seg.assign(64, 0.0f);
    return seg;
}

void mix_with_crossfade(std::vector<float>& out,
                        const std::vector<float>& in,
                        int overlap,
                        uint32_t sr,
                        int target_size_after,
                        bool use_crossfade) {
    if (out.empty() || overlap <= 0 || !use_crossfade) {
        const size_t old = out.size();
        out.resize(old + in.size(), 0.0f);
        for (size_t i = 0; i < in.size(); ++i) out[old + i] = clamp_sample(out[old + i] + in[i]);
        if (target_size_after > 0) out.resize(std::max<int>(target_size_after, 1), 0.0f);
        return;
    }

    const int insert = static_cast<int>(out.size());
    const int needed = insert + static_cast<int>(in.size());
    out.resize(needed, 0.0f);

    const int fade = std::max(1, std::min(overlap, ms_to_samples(2.0, sr)));
    const int old_size = insert;
    if (old_size > 0) {
        const int begin = std::max(0, old_size - fade);
        for (int i = begin; i < old_size; ++i) {
            const double t = static_cast<double>(i - begin) /
                             static_cast<double>(std::max(1, old_size - begin - 1));
            const double duck = 1.0 - 0.18 * (0.5 - 0.5 * std::cos(kPi * t));
            out[i] = clamp_sample(static_cast<double>(out[i]) * duck);
        }
    }

    for (int i = 0; i < static_cast<int>(in.size()); ++i) {
        const int dst = insert + i;
        if (i < fade) {
            const double in_t = static_cast<double>(i) /
                                static_cast<double>(std::max(1, fade - 1));
            const double w_in = 0.5 - 0.5 * std::cos(kPi * in_t);
            out[dst] = clamp_sample(static_cast<double>(in[i]) * w_in);
        } else {
            out[dst] = in[i];
        }
    }

    if (target_size_after > 0) {
        out.resize(std::max<int>(target_size_after, 1), 0.0f);
    }
}

fs::path debug_log_path() {
    const std::string env_path = read_env("WT_DEBUG_LOG");
    if (!env_path.empty()) return fs::path(env_path);
    try {
        return fs::temp_directory_path() / "V_wavtool_calls.jsonl";
    } catch (...) {
        return fs::path("V_wavtool_calls.jsonl");
    }
}

void debug_log(const std::string& line) {
    if (!env_truthy("WT_LOG") && !env_truthy("WT_DEBUG")) return;
    try {
        fs::path path = debug_log_path();
        if (path.has_parent_path()) fs::create_directories(path.parent_path());
        std::ofstream ofs(path, std::ios::binary | std::ios::app);
        if (ofs) ofs << line << "\n";
    } catch (...) {
    }
}

bool logging_enabled() {
    return env_truthy("WT_LOG") || env_truthy("WT_DEBUG");
}

void log_render_fast(int argc,
                     char** argv,
                     const std::string& mode,
                     const FastWavtoolArgs& args,
                     size_t input_samples,
                     size_t written_samples,
                     uint32_t sample_rate,
                     double overlap_ms,
                     int advance_samples,
                     int final_samples) {
    std::ostringstream ss;
    ss << "{\"event\":\"render\",\"mode\":\"" << json_escape(mode)
       << "\",\"argc\":" << argc
       << ",\"args\":[";
    for (int i = 0; i < argc; ++i) {
        if (i > 0) ss << ",";
        ss << "\"" << json_escape(argv[i] ? argv[i] : "") << "\"";
    }
    ss << "]"
       << ",\"input_samples\":" << input_samples
       << ",\"written_samples\":" << written_samples
       << ",\"sample_rate\":" << sample_rate
       << ",\"skip_ms\":" << args.skip_ms
       << ",\"duration_ms\":" << args.duration_ms
       << ",\"overlap_ms\":" << overlap_ms
       << ",\"advance_samples\":" << advance_samples
       << ",\"final_samples\":" << final_samples
       << "}";
    debug_log(ss.str());
}

void append_wav_data(const std::string& output_path,
                     const std::vector<float>& samples,
                     uint32_t sample_rate) {
    fs::path out_path(output_path);
    if (out_path.has_parent_path()) fs::create_directories(out_path.parent_path());

    bool create_new = true;
    uint32_t old_data_size = 0;
    if (fs::exists(out_path) && fs::file_size(out_path) >= 44) {
        std::fstream f(output_path, std::ios::binary | std::ios::in | std::ios::out);
        if (f) {
            char riff[4] = {};
            char wave[4] = {};
            char data[4] = {};
            f.seekg(0, std::ios::beg);
            f.read(riff, 4);
            f.seekg(8, std::ios::beg);
            f.read(wave, 4);
            f.seekg(36, std::ios::beg);
            f.read(data, 4);
            if (std::string(riff, 4) == "RIFF" &&
                std::string(wave, 4) == "WAVE" &&
                std::string(data, 4) == "data") {
                old_data_size = read_u32_at(f, 40);
                create_new = false;
            }
        }
    }

    const uint32_t add_size = static_cast<uint32_t>(samples.size() * 2);
    if (create_new) {
        std::ofstream f(output_path, std::ios::binary | std::ios::trunc);
        if (!f) throw std::runtime_error("Cannot write WAV: " + output_path);
        write_pcm16_header(f, sample_rate, add_size);
        write_pcm16_samples(f, samples);
        return;
    }

    std::fstream f(output_path, std::ios::binary | std::ios::in | std::ios::out);
    if (!f) throw std::runtime_error("Cannot append WAV: " + output_path);
    const uint32_t new_data_size = old_data_size + add_size;
    f.seekp(4, std::ios::beg);
    write_u32(f, 36 + new_data_size);
    f.seekp(40, std::ios::beg);
    write_u32(f, new_data_size);
    f.seekp(0, std::ios::end);
    write_pcm16_samples(f, samples);
}

void write_whd_dat_positioned(const std::string& output_path,
                              const std::vector<int16_t>& samples,
                              uint32_t sample_rate,
                              int overlap_samples,
                              int duration_samples,
                              const JoinOptions& options = {}) {
    fs::path out_path(output_path);
    if (out_path.has_parent_path()) fs::create_directories(out_path.parent_path());
    const fs::path whd_path = fs::path(output_path + ".whd");
    const fs::path dat_path = fs::path(output_path + ".dat");

    uint32_t old_data_size = 0;
    if (fs::exists(dat_path)) {
        old_data_size = static_cast<uint32_t>(fs::file_size(dat_path));
    }
    const int old_samples = static_cast<int>(old_data_size / 2);
    const int effective_overlap = std::min(old_samples, std::max(0, overlap_samples));
    const int start_sample = old_samples - effective_overlap;
    const int advance_samples = std::max(0, duration_samples - effective_overlap);
    const int final_samples = old_samples + advance_samples;
    const uint32_t final_data_size = static_cast<uint32_t>(final_samples * 2);
    const int writable_samples = std::max(0, final_samples - start_sample);
    const int copy_count = std::min(static_cast<int>(samples.size()), writable_samples);

    {
        std::ofstream whd(whd_path, std::ios::binary | std::ios::trunc);
        if (!whd) throw std::runtime_error("Cannot write WAV header: " + whd_path.string());
        write_pcm16_header(whd, sample_rate, final_data_size);
    }

    std::fstream dat(dat_path, std::ios::binary | std::ios::in | std::ios::out);
    if (!dat) {
        std::ofstream create(dat_path, std::ios::binary | std::ios::trunc);
        if (!create) throw std::runtime_error("Cannot create WAV data: " + dat_path.string());
        create.close();
        dat.open(dat_path, std::ios::binary | std::ios::in | std::ios::out);
    }
    if (!dat) throw std::runtime_error("Cannot open WAV data: " + dat_path.string());

    if (old_samples < start_sample) {
        dat.seekp(0, std::ios::end);
        std::vector<char> zeros(static_cast<size_t>((start_sample - old_samples) * 2), 0);
        dat.write(zeros.data(), static_cast<std::streamsize>(zeros.size()));
    }

    const int mix_count = std::max(0, std::min(old_samples - start_sample, copy_count));
    if (mix_count > 0) {
        dat.seekg(static_cast<std::streamoff>(start_sample) * 2, std::ios::beg);
        std::vector<int16_t> old(static_cast<size_t>(mix_count));
        dat.read(reinterpret_cast<char*>(old.data()), static_cast<std::streamsize>(mix_count * 2));

        MixShape mix{};
        const int probe = std::min(mix_count, 256);
        mix.consonant_like = options.consonant_guard &&
            zero_crossing_rate_pcm16(samples.data(), probe) > 0.18;
        if (options.level_match && probe > 16) {
            const double old_rms = rms_pcm16(old.data(), probe);
            const double new_rms = rms_pcm16(samples.data(), probe);
            if (old_rms > 0.0005 && new_rms > 0.0005) {
                mix.level_gain = std::clamp(old_rms / new_rms, 0.88, mix.consonant_like ? 1.06 : 1.16);
            }
        }
        if (options.phase && probe > 64) {
            const double old_zcr = zero_crossing_rate_pcm16(old.data(), probe);
            const double new_zcr = zero_crossing_rate_pcm16(samples.data(), probe);
            const double corr = correlation_pcm16(old.data(), samples.data(), probe);
            mix.phase_mismatch = old_zcr < 0.15 && new_zcr < 0.15 && corr < 0.10;
        }

        dat.seekp(static_cast<std::streamoff>(start_sample) * 2, std::ios::beg);
        const int fade = std::max(1, mix_count);
        for (int i = 0; i < mix_count; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(std::max(1, fade - 1));
            double w_in = t * t * (3.0 - 2.0 * t);
            if (mix.phase_mismatch) {
                const double steep = std::clamp((t - 0.5) * 1.65 + 0.5, 0.0, 1.0);
                w_in = steep * steep * (3.0 - 2.0 * steep);
            }
            if (mix.consonant_like) {
                w_in = std::min(1.0, w_in * 1.04 + 0.02 * t);
            }
            const double w_old = 1.0 - w_in;
            const double adjusted = static_cast<double>(samples[i]) * mix.level_gain;
            const int mixed = static_cast<int>(std::llround(old[i] * w_old + adjusted * w_in));
            const int16_t s = static_cast<int16_t>(std::clamp(mixed, -32768, 32767));
            dat.write(reinterpret_cast<const char*>(&s), 2);
        }
    }

    if (copy_count > mix_count) {
        dat.seekp(static_cast<std::streamoff>(start_sample + mix_count) * 2, std::ios::beg);
        dat.write(reinterpret_cast<const char*>(samples.data() + mix_count),
                  static_cast<std::streamsize>((copy_count - mix_count) * 2));
    }

    const uint32_t written_data_size = static_cast<uint32_t>((start_sample + copy_count) * 2);
    if (final_data_size > written_data_size) {
        dat.seekp(static_cast<std::streamoff>(final_data_size) - 1, std::ios::beg);
        const char zero = 0;
        dat.write(&zero, 1);
    }
    dat.close();
    if (old_data_size > final_data_size || written_data_size > final_data_size) {
        std::error_code resize_error;
        fs::resize_file(dat_path, final_data_size, resize_error);
    }
}

int write_output_wav_positioned(const std::string& output_path,
                                 const std::vector<int16_t>& samples,
                                 uint32_t sample_rate,
                                 int overlap_samples,
                                 int duration_samples) {
    fs::path out_path(output_path);
    if (out_path.has_parent_path()) fs::create_directories(out_path.parent_path());

    bool create_new = !fs::exists(out_path) || fs::file_size(out_path) < 44;
    if (create_new) {
        std::ofstream create(output_path, std::ios::binary | std::ios::trunc);
        if (!create) throw std::runtime_error("Cannot create WAV: " + output_path);
        write_pcm16_header(create, sample_rate, 0);
    }

    std::fstream wav(output_path, std::ios::binary | std::ios::in | std::ios::out);
    if (!wav) throw std::runtime_error("Cannot open WAV: " + output_path);

    char riff[4] = {};
    char wave[4] = {};
    char data[4] = {};
    wav.seekg(0, std::ios::beg);
    wav.read(riff, 4);
    wav.seekg(8, std::ios::beg);
    wav.read(wave, 4);
    wav.seekg(36, std::ios::beg);
    wav.read(data, 4);
    if (std::string(riff, 4) != "RIFF" ||
        std::string(wave, 4) != "WAVE" ||
        std::string(data, 4) != "data") {
        wav.close();
        std::ofstream recreate(output_path, std::ios::binary | std::ios::trunc);
        if (!recreate) throw std::runtime_error("Cannot recreate WAV: " + output_path);
        write_pcm16_header(recreate, sample_rate, 0);
        recreate.close();
        wav.open(output_path, std::ios::binary | std::ios::in | std::ios::out);
        if (!wav) throw std::runtime_error("Cannot reopen WAV: " + output_path);
    }

    const uint32_t old_data_size = read_u32_at(wav, 40);
    const int old_samples = static_cast<int>(old_data_size / 2);
    const int effective_overlap = std::min(old_samples, std::max(0, overlap_samples));
    const int start_sample = old_samples - effective_overlap;
    const int advance_samples = std::max(0, duration_samples - effective_overlap);
    const int final_samples = old_samples + advance_samples;
    const uint32_t final_data_size = static_cast<uint32_t>(final_samples * 2);

    const int mix_count = std::max(0, std::min(old_samples - start_sample, static_cast<int>(samples.size())));
    if (mix_count > 0) {
        wav.seekg(44 + static_cast<std::streamoff>(start_sample) * 2, std::ios::beg);
        std::vector<int16_t> old(static_cast<size_t>(mix_count));
        wav.read(reinterpret_cast<char*>(old.data()), static_cast<std::streamsize>(mix_count * 2));
        wav.seekp(44 + static_cast<std::streamoff>(start_sample) * 2, std::ios::beg);
        for (int i = 0; i < mix_count; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(std::max(1, mix_count - 1));
            const double w_in = 0.5 - 0.5 * std::cos(kPi * t);
            const double w_old = 1.0 - w_in;
            const int mixed = static_cast<int>(std::llround(old[i] * w_old + samples[i] * w_in));
            const int16_t s = static_cast<int16_t>(std::clamp(mixed, -32768, 32767));
            wav.write(reinterpret_cast<const char*>(&s), 2);
        }
    }

    const int writable_samples = std::max(0, final_samples - start_sample);
    const int copy_count = std::min(static_cast<int>(samples.size()), writable_samples);
    if (copy_count > mix_count) {
        wav.seekp(44 + static_cast<std::streamoff>(start_sample + mix_count) * 2, std::ios::beg);
        wav.write(reinterpret_cast<const char*>(samples.data() + mix_count),
                  static_cast<std::streamsize>((copy_count - mix_count) * 2));
    }

    wav.seekp(4, std::ios::beg);
    write_u32(wav, 36 + final_data_size);
    wav.seekp(40, std::ios::beg);
    write_u32(wav, final_data_size);
    wav.seekp(44 + static_cast<std::streamoff>(final_data_size) - 1, std::ios::beg);
    const char zero = 0;
    wav.write(&zero, 1);
    return final_samples;
}

int render_fast_append(int argc, char** argv, const std::string& mode) {
    FastWavtoolArgs args = parse_fast_args(argc, argv);
    const bool natural_mode = mode == "natural";
    double overlap_ms = args.overlap_ms;
    uint32_t sr = 44100;
    std::vector<int16_t> segment = load_pcm16_segment_fast(args, sr);
    const int duration_samples_sr = std::max(0, ms_to_samples(args.duration_ms, sr));
    const int overlap_samples = std::max(0, ms_to_samples(overlap_ms, sr));
    JoinOptions join_options{};
    join_options.envelope = natural_mode;
    join_options.level_match = natural_mode;
    join_options.consonant_guard = env_truthy("WT_CV");
    join_options.phase = env_truthy("WT_PHASE");

    if (join_options.envelope) {
        apply_envelope_pcm16(segment, parse_envelope_args(argc, argv), args.duration_ms, sr);
    }

    write_whd_dat_positioned(args.output_path, segment, sr, overlap_samples,
                             duration_samples_sr, join_options);
    const int advance_samples = std::max(0, duration_samples_sr - overlap_samples);
    const fs::path dat_path = fs::path(args.output_path + ".dat");
    const int final_samples = fs::exists(dat_path)
        ? static_cast<int>(fs::file_size(dat_path) / 2)
        : 0;
    if (logging_enabled()) {
        log_render_fast(argc, argv, mode, args, segment.size(), segment.size(), sr,
                        overlap_ms, advance_samples, final_samples);
    }
    return 0;
}

std::vector<float> render_basic_append(const WavtoolArgs& args, uint32_t& write_sr) {
    resamp::io::WavInfo in_info{};
    std::vector<float> input = resamp::io::load_wav(args.input_path, in_info);
    write_sr = in_info.sample_rate;
    const int skip_samples = ms_to_samples(std::max(0.0, args.skip_ms), write_sr);
    input = trim_input(std::move(input), skip_samples, 0, 0, write_sr);

    std::vector<float> out;
    resamp::io::WavInfo out_info{};
    if (fs::exists(args.output_path)) {
        out = resamp::io::load_wav(args.output_path, out_info);
        if (out_info.sample_rate != write_sr) out.clear();
    }
    out.insert(out.end(), input.begin(), input.end());
    return out;
}

int render_wavtool(int argc, char** argv) {
    const std::string mode = lower_ascii(read_env("WT_MODE"));
    if (mode.empty() || mode == "natural" || mode == "fast" ||
        mode == "append" || mode == "off" || mode == "legacy") {
        return render_fast_append(argc, argv, mode.empty() ? "natural" : mode);
    }

    WavtoolArgs args = parse_args(argc, argv);

    const bool append_mode = false;
    const bool xfade_only = mode == "xfade" || mode == "basic";
    const bool smart_mode = mode == "smart" && !xfade_only;

    resamp::io::WavInfo in_info{};
    std::vector<float> input = resamp::io::load_wav(args.input_path, in_info);
    const uint32_t sr = in_info.sample_rate;

    std::vector<float> out;
    resamp::io::WavInfo out_info{};
    if (fs::exists(args.output_path)) {
        try {
            out = resamp::io::load_wav(args.output_path, out_info);
            if (out_info.sample_rate != sr) out.clear();
        } catch (...) {
            out.clear();
        }
    }

    const int previous_size = static_cast<int>(out.size());
    int duration_samples = ms_to_samples(args.duration_ms, sr);
    if (duration_samples <= 0 && out.empty()) duration_samples = static_cast<int>(input.size());

    remove_dc(input, sr);

    int skip_samples = ms_to_samples(std::max(0.0, args.skip_ms), sr);
    JoinDecision join = decide_join(out, input, args.envelope, sr, smart_mode);
    skip_samples += join.skip_adjust;

    std::vector<float> segment = trim_input(std::move(input), skip_samples, duration_samples, join.overlap, sr);
    apply_envelope(segment, args.envelope, args.duration_ms, sr);
    suppress_bright_tail(segment, sr);
    apply_short_ramps(segment, sr);

    const int overlap = choose_overlap_samples(out, segment, args.envelope, sr, smart_mode);
    const int target_size = duration_samples > 0 ? previous_size + duration_samples : 0;
    mix_with_crossfade(out, segment, overlap, sr, target_size, !append_mode);

    fs::path out_path(args.output_path);
    if (out_path.has_parent_path()) fs::create_directories(out_path.parent_path());
    resamp::io::save_wav(args.output_path, out, sr);

    if (logging_enabled()) {
        std::ostringstream ss;
        ss << "{\"event\":\"render\",\"mode\":\"" << (mode.empty() ? "smart" : mode)
           << "\",\"argc\":" << argc
           << ",\"args\":[";
        for (int i = 0; i < argc; ++i) {
            if (i > 0) ss << ",";
            ss << "\"" << json_escape(argv[i] ? argv[i] : "") << "\"";
        }
        ss << "]"
           << ",\"input_samples\":" << segment.size()
           << ",\"previous_samples\":" << previous_size
           << ",\"output_samples\":" << out.size()
           << ",\"overlap\":" << overlap
           << ",\"skip_adjust\":" << join.skip_adjust
           << ",\"voiced\":" << (join.voiced ? "true" : "false")
           << ",\"confidence\":" << join.confidence
           << ",\"skip_ms\":" << args.skip_ms
           << ",\"duration_ms\":" << args.duration_ms
           << "}";
        debug_log(ss.str());
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    try {
        return render_wavtool(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "[wavtool] Error: " << e.what() << "\n";
        if (env_truthy("WT_STRICT")) {
            return 1;
        }
        try {
            WavtoolArgs args = parse_args(argc, argv);
            uint32_t sr = 44100;
            auto out = render_basic_append(args, sr);
            fs::path out_path(args.output_path);
            if (out_path.has_parent_path()) fs::create_directories(out_path.parent_path());
            resamp::io::save_wav(args.output_path, out, sr);
            debug_log("{\"event\":\"fallback\",\"mode\":\"append\"}");
            return 0;
        } catch (const std::exception& fallback_error) {
            std::cerr << "[wavtool] Fallback failed: " << fallback_error.what() << "\n";
            return 1;
        }
    }
}
