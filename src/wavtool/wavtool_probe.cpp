#include "io/wav_reader.hpp"
#include "io/wav_writer.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

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

std::string now_iso8601_local() {
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S");
    return ss.str();
}

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

fs::path resolve_log_path() {
    std::string env_path = read_env("RESAMP_WAVTOOL_LOG");
    if (!env_path.empty()) return fs::path(env_path);
    try {
        return fs::temp_directory_path() / "resamp_wavtool_calls.jsonl";
    } catch (...) {
        return fs::path("resamp_wavtool_calls.jsonl");
    }
}

void append_json_line(const fs::path& log_path, const std::string& json_line) {
    try {
        if (log_path.has_parent_path()) {
            fs::create_directories(log_path.parent_path());
        }
        std::ofstream ofs(log_path, std::ios::binary | std::ios::app);
        if (!ofs) return;
        ofs << json_line << "\n";
    } catch (...) {
        // M0 probe는 로깅 실패로 렌더를 실패시키지 않는다.
    }
}

void log_invoke(const fs::path& log_path, int argc, char** argv) {
    std::ostringstream ss;
    ss << "{\"time\":\"" << json_escape(now_iso8601_local()) << "\"";
    ss << ",\"event\":\"invoke\"";
    ss << ",\"cwd\":\"" << json_escape(fs::current_path().string()) << "\"";
    ss << ",\"argc\":" << argc;
    ss << ",\"args\":[";
    for (int i = 0; i < argc; ++i) {
        if (i > 0) ss << ",";
        ss << "\"" << json_escape(argv[i] ? argv[i] : "") << "\"";
    }
    ss << "]}";
    append_json_line(log_path, ss.str());
}

void log_result(const fs::path& log_path,
                const std::string& status,
                const std::string& note,
                const std::string& output_path,
                const std::string& input_path,
                uint32_t out_sr,
                uint32_t in_sr,
                size_t out_samples,
                size_t in_samples) {
    std::ostringstream ss;
    ss << "{\"time\":\"" << json_escape(now_iso8601_local()) << "\"";
    ss << ",\"event\":\"result\"";
    ss << ",\"status\":\"" << json_escape(status) << "\"";
    ss << ",\"note\":\"" << json_escape(note) << "\"";
    ss << ",\"output\":\"" << json_escape(output_path) << "\"";
    ss << ",\"input\":\"" << json_escape(input_path) << "\"";
    ss << ",\"out_sr\":" << out_sr;
    ss << ",\"in_sr\":" << in_sr;
    ss << ",\"out_samples\":" << out_samples;
    ss << ",\"in_samples\":" << in_samples;
    ss << "}";
    append_json_line(log_path, ss.str());
}

} // namespace

int main(int argc, char** argv) {
    fs::path log_path = resolve_log_path();
    log_invoke(log_path, argc, argv);

    if (argc < 3) {
        log_result(log_path, "ok", "argc_lt_3_noop", "", "", 0, 0, 0, 0);
        return 0;
    }

    const std::string output_path = argv[1] ? argv[1] : "";
    const std::string input_path = argv[2] ? argv[2] : "";

    try {
        resamp::io::WavInfo in_info{};
        std::vector<float> in_samples = resamp::io::load_wav(input_path, in_info);
        if (in_samples.empty()) in_samples.assign(64, 0.0f);
        const size_t in_count = in_samples.size();

        std::vector<float> out_samples;
        resamp::io::WavInfo out_info{};
        bool has_existing_out = false;
        size_t out_prev_count = 0;
        if (fs::exists(output_path)) {
            try {
                out_samples = resamp::io::load_wav(output_path, out_info);
                has_existing_out = true;
                out_prev_count = out_samples.size();
            } catch (...) {
                has_existing_out = false;
                out_samples.clear();
            }
        }

        std::string note = "replace";
        uint32_t write_sr = in_info.sample_rate;
        if (has_existing_out && out_info.sample_rate == in_info.sample_rate) {
            out_samples.insert(out_samples.end(), in_samples.begin(), in_samples.end());
            note = "append";
        } else {
            if (has_existing_out && out_info.sample_rate != in_info.sample_rate) {
                note = "replace_sr_mismatch";
            }
            out_samples = std::move(in_samples);
        }

        fs::path out_path_fs(output_path);
        if (out_path_fs.has_parent_path()) {
            fs::create_directories(out_path_fs.parent_path());
        }
        resamp::io::save_wav(output_path, out_samples, write_sr);

        log_result(log_path,
                   "ok",
                   note,
                   output_path,
                   input_path,
                   has_existing_out ? out_info.sample_rate : write_sr,
                   in_info.sample_rate,
                   out_prev_count,
                   in_count);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[wavtool_probe] Error: " << e.what() << "\n";
        log_result(log_path,
                   "error",
                   e.what(),
                   output_path,
                   input_path,
                   0,
                   0,
                   0,
                   0);
        return 1;
    }
}
