#include "frq_reader.hpp"
#include <fstream>
#include <cstring>
#include <algorithm>
#include <filesystem>

namespace resamp::io {

namespace {

std::filesystem::path path_from_utf8(const std::string& path) {
#ifdef _WIN32
    return std::filesystem::u8path(path);
#else
    return std::filesystem::path(path);
#endif
}

} // namespace

static double read_f64_le(std::ifstream& f) {
    double v; f.read(reinterpret_cast<char*>(&v), 8); return v;
}
static int32_t read_i32_le(std::ifstream& f) {
    uint8_t b[4]; f.read(reinterpret_cast<char*>(b), 4);
    return static_cast<int32_t>(b[0] | (b[1]<<8) | (b[2]<<16) | (b[3]<<24));
}

FrqData load_frq(const std::string& wav_path) {
    // 경로 .wav → .frq
    std::string frq_path = wav_path;
    size_t dot = frq_path.rfind('.');
    if (dot != std::string::npos) frq_path = frq_path.substr(0, dot);
    frq_path += ".frq";

    std::ifstream f(path_from_utf8(frq_path), std::ios::binary);
    if (!f) return {}; // 파일 없음

    // 매직 헤더 "FREQ0003"
    char magic[8]; f.read(magic, 8);
    if (std::memcmp(magic, "FREQ0003", 8) != 0) return {};

    FrqData data;
    data.valid           = true;
    data.samples_per_frame = read_i32_le(f);
    data.average_f0      = read_f64_le(f);

    // 50바이트 패딩 건너뜀
    f.seekg(50, std::ios::cur);

    // F0/amplitude 쌍 읽기
    while (f) {
        double f0  = read_f64_le(f);
        double amp = read_f64_le(f);
        if (!f) break;
        data.frames.push_back({f0, amp});
    }

    return data;
}

double frq_get_f0(const FrqData& frq, int sample_pos) {
    if (!frq.valid || frq.frames.empty()) return 0.0;
    int frame_idx = sample_pos / frq.samples_per_frame;
    frame_idx = std::max(0, std::min(frame_idx, (int)frq.frames.size() - 1));
    return frq.frames[frame_idx].f0;
}

} // namespace resamp::io
