#include "wav_writer.hpp"
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>
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

static void write_u16(std::ofstream& f, uint16_t v) {
    uint8_t b[2] = { static_cast<uint8_t>(v & 0xFF),
                     static_cast<uint8_t>((v >> 8) & 0xFF) };
    f.write(reinterpret_cast<char*>(b), 2);
}
static void write_u32(std::ofstream& f, uint32_t v) {
    uint8_t b[4] = { static_cast<uint8_t>(v & 0xFF),
                     static_cast<uint8_t>((v >> 8)  & 0xFF),
                     static_cast<uint8_t>((v >> 16) & 0xFF),
                     static_cast<uint8_t>((v >> 24) & 0xFF) };
    f.write(reinterpret_cast<char*>(b), 4);
}

void save_wav(const std::string& path,
              const std::vector<float>& samples,
              uint32_t sample_rate) {
    std::ofstream f(path_from_utf8(path), std::ios::binary);
    if (!f) throw std::runtime_error("Cannot write WAV: " + path);

    uint32_t num_samples     = static_cast<uint32_t>(samples.size());
    uint16_t num_channels    = 1;
    uint16_t bits_per_sample = 16;
    uint32_t byte_rate       = sample_rate * num_channels * bits_per_sample / 8;
    uint16_t block_align     = static_cast<uint16_t>(num_channels * bits_per_sample / 8);
    uint32_t data_size       = num_samples * block_align;

    // RIFF 헤더
    f.write("RIFF", 4);
    write_u32(f, 36 + data_size); // file size - 8
    f.write("WAVE", 4);

    // fmt 청크
    f.write("fmt ", 4);
    write_u32(f, 16);              // chunk size
    write_u16(f, 1);               // PCM
    write_u16(f, num_channels);
    write_u32(f, sample_rate);
    write_u32(f, byte_rate);
    write_u16(f, block_align);
    write_u16(f, bits_per_sample);

    // data 청크
    f.write("data", 4);
    write_u32(f, data_size);

    for (float s : samples) {
        // 클리핑 후 int16 변환
        float clamped = std::max(-1.0f, std::min(1.0f, s));
        int16_t pcm = static_cast<int16_t>(
            clamped >= 0.0f
                ? clamped * 32767.0f
                : clamped * 32768.0f
        );
        uint8_t b[2] = { static_cast<uint8_t>(pcm & 0xFF),
                         static_cast<uint8_t>((pcm >> 8) & 0xFF) };
        f.write(reinterpret_cast<char*>(b), 2);
    }
}

} // namespace resamp::io
