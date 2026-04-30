#include "wav_reader.hpp"
#include <fstream>
#include <stdexcept>
#include <cstring>
#include <cmath>

namespace resamp::io {

// ── 리틀엔디언 읽기 헬퍼 ─────────────────────────────────────────────────
static uint16_t read_u16(std::ifstream& f) {
    uint8_t b[2]; f.read(reinterpret_cast<char*>(b), 2);
    return static_cast<uint16_t>(b[0] | (b[1] << 8));
}
static uint32_t read_u32(std::ifstream& f) {
    uint8_t b[4]; f.read(reinterpret_cast<char*>(b), 4);
    return static_cast<uint32_t>(b[0] | (b[1]<<8) | (b[2]<<16) | (b[3]<<24));
}
static int32_t read_i32(std::ifstream& f) {
    return static_cast<int32_t>(read_u32(f));
}

std::vector<float> load_wav(const std::string& path, WavInfo& info) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open WAV: " + path);

    // RIFF 헤더
    char riff[4]; f.read(riff, 4);
    if (std::memcmp(riff, "RIFF", 4) != 0)
        throw std::runtime_error("Not a RIFF file: " + path);
    read_u32(f); // file size - 8 (무시)
    char wave[4]; f.read(wave, 4);
    if (std::memcmp(wave, "WAVE", 4) != 0)
        throw std::runtime_error("Not a WAVE file: " + path);

    uint16_t audio_format   = 0;
    uint16_t num_channels   = 1;
    uint32_t sample_rate    = 44100;
    uint16_t bits_per_sample = 16;
    uint32_t data_size      = 0;
    bool     found_data     = false;

    // 청크 순회
    while (f && !found_data) {
        char chunk_id[4]; f.read(chunk_id, 4);
        if (!f) break;
        uint32_t chunk_size = read_u32(f);

        if (std::memcmp(chunk_id, "fmt ", 4) == 0) {
            audio_format    = read_u16(f);
            num_channels    = read_u16(f);
            sample_rate     = read_u32(f);
            read_u32(f); // byte_rate
            read_u16(f); // block_align
            bits_per_sample = read_u16(f);
            // WAVE_FORMAT_EXTENSIBLE: read sub-format GUID to get the real codec tag
            if (audio_format == 65534 && chunk_size >= 40) {
                read_u16(f);                 // cbSize
                read_u16(f);                 // wValidBitsPerSample
                read_u32(f);                 // dwChannelMask
                audio_format = read_u16(f);  // SubFormat GUID bytes 0-1 = real format tag
                f.seekg(chunk_size - 26, std::ios::cur); // skip remaining GUID bytes + any padding
            } else if (chunk_size > 16) {
                f.seekg(chunk_size - 16, std::ios::cur);
            }
        } else if (std::memcmp(chunk_id, "data", 4) == 0) {
            data_size  = chunk_size;
            found_data = true;
        } else {
            f.seekg(chunk_size, std::ios::cur);
        }
    }

    if (!found_data)
        throw std::runtime_error("No data chunk in WAV: " + path);

    // audio_format: 1=PCM, 3=IEEE float
    if (audio_format != 1 && audio_format != 3)
        throw std::runtime_error("Unsupported WAV format: " + std::to_string(audio_format));

    uint32_t bytes_per_sample = bits_per_sample / 8;
    uint32_t total_samples    = data_size / (bytes_per_sample * num_channels);

    info.sample_rate      = sample_rate;
    info.num_channels     = num_channels;
    info.bits_per_sample  = bits_per_sample;
    info.num_samples      = total_samples;

    std::vector<float> mono(total_samples);

    for (uint32_t i = 0; i < total_samples; ++i) {
        double sum = 0.0;
        for (uint16_t ch = 0; ch < num_channels; ++ch) {
            float s = 0.0f;
            if (audio_format == 3 && bits_per_sample == 32) {
                // IEEE 32-bit float
                float v; f.read(reinterpret_cast<char*>(&v), 4); s = v;
            } else if (bits_per_sample == 16) {
                int16_t v; f.read(reinterpret_cast<char*>(&v), 2);
                s = v / 32768.0f;
            } else if (bits_per_sample == 24) {
                uint8_t b[3]; f.read(reinterpret_cast<char*>(b), 3);
                int32_t v = (b[0]) | (b[1]<<8) | (b[2]<<16);
                if (v & 0x800000) v |= 0xFF000000; // 부호 확장
                s = v / 8388608.0f;
            } else if (bits_per_sample == 32) {
                int32_t v = read_i32(f);
                s = v / 2147483648.0f;
            }
            sum += s;
        }
        mono[i] = static_cast<float>(sum / num_channels);
    }

    return mono;
}

} // namespace resamp::io
