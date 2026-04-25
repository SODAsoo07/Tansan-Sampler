#pragma once
#include <string>
#include <vector>
#include <cstdint>

namespace resamp::io {

struct WavInfo {
    uint32_t sample_rate   = 44100;
    uint16_t num_channels  = 1;
    uint16_t bits_per_sample = 16;
    uint32_t num_samples   = 0;   // 채널당 샘플 수
};

// WAV 파일을 읽어 mono float [-1.0, +1.0] 로 반환
// stereo → 채널 평균, 16/24/32bit PCM 및 32bit float 지원
std::vector<float> load_wav(const std::string& path, WavInfo& info);

} // namespace resamp::io
