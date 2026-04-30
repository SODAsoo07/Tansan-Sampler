#pragma once
#include <string>
#include <vector>
#include <cstdint>

namespace resamp::io {

// mono float [-1.0, +1.0] → 16bit PCM WAV 파일 저장
void save_wav(const std::string& path,
              const std::vector<float>& samples,
              uint32_t sample_rate = 44100);

} // namespace resamp::io
