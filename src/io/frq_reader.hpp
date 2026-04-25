#pragma once
#include <string>
#include <vector>

namespace resamp::io {

struct FrqFrame {
    double f0;        // 기본 주파수 Hz (0=무성)
    double amplitude; // 진폭 (정규화)
};

struct FrqData {
    bool   valid           = false;
    int    samples_per_frame = 256; // wavetool 기본값
    double average_f0      = 0.0;
    std::vector<FrqFrame> frames;
};

// "<wav_path의 확장자를 .frq로 바꾼 경로>" 시도
// 파일이 없거나 파싱 실패 시 valid=false 반환
FrqData load_frq(const std::string& wav_path);

// frq에서 특정 샘플 위치의 F0 보간 반환 (없으면 0)
double frq_get_f0(const FrqData& frq, int sample_pos);

} // namespace resamp::io
