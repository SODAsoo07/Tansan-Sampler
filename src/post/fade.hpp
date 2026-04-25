#pragma once
#include <vector>

namespace resamp::post {

// 시작 fade-in (samples 인플레이스)
void fade_in(std::vector<float>& samples, int fade_samples);

// 끝 fade-out
void fade_out(std::vector<float>& samples, int fade_samples);

// 양쪽 모두 (편의 함수)
// fade_ms: 밀리초, sample_rate: 샘플레이트
inline void apply_fades(std::vector<float>& samples,
                        double fade_in_ms, double fade_out_ms,
                        int sample_rate) {
    int fi = static_cast<int>(fade_in_ms  * sample_rate / 1000.0);
    int fo = static_cast<int>(fade_out_ms * sample_rate / 1000.0);
    fade_in(samples, fi);
    fade_out(samples, fo);
}

} // namespace resamp::post
