#pragma once
#include <vector>

namespace resamp::analysis {

// FRQ 파일 없을 때 자기상관 기반 F0 추정 (fallback)
// 반환: Hz (0.0 = 무성음)
double detect_f0_autocorr(const float* frame, int frame_len,
                           int sample_rate,
                           double f0_min = 60.0,
                           double f0_max = 1100.0);

// 신호 전체에 hop_size 간격으로 F0 추정 → 벡터
std::vector<double> estimate_f0_contour(const std::vector<float>& signal,
                                        int sample_rate,
                                        int frame_size = 1024,
                                        int hop_size   = 256);

} // namespace resamp::analysis
