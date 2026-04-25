#pragma once
#include "flags/synth_params.hpp"
#include <vector>

namespace resamp::synth {

// LF 글로탈 소스 생성기
// f0_contour: 샘플 단위 F0 (0=무성음)
// 반환: 글로탈 소스 신호 (voiced + unvoiced 혼합)
std::vector<float> generate_source(const std::vector<double>& f0_contour,
                                   int sample_rate,
                                   const SynthParams& params);

} // namespace resamp::synth
