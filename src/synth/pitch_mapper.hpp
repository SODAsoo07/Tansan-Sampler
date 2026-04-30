#pragma once
#include "args/arg_parser.hpp"
#include "flags/synth_params.hpp"
#include <vector>

namespace resamp::synth {

// 샘플 단위 타깃 F0 배열 생성
// RenderParams: target_hz, pitch_bend, modulation, tempo
// output_samples: 생성할 샘플 수
std::vector<double> make_f0_contour(const RenderParams& params,
                                    const SynthParams&  sp,
                                    int output_samples,
                                    int sample_rate);

} // namespace resamp::synth
