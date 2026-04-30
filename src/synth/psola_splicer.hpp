#pragma once
#include "analysis/frame_slicer.hpp"
#include "args/arg_parser.hpp"
#include "flags/synth_params.hpp"
#include <vector>

namespace resamp::synth {

// TD-PSOLA: 원본 신호 grain을 직접 OLA → 포먼트 자연 보존
//
// signal: 원본 신호 (트리밍 후, raw 도메인)
// frames: 소스 F0/LPC (피치 마크 구성과 gender 워핑용)
// f0_contour: 타겟 F0 (샘플 단위)
std::vector<float> psola_splice(
    const std::vector<float>&                   signal,
    const std::vector<analysis::AnalysisFrame>& frames,
    const std::vector<double>&                  f0_contour,
    const RenderParams&                         params,
    const SynthParams&                          sp,
    int                                         sample_rate,
    int                                         output_samples
);

} // namespace resamp::synth
