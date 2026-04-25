#pragma once
#include "analysis/frame_slicer.hpp"
#include "args/arg_parser.hpp"
#include "flags/synth_params.hpp"
#include <vector>

namespace resamp::synth {

// PSOLA 기반 접합: 분석 프레임들을 타깃 길이/피치로 OLA 합산
//
// 처리 순서:
//   1. 자음 영역: velocity에 따른 시간 스케일
//   2. 모음 영역: 루프/크롭으로 target_length 충족
//   3. 각 합성 마커에서 가장 가까운 분석 마커의 윈도우를 OLA 합산
//
// signal: 원본 신호 (자름 적용 후)
// frames: slice_and_analyze() 결과
// f0_contour: 타깃 F0 (샘플 단위, output_samples 길이)
// output_samples: 출력 샘플 수
std::vector<float> psola_splice(
    const std::vector<float>&              signal,
    const std::vector<analysis::AnalysisFrame>& frames,
    const std::vector<double>&             f0_contour,
    const RenderParams&                    params,
    const SynthParams&                     sp,
    int                                    sample_rate,
    int                                    output_samples
);

} // namespace resamp::synth
