#pragma once
#include "flags/synth_params.hpp"
#include <vector>

namespace resamp::post {

// 볼륨 스케일 + 피크 제한 적용
// volume_param: RenderParams.volume (0~200, 100=1.0x)
// sp.peak_comp: P 플래그 (0=강한 제한, 100=제한없음)
void apply_volume(std::vector<float>& samples,
                  int volume_param,
                  const SynthParams& sp);

// 플래그 기반 최종 샘플 후처리.
// Fc 필터는 이 함수 내부에서 가장 마지막에 적용된다.
void apply_flag_post_effects(std::vector<float>& samples,
                             int sample_rate,
                             double consonant_ms,
                             const SynthParams& sp);

// RMS 정규화 (목표 RMS로)
void normalize_rms(std::vector<float>& samples, float target_rms = 0.25f);

} // namespace resamp::post
