#pragma once
#include "synth_params.hpp"
#include <string>

namespace resamp {

// @preset 토큰을 tansanSampler.txt 설정 파일의 플래그 묶음으로 확장한다.
// 프리셋에서 나온 플래그를 먼저 배치하고, 사용자가 직접 적은 플래그는 마지막에
// 유지해 직접 입력값이 프리셋 값보다 우선 적용되게 한다.
std::string expand_flag_presets(const std::string& flag_str,
                                const std::string& executable_path = std::string());

// "g-10B50t5" 등의 UTAU 플래그 문자열 → SynthParams
// 알 수 없는 플래그는 무시
SynthParams parse_flags(const std::string& flag_str);

} // namespace resamp
