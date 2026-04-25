#pragma once
#include "synth_params.hpp"
#include <string>

namespace resamp {

// "g-10B50t5" 등의 UTAU 플래그 문자열 → SynthParams
// 알 수 없는 플래그는 무시
SynthParams parse_flags(const std::string& flag_str);

} // namespace resamp
