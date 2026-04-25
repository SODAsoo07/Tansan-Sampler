#pragma once
#include "lpc_analyzer.hpp"
#include <vector>

namespace resamp::analysis {

// LPC 역필터: 원본 신호 → 잔차(글로탈 소스)
// e[n] = s[n] + a[1]*s[n-1] + ... + a[P]*s[n-P]
std::vector<float> extract_residual(const std::vector<float>& signal,
                                    const LpcCoeffs& lpc);

// LPC 합성 필터: 잔차 → 재합성 신호
// s[n] = e[n]*gain - a[1]*s[n-1] - ... - a[P]*s[n-P]
// (= Kelly-Lochbaum 성도 필터의 Direct Form 구현)
std::vector<float> synthesize_from_residual(const std::vector<float>& residual,
                                            const LpcCoeffs& lpc);

} // namespace resamp::analysis
