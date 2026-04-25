#pragma once
#include <vector>

namespace resamp::analysis {

struct LpcCoeffs {
    std::vector<float> a;    // a[0..order-1] = a[1..P] (LPC 계수, a[0]=1 암시)
    std::vector<float> k;    // PARCOR 반사 계수 k[0..order-1] = k[1..P]
    float gain = 1.0f;       // 예측 오차 에너지 sqrt
    int   order = 0;
};

// LPC 분석: Levinson-Durbin 알고리즘
// order = 0 → sample_rate/1000 + 4 자동 결정
LpcCoeffs analyze_lpc(const float* frame, int frame_len,
                      int order = 0, int sample_rate = 44100);

// 포먼트 워핑 (g 플래그): 비선형 주파수 스케일링
// warp_lambda: -0.33~+0.33 정도 (g=-100→+0.33, g=+100→-0.33)
LpcCoeffs warp_lpc(const LpcCoeffs& lpc, float warp_lambda);

} // namespace resamp::analysis
