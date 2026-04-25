#pragma once
#include "lpc_analyzer.hpp"
#include "io/frq_reader.hpp"
#include <vector>

namespace resamp::analysis {

struct AnalysisFrame {
    int   center_sample = 0;   // 원본 신호에서의 중심 샘플 인덱스
    double f0           = 0.0; // Hz (0 = 무성)
    bool  voiced        = false;
    LpcCoeffs lpc;             // 이 프레임의 LPC 계수
    std::vector<float> residual; // LPC 역필터 잔차
};

// 신호를 hop_size 간격으로 프레임 분할하고 LPC 분석까지 수행
// frq: 유효하면 frq에서 F0 읽기, 아니면 autocorr fallback
std::vector<AnalysisFrame> slice_and_analyze(
    const std::vector<float>& signal,
    const io::FrqData&        frq,
    int sample_rate  = 44100,
    int frame_size   = 1024,
    int hop_size     = 256,
    int lpc_order    = 0      // 0=자동
);

} // namespace resamp::analysis
