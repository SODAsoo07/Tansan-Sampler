#pragma once
#include "analysis/lpc_analyzer.hpp"
#include <vector>

namespace resamp::synth {

// Kelly-Lochbaum 성도 래더 필터 (= LPC 합성 필터 Direct Form)
// stateful: 프레임 간 연속성을 위해 상태 유지
class KellyLochbaumFilter {
public:
    explicit KellyLochbaumFilter(int max_order = 64);

    // 계수 설정 (새 프레임마다 호출)
    // smooth: 이전 계수에서 서서히 전환 (팝 노이즈 방지)
    // apply_gain: false면 gain=1.0으로 합성 (새 소스 사용 시 권장)
    void set_coeffs(const analysis::LpcCoeffs& lpc, bool smooth = true,
                    bool apply_gain = true);

    // 상태 초기화 (자음→모음 경계 등)
    void reset();

    // 샘플 단위 합성
    float process(float source_sample);

    // 블록 처리 (인플레이스)
    void process_block(const float* src, float* dst, int n);

    // 현재 필터 에너지 (디버그용)
    float energy() const;

private:
    std::vector<float> a_;      // LPC 계수 (현재)
    std::vector<float> a_prev_; // LPC 계수 (이전, smooth용)
    std::vector<float> hist_;   // 출력 히스토리 s[n-1..n-P]
    float gain_      = 1.0f;
    float gain_prev_ = 1.0f;
    bool  use_gain_  = true;
    int   order_    = 0;
    int   smooth_cnt_ = 0;         // smooth 전환 카운터
    static constexpr int SMOOTH_N = 32; // smooth 전환 샘플 수 (64→32: 전환 시간 단축)
};

} // namespace resamp::synth
