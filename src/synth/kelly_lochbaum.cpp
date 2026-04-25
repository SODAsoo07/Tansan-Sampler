#include "kelly_lochbaum.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace resamp::synth {

KellyLochbaumFilter::KellyLochbaumFilter(int max_order)
    : a_(max_order, 0.0f), a_prev_(max_order, 0.0f),
      hist_(max_order, 0.0f) {}

void KellyLochbaumFilter::set_coeffs(const analysis::LpcCoeffs& lpc, bool smooth,
                                     bool apply_gain) {
    int P = lpc.order;
    if (P > static_cast<int>(a_.size())) {
        a_.resize(P, 0.0f);
        a_prev_.resize(P, 0.0f);
        hist_.resize(P, 0.0f);
    }
    if (smooth && order_ > 0) {
        a_prev_   = a_;
        gain_prev_= gain_;
        smooth_cnt_ = SMOOTH_N;
    } else {
        smooth_cnt_ = 0;
    }
    for (int i = 0; i < P; ++i) a_[i] = lpc.a[i];
    gain_     = apply_gain ? lpc.gain : 1.0f;
    use_gain_ = apply_gain;
    order_ = P;
}

void KellyLochbaumFilter::reset() {
    std::fill(hist_.begin(), hist_.end(), 0.0f);
    smooth_cnt_ = 0;
}

float KellyLochbaumFilter::process(float e) {
    if (order_ == 0) return e;

    // smooth 전환: 이전/현재 계수 선형 보간
    float t = (smooth_cnt_ > 0)
        ? static_cast<float>(smooth_cnt_) / SMOOTH_N
        : 0.0f;
    if (smooth_cnt_ > 0) --smooth_cnt_;

    // s[n] = e[n]*gain - sum_{i=1}^{P} a[i]*s[n-i]
    float gain_now = gain_ + t * (gain_prev_ - gain_);
    float s = e * gain_now;
    for (int i = 0; i < order_; ++i) {
        float a_now = a_[i] + t * (a_prev_[i] - a_[i]);
        s -= a_now * hist_[i];
    }

    // 수치 안정성: 발산 감지 시 히스토리 전체 초기화
    // s=0 만 하면 hist_에 큰 값이 남아 계속 발산 → 무음 출력 버그
    if (!std::isfinite(s) || std::fabs(s) > 1e6f) {
        s = 0.0f;
        std::fill(hist_.begin(), hist_.end(), 0.0f);
    }

    // 히스토리 업데이트
    for (int i = order_ - 1; i > 0; --i)
        hist_[i] = hist_[i - 1];
    hist_[0] = s;

    return s;
}

void KellyLochbaumFilter::process_block(const float* src, float* dst, int n) {
    for (int i = 0; i < n; ++i)
        dst[i] = process(src[i]);
}

float KellyLochbaumFilter::energy() const {
    float e = 0.0f;
    for (float h : hist_) e += h * h;
    return e;
}

} // namespace resamp::synth
