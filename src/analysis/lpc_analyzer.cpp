#include "lpc_analyzer.hpp"
#include "util/math_util.hpp"
#include <cmath>
#include <algorithm>

namespace resamp::analysis {

LpcCoeffs analyze_lpc(const float* frame, int frame_len,
                      int order, int sample_rate) {
    // 차수 자동 결정: 프리엠퍼시스 후 분석이므로 16차로도 F1~F8 포먼트 충분
    // 24차 이상은 배음 구조까지 모델링 → 피치 의존적 반향감의 원인
    if (order <= 0) order = 16;
    if (order > 20)  order = 20;          // 안전 상한
    if (order > frame_len / 2) order = frame_len / 2;

    // ── 자기상관 계산 ──────────────────────────────────────────────────────
    auto R = math::autocorr(frame, frame_len, order);

    LpcCoeffs result;
    result.order = order;
    result.a.resize(order, 0.0f);
    result.k.resize(order, 0.0f);

    if (R[0] < 1e-30) {
        // 무음: gain=0
        result.gain = 0.0f;
        return result;
    }

    // ── Levinson-Durbin ────────────────────────────────────────────────────
    // a_cur[i]: 현재 차수의 LPC 계수 (0-indexed, a[0] = a_1)
    std::vector<double> a_cur(order, 0.0);
    double error = R[0];

    for (int m = 0; m < order; ++m) {
        // 반사 계수 계산
        double lambda = -R[m + 1];
        for (int j = 0; j < m; ++j)
            lambda -= a_cur[j] * R[m - j];
        lambda /= error;

        // 안정성 클리핑
        if (lambda > 0.9999)  lambda =  0.9999;
        if (lambda < -0.9999) lambda = -0.9999;

        result.k[m] = static_cast<float>(lambda);

        // Schur 갱신: a_new[i] = a_cur[i] + lambda * a_cur[m-1-i]
        std::vector<double> a_new = a_cur;
        for (int j = 0; j < m; ++j)
            a_new[j] = a_cur[j] + lambda * a_cur[m - 1 - j];
        a_new[m] = lambda;
        a_cur = a_new;

        error *= (1.0 - lambda * lambda);
        if (error < 1e-30) { error = 1e-30; break; }
    }

    // Bandwidth expansion (pole 반지름 축소) — γ^i 스케일로 안정성 보장
    // γ=0.997: 안정성 보장용 최소 bandwidth expansion
    // 소스도 프리엠퍼시스 처리되므로 스펙트럼 불일치가 해소됨 → 포먼트를 넓힐 필요 없음
    // 0.994는 Δ BW≈84Hz 추가 → 포먼트 과대역화 → 먹먹함의 또 다른 원인
    {
        const double gamma = 0.997;
        double gk = gamma;
        for (int i = 0; i < order; ++i, gk *= gamma)
            result.a[i] = static_cast<float>(a_cur[i] * gk);
    }
    result.gain = static_cast<float>(std::sqrt(std::max(error, 1e-30)));

    return result;
}

// ── Bilinear 주파수 워핑 ────────────────────────────────────────────────
// LPC 계수 a[]를 z^{-1} → (z^{-1}-λ)/(1-λz^{-1}) 치환으로 변환
// warp_lambda > 0 → 포먼트 상승 (여성화), < 0 → 포먼트 하강 (남성화)
LpcCoeffs warp_lpc(const LpcCoeffs& lpc, float warp_lambda) {
    if (std::fabs(warp_lambda) < 1e-6f) return lpc;

    int P = lpc.order;
    // 워핑된 계수를 Levinson-Durbin 없이 직접 계산
    // 방법: 올파-래더 방식으로 워핑된 자기상관 재구성
    // 간단한 구현: a[] 계수에 bilinear warping 적용
    // A(z) = 1 + a1*z^-1 + ... + aP*z^-P
    // A_w(z) = A(z(zeta)) where z(zeta) = (zeta + lambda)/(1 + lambda*zeta)
    // 점화식으로 계산:
    std::vector<double> aw(P + 1, 0.0);
    aw[0] = 1.0;
    for (int i = 0; i < P; ++i)
        aw[i + 1] = lpc.a[i];

    // 역워핑 반복 (P번)
    double lam = static_cast<double>(warp_lambda);
    for (int iter = 0; iter < P; ++iter) {
        std::vector<double> bw(P + 1, 0.0);
        bw[0] = aw[0];
        for (int i = 1; i <= P; ++i)
            bw[i] = aw[i] - lam * (aw[i - 1] - (i < P ? bw[i - 1] : 0.0));
        aw = bw;
    }

    LpcCoeffs out = lpc;
    for (int i = 0; i < P; ++i)
        out.a[i] = static_cast<float>(aw[i + 1]);
    // gain, k는 근사적으로 유지 (정확한 재계산은 비용이 큼)
    return out;
}

} // namespace resamp::analysis
