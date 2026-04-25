#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>

namespace resamp::math {

static constexpr double PI  = 3.14159265358979323846;
static constexpr double TAU = 6.28318530717958647692;

// ── 자기상관 ───────────────────────────────────────────────────────────────
// r[k] = sum_{n=0}^{N-1-k} x[n]*x[n+k]  (k = 0..max_lag)
std::vector<double> autocorr(const float* x, int n, int max_lag);

// ── 선형 보간 ─────────────────────────────────────────────────────────────
inline double lerp(double a, double b, double t) { return a + (b - a) * t; }
inline float  lerpf(float a, float b, float t)   { return a + (b - a) * t; }

// ── RMS ───────────────────────────────────────────────────────────────────
// NaN/inf 샘플은 제외하여 계산 (필터 발산 방어)
inline double rms(const float* x, int n) {
    double s = 0.0;
    int cnt = 0;
    for (int i = 0; i < n; ++i) {
        if (std::isfinite(x[i])) { s += x[i] * x[i]; ++cnt; }
    }
    return (cnt > 0) ? std::sqrt(s / cnt) : 0.0;
}

// ── 클램프 ────────────────────────────────────────────────────────────────
template<typename T>
inline T clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ── 주파수 → 반음 거리 (A4=69 기준) ─────────────────────────────────────
inline double hz_to_semitone(double hz) {
    return 12.0 * std::log2(hz / 440.0) + 69.0;
}
inline double semitone_to_hz(double semi) {
    return 440.0 * std::pow(2.0, (semi - 69.0) / 12.0);
}

} // namespace resamp::math
