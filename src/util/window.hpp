#pragma once
#include <cmath>
#include <vector>

namespace resamp::window {

static constexpr double PI = 3.14159265358979323846;

// Hann 윈도우 (n = 0..N-1)
inline std::vector<float> hann(int n) {
    std::vector<float> w(n);
    for (int i = 0; i < n; ++i)
        w[i] = static_cast<float>(0.5 - 0.5 * std::cos(2.0 * PI * i / (n - 1)));
    return w;
}

// Hamming 윈도우
inline std::vector<float> hamming(int n) {
    std::vector<float> w(n);
    for (int i = 0; i < n; ++i)
        w[i] = static_cast<float>(0.54 - 0.46 * std::cos(2.0 * PI * i / (n - 1)));
    return w;
}

// 인플레이스 윈도우 곱
inline void apply_hann(float* data, int n) {
    for (int i = 0; i < n; ++i)
        data[i] *= static_cast<float>(0.5 - 0.5 * std::cos(2.0 * PI * i / (n - 1)));
}

} // namespace resamp::window
