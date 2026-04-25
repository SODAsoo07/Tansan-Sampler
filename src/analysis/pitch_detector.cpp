#include "pitch_detector.hpp"
#include "util/math_util.hpp"
#include "util/window.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace resamp::analysis {

double detect_f0_autocorr(const float* frame, int frame_len,
                           int sample_rate,
                           double f0_min, double f0_max) {
    int lag_min = static_cast<int>(sample_rate / f0_max);
    int lag_max = static_cast<int>(sample_rate / f0_min);
    if (lag_max >= frame_len) lag_max = frame_len - 1;

    auto R = math::autocorr(frame, frame_len, lag_max);
    if (R[0] < 1e-20) return 0.0; // 무음

    // 정규화 자기상관
    // 피크 탐색 (lag_min ~ lag_max)
    int best_lag = lag_min;
    double best_val = R[lag_min] / R[0];
    for (int lag = lag_min + 1; lag <= lag_max; ++lag) {
        double val = R[lag] / R[0];
        if (val > best_val) { best_val = val; best_lag = lag; }
    }

    // 유성음 판정 임계값
    if (best_val < 0.3) return 0.0; // 무성음

    // 포물선 보간으로 서브샘플 정밀도
    if (best_lag > lag_min && best_lag < lag_max) {
        double y0 = R[best_lag - 1];
        double y1 = R[best_lag];
        double y2 = R[best_lag + 1];
        double delta = (y2 - y0) / (2.0 * (2.0 * y1 - y0 - y2));
        best_lag = static_cast<int>(best_lag + delta + 0.5);
        if (best_lag < 1) best_lag = 1;
    }

    return static_cast<double>(sample_rate) / best_lag;
}

std::vector<double> estimate_f0_contour(const std::vector<float>& signal,
                                        int sample_rate,
                                        int frame_size, int hop_size) {
    int N = static_cast<int>(signal.size());
    int n_frames = (N - frame_size) / hop_size + 1;
    if (n_frames < 1) n_frames = 1;

    std::vector<double> f0_contour(N, 0.0);
    auto win = window::hann(frame_size);
    std::vector<float> buf(frame_size);

    for (int fi = 0; fi < n_frames; ++fi) {
        int start = fi * hop_size;
        int end   = std::min(start + frame_size, N);
        int actual = end - start;

        for (int i = 0; i < actual; ++i)
            buf[i] = signal[start + i] * win[i];
        for (int i = actual; i < frame_size; ++i)
            buf[i] = 0.0f;

        double f0 = detect_f0_autocorr(buf.data(), frame_size, sample_rate);

        // hop 구간 전체에 동일한 F0 설정 (추후 보간 가능)
        for (int i = start; i < start + hop_size && i < N; ++i)
            f0_contour[i] = f0;
    }

    // 마지막 구간
    int last = (n_frames - 1) * hop_size + hop_size;
    for (int i = last; i < N; ++i)
        f0_contour[i] = f0_contour[last - 1];

    return f0_contour;
}

} // namespace resamp::analysis
