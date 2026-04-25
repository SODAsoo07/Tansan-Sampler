#include "residual.hpp"
#include <algorithm>
#include <cmath>

namespace resamp::analysis {

std::vector<float> compute_global_residual(
    const std::vector<float>& signal,
    const std::vector<AnalysisFrame>& frames)
{
    int N = static_cast<int>(signal.size());
    std::vector<float> residual(N, 0.0f);
    if (frames.empty()) {
        std::copy(signal.begin(), signal.end(), residual.begin());
        return residual;
    }

    int n_frames = static_cast<int>(frames.size());
    int fi = 0;
    for (int n = 0; n < N; ++n) {
        // 가장 가까운 프레임으로 fi 전진 (frames는 center_sample 오름차순)
        while (fi + 1 < n_frames &&
               std::abs(frames[fi+1].center_sample - n) <
               std::abs(frames[fi  ].center_sample - n)) ++fi;

        const auto& lpc = frames[fi].lpc;
        int P = lpc.order;
        float e = signal[n];
        for (int k = 0; k < P && n - 1 - k >= 0; ++k)
            e += lpc.a[k] * signal[n - 1 - k];
        residual[n] = e;
    }
    return residual;
}

std::vector<float> extract_residual(const std::vector<float>& signal,
                                    const LpcCoeffs& lpc) {
    int N = static_cast<int>(signal.size());
    int P = lpc.order;
    std::vector<float> res(N, 0.0f);

    for (int n = 0; n < N; ++n) {
        float pred = 0.0f;
        for (int i = 0; i < P && n - i - 1 >= 0; ++i)
            pred += lpc.a[i] * signal[n - i - 1];
        res[n] = signal[n] + pred; // e[n] = s[n] + sum a[i]*s[n-i]
    }
    return res;
}

std::vector<float> synthesize_from_residual(const std::vector<float>& residual,
                                            const LpcCoeffs& lpc) {
    int N = static_cast<int>(residual.size());
    int P = lpc.order;
    std::vector<float> out(N, 0.0f);

    // s[n] = e[n]*gain - sum_{i=1}^{P} a[i]*s[n-i]
    for (int n = 0; n < N; ++n) {
        float s = residual[n] * lpc.gain;
        for (int i = 0; i < P && n - i - 1 >= 0; ++i)
            s -= lpc.a[i] * out[n - i - 1];
        out[n] = s;
    }
    return out;
}

} // namespace resamp::analysis
