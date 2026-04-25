#include "residual.hpp"
#include <algorithm>

namespace resamp::analysis {

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
