#include "math_util.hpp"

namespace resamp::math {

std::vector<double> autocorr(const float* x, int n, int max_lag) {
    std::vector<double> r(max_lag + 1, 0.0);
    for (int k = 0; k <= max_lag; ++k) {
        double s = 0.0;
        for (int i = 0; i < n - k; ++i)
            s += static_cast<double>(x[i]) * x[i + k];
        r[k] = s;
    }
    return r;
}

} // namespace resamp::math
