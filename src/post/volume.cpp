#include "volume.hpp"
#include "fade.hpp"
#include "util/math_util.hpp"
#include <cmath>
#include <algorithm>

namespace resamp::post {

void apply_volume(std::vector<float>& samples,
                  int volume_param,
                  const SynthParams& sp) {
    if (samples.empty()) return;

    float scale = volume_param / 100.0f;
    float comp  = sp.peak_comp / 100.0f;

    for (auto& s : samples) {
        s *= scale;
        if (comp < 0.99f) {
            float k = (1.0f - comp) * 3.0f;
            if (k > 1e-4f) {
                float norm = std::tanh(k);
                s = std::tanh(k * s) / norm;
            }
        }
    }
}

void normalize_rms(std::vector<float>& samples, float target_rms) {
    if (samples.empty()) return;
    float cur = static_cast<float>(
        math::rms(samples.data(), static_cast<int>(samples.size())));
    if (cur < 1e-10f) return;
    float gain = target_rms / cur;
    for (auto& s : samples) s *= gain;
}

void fade_in(std::vector<float>& samples, int fade_samples) {
    int n = std::min(fade_samples, static_cast<int>(samples.size()));
    for (int i = 0; i < n; ++i)
        samples[i] *= static_cast<float>(i) / fade_samples;
}

void fade_out(std::vector<float>& samples, int fade_samples) {
    int N = static_cast<int>(samples.size());
    int n = std::min(fade_samples, N);
    for (int i = 0; i < n; ++i)
        samples[N - 1 - i] *= static_cast<float>(i) / fade_samples;
}

} // namespace resamp::post
