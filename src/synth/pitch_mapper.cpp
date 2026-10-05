#include "pitch_mapper.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace resamp::synth {
std::vector<double> make_f0_contour(const RenderParams& params, const SynthParams& sp,
                                   int output_samples, int sample_rate) {
    if (output_samples < 0 || sample_rate <= 0)
        throw std::invalid_argument("Invalid pitch output dimensions");
    if (!std::isfinite(params.tempo) || params.tempo <= 0.0)
        throw std::invalid_argument("Tempo must be finite and positive");
    std::vector<double> f0(output_samples, params.target_hz);
    if (params.target_hz <= 0.0) return f0;
    // The host includes pitchLeadingMs; wavtool applies skipOver. No extra shift.
    const double interval_samples = sample_rate * 60.0 / params.tempo * 5.0 / 480.0;
    for (int i = 0; i < output_samples; ++i) {
        double cents = sp.pitch_cents;
        if (!params.pitch_bend.empty()) {
            const size_t last = params.pitch_bend.size() - 1;
            const double x = std::min(static_cast<double>(last), i / interval_samples);
            const size_t a = static_cast<size_t>(x), b = std::min(a + 1, last);
            const double ca = std::clamp(params.pitch_bend[a], -2400, 2400);
            const double cb = std::clamp(params.pitch_bend[b], -2400, 2400);
            cents += ca + (cb - ca) * (x - a);
        }
        // Preserve explicit bends/vibrato and hold the last value in output padding.
        f0[i] *= std::exp2(cents / 1200.0);
    }
    return f0;
}
} // namespace resamp::synth
