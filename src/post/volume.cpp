#include "volume.hpp"
#include "fade.hpp"
#include "util/math_util.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace resamp::post {

static float abs_percentile(const std::vector<float>& samples, float q) {
    if (samples.empty()) return 0.0f;
    std::vector<float> v;
    v.reserve(samples.size());
    for (float s : samples) v.push_back(std::fabs(s));
    q = std::clamp(q, 0.0f, 1.0f);
    size_t idx = static_cast<size_t>(q * static_cast<float>(v.size() - 1));
    std::nth_element(v.begin(), v.begin() + idx, v.end());
    return v[idx];
}

static float gated_rms(const std::vector<float>& samples, float gate_abs) {
    if (samples.empty()) return 0.0f;
    double sum2 = 0.0;
    int cnt = 0;
    for (float s : samples) {
        float a = std::fabs(s);
        if (a < gate_abs) continue;
        sum2 += static_cast<double>(s) * s;
        ++cnt;
    }
    if (cnt < 16) {
        return static_cast<float>(
            math::rms(samples.data(), static_cast<int>(samples.size())));
    }
    return static_cast<float>(std::sqrt(sum2 / std::max(1, cnt)));
}

void apply_volume(std::vector<float>& samples,
                  int volume_param,
                  const SynthParams& sp) {
    if (samples.empty()) return;

    float user_scale = std::max(0.0f, volume_param / 100.0f);
    if (user_scale <= 1.0e-6f) {
        std::fill(samples.begin(), samples.end(), 0.0f);
        return;
    }
    float comp  = sp.peak_comp / 100.0f;

    // 적응형 loudness 정규화:
    // 다양한 음색/발성에서도 출력 음량을 최대한 일정하게 유지.
    float cur_p95 = abs_percentile(samples, 0.95f);
    float cur_p995 = abs_percentile(samples, 0.995f);
    float gate_abs = std::max(0.0025f, cur_p95 * 0.09f);
    float cur_rms = gated_rms(samples, gate_abs);

    float target_rms = 0.105f;
    float target_p95 = 0.70f;
    float target_p995 = 0.92f;

    float gain_rms = (cur_rms > 1e-9f) ? (target_rms / cur_rms) : 1.0f;
    float gain_p95 = (cur_p95 > 1e-9f) ? (target_p95 / cur_p95) : 1.0f;
    float gain_p995 = (cur_p995 > 1e-9f) ? (target_p995 / cur_p995) : 1.0f;
    float norm_gain = std::min({gain_rms, gain_p95, gain_p995});
    norm_gain = std::clamp(norm_gain, 0.35f, 3.20f);
    // 과도한 보정으로 잔향/히스가 전면으로 나오지 않도록 보정량을 일부 완화.
    norm_gain = 1.0f + 0.80f * (norm_gain - 1.0f);

    for (auto& s : samples) {
        // low-level 성분(잔향/히스)은 완만히 감쇠해 거친 질감 부각 방지.
        float a = std::fabs(s);
        float tail_damp = 1.0f;
        if (a < gate_abs) {
            float t = a / std::max(1.0e-8f, gate_abs);
            tail_damp = 0.55f + 0.45f * t * t;
        }

        s *= norm_gain * user_scale * tail_damp;
        // T+에서 crest factor가 증가하므로 limiter를 소폭 강화.
        float t_pos = std::clamp(sp.tension / 100.0f, 0.0f, 1.0f);
        float comp_eff = std::clamp(comp - 0.10f * t_pos, 0.35f, 0.99f);
        if (comp_eff < 0.99f) {
            float k = (1.0f - comp_eff) * 3.0f;
            if (k > 1e-4f) {
                float norm = std::tanh(k);
                s = std::tanh(k * s) / norm;
            }
        }
    }

    // 최종 hard peak guard: 드문 순간 피크만 정리해 파형 피크 튐 방지.
    float peak = abs_percentile(samples, 0.999f);
    if (peak > 0.985f) {
        float pg = 0.985f / peak;
        for (auto& s : samples) s *= pg;
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
