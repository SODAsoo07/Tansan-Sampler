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

static float max_abs_sample(const std::vector<float>& samples) {
    float peak = 0.0f;
    for (float s : samples) peak = std::max(peak, std::fabs(s));
    return peak;
}

static void apply_peak_guard(std::vector<float>& samples, float ceiling) {
    if (samples.empty()) return;
    ceiling = std::clamp(ceiling, 0.50f, 0.985f);
    float peak = max_abs_sample(samples);
    if (peak <= ceiling || peak <= 1.0e-8f) return;

    float knee = ceiling * 0.82f;
    float inv_range = 1.0f / std::max(1.0e-6f, peak - knee);
    for (auto& s : samples) {
        float a = std::fabs(s);
        if (a <= knee) continue;
        float sign = (s < 0.0f) ? -1.0f : 1.0f;
        float x = std::clamp((a - knee) * inv_range, 0.0f, 1.0f);
        float limited = knee + (ceiling - knee) * std::tanh(2.2f * x) / std::tanh(2.2f);
        s = sign * limited;
    }

    peak = max_abs_sample(samples);
    if (peak > ceiling && peak > 1.0e-8f) {
        float g = ceiling / peak;
        for (auto& s : samples) s *= g;
    }
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

static float rms_slice(const std::vector<float>& samples, int begin, int end) {
    if (samples.empty()) return 0.0f;
    begin = std::clamp(begin, 0, static_cast<int>(samples.size()));
    end = std::clamp(end, begin, static_cast<int>(samples.size()));
    if (end <= begin) return 0.0f;
    double sum2 = 0.0;
    for (int i = begin; i < end; ++i) {
        sum2 += static_cast<double>(samples[i]) * samples[i];
    }
    return static_cast<float>(std::sqrt(sum2 / std::max(1, end - begin)));
}

static float peak_slice(const std::vector<float>& samples, int begin, int end) {
    if (samples.empty()) return 0.0f;
    begin = std::clamp(begin, 0, static_cast<int>(samples.size()));
    end = std::clamp(end, begin, static_cast<int>(samples.size()));
    float peak = 0.0f;
    for (int i = begin; i < end; ++i) peak = std::max(peak, std::fabs(samples[i]));
    return peak;
}

struct LevelStats {
    float p90 = 0.0f;
    float p95 = 0.0f;
    float p995 = 0.0f;
    float peak = 0.0f;
    float rms = 0.0f;
};

static LevelStats measure_level(const std::vector<float>& samples) {
    LevelStats st;
    if (samples.empty()) return st;
    st.p90 = abs_percentile(samples, 0.90f);
    st.p95 = abs_percentile(samples, 0.95f);
    st.p995 = abs_percentile(samples, 0.995f);
    st.peak = max_abs_sample(samples);
    float gate_abs = std::max(0.0018f, std::min(st.p90 * 0.16f, st.p95 * 0.10f));
    st.rms = gated_rms(samples, gate_abs);
    return st;
}

static void apply_makeup_level(std::vector<float>& samples, int sample_rate, const SynthParams& sp) {
    if (samples.empty()) return;

    LevelStats st = measure_level(samples);
    if (st.rms <= 1.0e-8f || st.peak <= 1.0e-8f) return;

    float ln = std::clamp(sp.loud_norm / 100.0f, -1.0f, 1.0f);
    float ln_pos = std::max(0.0f, ln);
    float ln_neg = std::max(0.0f, -ln);
    float fx_load = std::clamp(0.55f * (sp.distortion / 100.0f) +
                               0.35f * (sp.bitcrusher / 100.0f) +
                               0.22f * (std::abs(sp.final_filter)), 0.0f, 1.0f);
    float duration_ms = (sample_rate > 0)
        ? (static_cast<float>(samples.size()) * 1000.0f / static_cast<float>(sample_rate))
        : 1000.0f;
    float short_note = std::clamp((360.0f - duration_ms) / 240.0f, 0.0f, 1.0f);
    short_note = short_note * short_note * (3.0f - 2.0f * short_note);

    float target_rms = 0.118f + 0.020f * ln_pos - 0.018f * ln_neg;
    float target_p95 = 0.50f + 0.048f * ln_pos - 0.055f * ln_neg;
    float target_p995 = 0.91f - 0.055f * fx_load + 0.018f * ln_pos - 0.030f * ln_neg;
    float ceiling = 0.982f - 0.055f * fx_load;

    float gain = 1.0f;
    if (st.rms < target_rms * 0.82f) {
        float g_rms = target_rms / st.rms;
        float g_p95 = (st.p95 > 1.0e-8f) ? (target_p95 / st.p95) : g_rms;
        float g_peak = (st.p995 > 1.0e-8f) ? (target_p995 / st.p995) : g_rms;
        float headroom_gain = std::min(g_p95 * 1.10f, g_peak);
        float wanted = std::min(g_rms, headroom_gain);
        wanted = std::clamp(wanted, 1.0f, 2.15f + 2.35f * short_note - 0.45f * fx_load);
        float makeup_blend = std::clamp(0.48f + 0.30f * short_note + 0.32f * ln_pos - 0.30f * ln_neg,
                                        0.18f, 0.92f);
        gain = 1.0f + makeup_blend * (wanted - 1.0f);
    } else if (st.rms > target_rms * 1.45f || st.p995 > target_p995 * 1.05f) {
        float g_rms = (target_rms * 1.18f) / st.rms;
        float g_peak = (st.p995 > 1.0e-8f) ? (target_p995 / st.p995) : 1.0f;
        gain = std::clamp(std::min(g_rms, g_peak), 0.62f, 1.0f);
    }

    if (std::fabs(gain - 1.0f) > 0.003f) {
        for (auto& s : samples) s *= gain;
    }
    apply_peak_guard(samples, ceiling);
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
    LevelStats st = measure_level(samples);
    float cur_p95 = st.p95;
    float cur_p995 = st.p995;
    float cur_rms = st.rms;
    float gate_abs = std::max(0.0018f, std::min(st.p90 * 0.16f, st.p95 * 0.10f));

    float ln = std::clamp(sp.loud_norm / 100.0f, -1.0f, 1.0f);
    float ln_pos = std::max(0.0f, ln);
    float ln_neg = std::max(0.0f, -ln);
    float target_rms = 0.118f + 0.020f * ln_pos - 0.018f * ln_neg;
    float target_p95 = 0.56f  + 0.035f * ln_pos - 0.060f * ln_neg;
    float target_p995 = 0.93f + 0.008f * ln_pos - 0.035f * ln_neg;

    float gain_rms = (cur_rms > 1e-9f) ? (target_rms / cur_rms) : 1.0f;
    float gain_p95 = (cur_p95 > 1e-9f) ? (target_p95 / cur_p95) : 1.0f;
    float gain_p995 = (cur_p995 > 1e-9f) ? (target_p995 / cur_p995) : 1.0f;
    float norm_gain = std::min({gain_rms, gain_p95, gain_p995});
    norm_gain = std::clamp(norm_gain, 0.38f, 2.75f);
    // 과도한 보정으로 잔향/히스가 전면으로 나오지 않도록 기본은 완화,
    // Ln으로 정규화 개입 강도를 조절한다.
    float norm_blend = std::clamp(0.72f + 0.60f * ln_pos - 0.58f * ln_neg, 0.10f, 1.35f);
    norm_gain = 1.0f + norm_blend * (norm_gain - 1.0f);

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

void apply_boundary_level_guard(std::vector<float>& samples,
                                int sample_rate) {
    if (samples.empty() || sample_rate <= 0) return;

    const int n = static_cast<int>(samples.size());
    const float note_ms = static_cast<float>(n) * 1000.0f / static_cast<float>(sample_rate);
    if (note_ms < 28.0f) return;

    const float dense_note = std::clamp((280.0f - note_ms) / 190.0f, 0.0f, 1.0f);
    const int edge_n = std::clamp(static_cast<int>(std::round(sample_rate * (0.026f + 0.010f * dense_note))),
                                  1,
                                  std::max(1, n / 4));
    const int body_margin = std::clamp(static_cast<int>(std::round(sample_rate * 0.045)),
                                       edge_n,
                                       std::max(edge_n, n / 3));
    int body_begin = std::min(body_margin, n - 1);
    int body_end = std::max(body_begin + 1, n - body_margin);
    if (body_end <= body_begin + 8) {
        body_begin = std::max(0, n / 4);
        body_end = std::max(body_begin + 1, (3 * n) / 4);
    }

    const float body_rms = rms_slice(samples, body_begin, body_end);
    const float body_peak = peak_slice(samples, body_begin, body_end);
    if (body_rms <= 1.0e-5f || body_peak <= 1.0e-5f) return;

    auto edge_gain = [&](float edge_rms, float edge_peak, float min_gain) {
        float rms_gain = (edge_rms > body_rms * 1.18f)
            ? (body_rms * 1.12f / std::max(edge_rms, 1.0e-8f))
            : 1.0f;
        float peak_gain = (edge_peak > body_peak * 1.35f)
            ? (body_peak * 1.24f / std::max(edge_peak, 1.0e-8f))
            : 1.0f;
        return std::clamp(std::min(rms_gain, peak_gain), min_gain, 1.0f);
    };

    float head_gain = edge_gain(rms_slice(samples, 0, edge_n),
                                peak_slice(samples, 0, edge_n),
                                0.84f);
    float tail_gain = edge_gain(rms_slice(samples, n - edge_n, n),
                                peak_slice(samples, n - edge_n, n),
                                0.80f);

    // Dense CVVC/VCV passages often overlap several nominally "normal" edges.
    // In that case relative edge matching alone is not enough, so short notes get
    // a mild fixed de-emphasis. Tail is stronger because it creates most of the
    // perceived residue; head stays conservative to preserve consonant attacks.
    const float dense_head_gain = 1.0f - 0.055f * dense_note;
    const float dense_tail_gain = 1.0f - 0.180f * dense_note;
    head_gain = std::min(head_gain, dense_head_gain);
    tail_gain = std::min(tail_gain, dense_tail_gain);

    if (head_gain < 0.999f) {
        for (int i = 0; i < edge_n; ++i) {
            float u = static_cast<float>(i) / static_cast<float>(std::max(1, edge_n - 1));
            float s = u * u * (3.0f - 2.0f * u);
            float g = head_gain + (1.0f - head_gain) * s;
            samples[i] *= g;
        }
    }

    if (tail_gain < 0.999f) {
        for (int i = n - edge_n; i < n; ++i) {
            float u = static_cast<float>(i - (n - edge_n)) /
                      static_cast<float>(std::max(1, edge_n - 1));
            float s = u * u * (3.0f - 2.0f * u);
            float g = 1.0f + (tail_gain - 1.0f) * s;
            samples[i] *= g;
        }
    }
}

static void apply_distortion(std::vector<float>& samples, int amount) {
    float a = std::clamp(amount / 100.0f, 0.0f, 1.0f);
    if (a <= 1.0e-4f) return;

    float pre_p95 = abs_percentile(samples, 0.95f);
    float pre_p999 = abs_percentile(samples, 0.999f);
    float drive = 1.0f + 8.0f * a + 30.0f * a * a;
    float wet = std::clamp(0.24f + 0.76f * a, 0.0f, 0.98f);
    float norm = std::tanh(drive);
    if (norm < 1.0e-6f) norm = 1.0f;
    for (auto& s : samples) {
        float dry = s;
        float biased = s + 0.12f * a * s * std::fabs(s);
        float shaped = std::tanh(drive * biased) / norm;
        s = dry * (1.0f - wet) + shaped * wet;
    }

    float post_p95 = abs_percentile(samples, 0.95f);
    if (pre_p95 > 1.0e-5f && post_p95 > 1.0e-5f) {
        float keep = std::clamp(pre_p95 * (1.0f + 0.18f * a), 0.04f, 0.86f);
        float g = std::clamp(keep / post_p95, 0.35f, 1.15f);
        for (auto& s : samples) s *= g;
    }

    float peak = abs_percentile(samples, 0.999f);
    float peak_target = std::max(0.82f, std::min(0.98f, pre_p999 * (1.0f + 0.10f * a)));
    if (peak > peak_target && peak > 1.0e-5f) {
        float g = peak_target / peak;
        for (auto& s : samples) s *= g;
    }
    apply_peak_guard(samples, 0.96f - 0.06f * a);
}

static void apply_bitcrusher(std::vector<float>& samples, int amount) {
    float a = std::clamp(amount / 100.0f, 0.0f, 1.0f);
    if (a <= 1.0e-4f) return;
    int bits = std::clamp(static_cast<int>(std::round(16.0f - 11.0f * a)), 5, 16);
    int hold = std::clamp(static_cast<int>(std::round(1.0f + 9.0f * a * a)), 1, 10);
    float levels = static_cast<float>(1 << (bits - 1));
    float held = 0.0f;
    for (size_t i = 0; i < samples.size(); ++i) {
        if ((i % static_cast<size_t>(hold)) == 0) {
            held = std::round(std::clamp(samples[i], -1.0f, 1.0f) * levels) / levels;
        }
        samples[i] = held;
    }
}

static void apply_one_pole_lowpass(std::vector<float>& samples, int sample_rate, float cutoff_hz) {
    if (samples.empty() || sample_rate <= 0) return;
    float dt = 1.0f / static_cast<float>(sample_rate);
    float rc = 1.0f / (2.0f * 3.14159265358979323846f * cutoff_hz);
    float alpha = dt / (rc + dt);
    float y = samples.front();
    for (auto& s : samples) {
        y += alpha * (s - y);
        s = y;
    }
}

static void apply_one_pole_highpass(std::vector<float>& samples, int sample_rate, float cutoff_hz) {
    if (samples.empty() || sample_rate <= 0) return;
    float dt = 1.0f / static_cast<float>(sample_rate);
    float rc = 1.0f / (2.0f * 3.14159265358979323846f * cutoff_hz);
    float alpha = rc / (rc + dt);
    float y = 0.0f;
    float prev_x = samples.front();
    for (auto& s : samples) {
        float x = s;
        y = alpha * (y + x - prev_x);
        prev_x = x;
        s = y;
    }
}

void apply_flag_post_effects(std::vector<float>& samples,
                             int sample_rate,
                             double consonant_ms,
                             const SynthParams& sp) {
    if (samples.empty()) return;

    if (sp.reverse_mode == 1) {
        std::reverse(samples.begin(), samples.end());
    }

    apply_distortion(samples, sp.distortion);
    apply_bitcrusher(samples, sp.bitcrusher);

    if (sp.reverse_mode != 0 && sample_rate > 0) {
        int fade_in_n = std::min(static_cast<int>(samples.size()), std::max(1, sample_rate / 500));  // ~2ms
        int fade_out_n = std::min(static_cast<int>(samples.size()), std::max(1, sample_rate / 250)); // ~4ms
        for (int i = 0; i < fade_in_n; ++i) {
            samples[i] *= static_cast<float>(i) / static_cast<float>(fade_in_n);
        }
        int n = static_cast<int>(samples.size());
        for (int i = 0; i < fade_out_n; ++i) {
            samples[n - 1 - i] *= static_cast<float>(i) / static_cast<float>(fade_out_n);
        }
    }

    // Fc는 의도적으로 최종 단계에 둔다.
    if (sp.final_filter > 0) {
        bool fry_active = (sp.fry_head > 0 || sp.fry_tail > 0);
        float cutoff = fry_active ? 7600.0f : 5200.0f;
        apply_one_pole_lowpass(samples, sample_rate, cutoff);
        if (!fry_active) apply_one_pole_lowpass(samples, sample_rate, cutoff);
    } else if (sp.final_filter < 0) {
        apply_one_pole_highpass(samples, sample_rate, 180.0f);
        apply_one_pole_highpass(samples, sample_rate, 180.0f);
    }

    apply_makeup_level(samples, sample_rate, sp);
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
