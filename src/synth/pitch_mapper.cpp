#include "pitch_mapper.hpp"
#include "util/math_util.hpp"
#include <cmath>
#include <algorithm>

namespace resamp::synth {

// zero-phase(대칭 FIR) smoothing on cents contour.
static void smooth_cents_zero_phase(std::vector<double>& cents, int radius) {
    if (radius <= 0 || cents.empty()) return;
    std::vector<double> in = cents;
    int n = static_cast<int>(cents.size());
    for (int i = 0; i < n; ++i) {
        double sum_w = 0.0;
        double sum_v = 0.0;
        for (int k = -radius; k <= radius; ++k) {
            int j = i + k;
            if (j < 0 || j >= n) continue;
            double w = static_cast<double>(radius + 1 - std::abs(k));
            sum_w += w;
            sum_v += w * in[j];
        }
        if (sum_w > 0.0) cents[i] = sum_v / sum_w;
    }
}

// 급격한 pitch-bend 코너만 선택적으로 둥글게 만든다.
// 일반 비브라토 주기(대략 5~8Hz)는 유지하고, 짧은 시간의 꺾임/계단만 완화한다.
static void soften_pitch_kinks(std::vector<double>& cents, int sample_rate) {
    if (cents.empty() || sample_rate <= 0) return;
    int n = static_cast<int>(cents.size());
    int radius = std::max(1, static_cast<int>(std::round(sample_rate * 0.0060))); // 6ms 후보
    int hop = std::max(1, static_cast<int>(std::round(sample_rate * 0.0030)));    // 3ms 곡률 측정

    std::vector<double> smoothed = cents;
    smooth_cents_zero_phase(smoothed, radius);

    std::vector<double> out = cents;
    for (int i = 0; i < n; ++i) {
        int a = std::max(0, i - hop);
        int b = std::min(n - 1, i + hop);
        double left = cents[i] - cents[a];
        double right = cents[b] - cents[i];
        double curvature = std::fabs(right - left);
        double local_slope_c_per_ms =
            std::max(std::fabs(left), std::fabs(right)) /
            std::max(1.0, static_cast<double>(hop) * 1000.0 / sample_rate);

        double kink = std::clamp((curvature - 3.0) / 18.0, 0.0, 1.0);
        double fast_slope = std::clamp((local_slope_c_per_ms - 7.0) / 20.0, 0.0, 1.0);
        double blend = std::clamp(0.10 * fast_slope + 0.68 * kink, 0.0, 0.72);
        if (blend > 1.0e-4) {
            out[i] = cents[i] * (1.0 - blend) + smoothed[i] * blend;
        }
    }
    cents.swap(out);
}

static double bounded_catmull_rom(double p0,
                                  double p1,
                                  double p2,
                                  double p3,
                                  double t) {
    t = std::clamp(t, 0.0, 1.0);
    double t2 = t * t;
    double t3 = t2 * t;
    double v = 0.5 * ((2.0 * p1) +
                      (-p0 + p2) * t +
                      (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
                      (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
    double lo = std::min(p1, p2);
    double hi = std::max(p1, p2);
    double pad = std::max(1.5, 0.18 * std::max(1.0, std::fabs(p2 - p1)));
    return std::clamp(v, lo - pad, hi + pad);
}

static double smoothstep01(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

static void stabilize_boundary_pitch(std::vector<double>& cents,
                                     int sample_rate,
                                     const RenderParams& params) {
    if (cents.empty() || sample_rate <= 0) return;

    const int n = static_cast<int>(cents.size());
    const double note_ms = static_cast<double>(n) * 1000.0 / sample_rate;
    if (note_ms < 24.0) return;

    auto apply_edge = [&](bool head, double guard_ms) {
        int guard = std::clamp(static_cast<int>(std::round(guard_ms * sample_rate / 1000.0)),
                               1,
                               std::max(1, n / 4));
        if (guard < 4 || guard >= n) return;

        const int ref_idx = head ? guard : (n - guard - 1);
        const double ref = cents[std::clamp(ref_idx, 0, n - 1)];

        double motion = 0.0;
        if (head) {
            for (int i = 0; i < guard; ++i) {
                motion = std::max(motion, std::fabs(cents[i] - ref));
            }
        } else {
            for (int i = n - guard; i < n; ++i) {
                motion = std::max(motion, std::fabs(cents[i] - ref));
            }
        }

        const double short_edge = std::clamp((180.0 - note_ms) / 120.0, 0.0, 1.0);
        const double base_strength = std::clamp((motion - 8.0) / 58.0, 0.0, 0.58);
        const double dense_strength = std::clamp((motion - 4.0) / 68.0, 0.0, 0.24) * short_edge;
        const double strength = std::max(base_strength, dense_strength);
        if (strength <= 1.0e-4) return;

        if (head) {
            for (int i = 0; i < guard; ++i) {
                double u = static_cast<double>(i) / std::max(1, guard - 1);
                double w = strength * (1.0 - smoothstep01(u));
                cents[i] = cents[i] * (1.0 - w) + ref * w;
            }
        } else {
            for (int i = n - guard; i < n; ++i) {
                double u = static_cast<double>(i - (n - guard)) / std::max(1, guard - 1);
                double w = strength * smoothstep01(u);
                cents[i] = cents[i] * (1.0 - w) + ref * w;
            }
        }
    };

    const double short_amt = std::clamp((220.0 - note_ms) / 160.0, 0.0, 1.0);
    const double head_ms = std::clamp(18.0 + 0.30 * params.consonant_ms + 7.0 * short_amt,
                                      18.0,
                                      54.0);
    const double tail_ms = std::clamp(24.0 + 6.0 * short_amt, 18.0, 38.0);
    apply_edge(true, head_ms);
    apply_edge(false, tail_ms);
}

std::vector<double> make_f0_contour(const RenderParams& params,
                                    const SynthParams& sp,
                                    int output_samples,
                                    int sample_rate) {
    std::vector<double> f0(output_samples, params.target_hz);

    if (params.target_hz <= 0.0) return f0; // rest

    // 플래그 t: 노트 전체 cents 오프셋 (피치 벤드와 독립)
    if (sp.pitch_cents != 0) {
        double ratio = std::pow(2.0, static_cast<double>(sp.pitch_cents) / 1200.0);
        for (double& v : f0) v *= ratio;
    }

    // ── pitch_bend 적용 ──────────────────────────────────────────────────
    // 내부 pitch_bend 단위는 cent.
    // (UTAU/OpenUtau 12bit 포맷은 그대로 cent, legacy int8은 10cent→cent 변환됨)
    // 안정성 우선: 제한 cubic 보간 + deadband(아주 미세 요동만 무시).
    std::vector<double> cents_contour(output_samples, 0.0);
    bool has_effective_bend = false;
    if (!params.pitch_bend.empty()) {
        int bend_size = static_cast<int>(params.pitch_bend.size());
        std::vector<double> bend_cents(bend_size, 0.0);
        double cmin = 1.0e18;
        double cmax = -1.0e18;
        double cabsmax = 0.0;
        for (int i = 0; i < bend_size; ++i) {
            // 극단값 클램프(안정성): 비정상 decode/입력으로 인한 폭주 방지
            double c = static_cast<double>(params.pitch_bend[i]);
            c = std::clamp(c, -2400.0, 2400.0);
            bend_cents[i] = c;
            cmin = std::min(cmin, bend_cents[i]);
            cmax = std::max(cmax, bend_cents[i]);
            cabsmax = std::max(cabsmax, std::fabs(c));
        }

        // range만 보면 "상수 오프셋"을 놓쳐 실제 음정이 틀어질 수 있다.
        // 따라서 range와 절대 크기를 함께 본다.
        bool ignore_small_bend = ((cmax - cmin) <= 2.0 && cabsmax <= 2.0);
        if (!ignore_small_bend) {
            has_effective_bend = true;
            double denom = std::max(1, output_samples - 1);
            for (int i = 0; i < output_samples; ++i) {
                // [0, bend_size-1] 구간으로 매핑해 endpoint를 정확히 보존
                double pos01 = static_cast<double>(i) / denom;
                double x     = pos01 * std::max(0, bend_size - 1);
                int ix0      = static_cast<int>(x);
                if (ix0 < 0) ix0 = 0;
                if (ix0 >= bend_size) ix0 = bend_size - 1;
                int ix1      = std::min(ix0 + 1, bend_size - 1);
                double t      = x - ix0;
                t = std::clamp(t, 0.0, 1.0);
                int ixm1 = std::max(0, ix0 - 1);
                int ix2  = std::min(ix0 + 2, bend_size - 1);
                double cents = bounded_catmull_rom(bend_cents[ixm1],
                                                   bend_cents[ix0],
                                                   bend_cents[ix1],
                                                   bend_cents[ix2],
                                                   t);
                cents_contour[i] = cents;
            }
        }
    }

    // cents contour zero-phase smoothing (IIR 없이 미세 jitter 제거)
    if (has_effective_bend) {
        int radius = std::max(1, static_cast<int>(std::round(sample_rate * 0.0018))); // 1.8ms
        smooth_cents_zero_phase(cents_contour, radius);
        soften_pitch_kinks(cents_contour, sample_rate);
        stabilize_boundary_pitch(cents_contour, sample_rate, params);
    }

    // contour 적용
    if (has_effective_bend) {
        for (int i = 0; i < output_samples; ++i) {
            f0[i] = f0[i] * std::pow(2.0, cents_contour[i] / 1200.0);
        }
    }

    // UTAU modulation은 "원본 피치 성분 혼합량" 계열 의미이며,
    // 여기서 합성 LFO 비브라토를 만들면 오히려 인위적 아티팩트를 유발하므로 비활성.
    (void)params.modulation;

    return f0;
}

} // namespace resamp::synth
