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
    // 안정성 우선: 선형 보간 + deadband(아주 미세 요동만 무시).
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
                double cents  = bend_cents[ix0] * (1.0 - t) + bend_cents[ix1] * t;
                cents_contour[i] = cents;
            }
        }
    }

    // cents contour zero-phase smoothing (IIR 없이 미세 jitter 제거)
    if (has_effective_bend) {
        int radius = std::max(1, static_cast<int>(std::round(sample_rate * 0.0012))); // 1.2ms
        smooth_cents_zero_phase(cents_contour, radius);
    }

    // contour 적용
    if (has_effective_bend) {
        for (int i = 0; i < output_samples; ++i) {
            f0[i] = f0[i] * std::pow(2.0, cents_contour[i] / 1200.0);
        }
    }

    // UTAU modulation은 "원본 피치 성분 혼합량" 계열 의미이며,
    // 여기서 합성 LFO 비브라토를 만들면 오히려 인위적 아티팩트를 유발하므로 비활성.
    (void)sample_rate;
    (void)params.modulation;

    return f0;
}

} // namespace resamp::synth
