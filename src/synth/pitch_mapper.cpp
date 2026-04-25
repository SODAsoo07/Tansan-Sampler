#include "pitch_mapper.hpp"
#include "util/math_util.hpp"
#include <cmath>
#include <algorithm>

namespace resamp::synth {

std::vector<double> make_f0_contour(const RenderParams& params,
                                    int output_samples,
                                    int sample_rate) {
    std::vector<double> f0(output_samples, params.target_hz);

    if (params.target_hz <= 0.0) return f0; // rest

    // ── pitch_bend 적용 ──────────────────────────────────────────────────
    // 각 int8_t = 반음×100 단위 (= cents)
    // 서브프레임 크기: 템포·4분음표 1개를 N_BEND_FRAMES로 분할
    // UTAU 관례: 한 노트의 pitch_bend는 96 서브프레임
    const int N_SUBFRAMES = 96;
    if (!params.pitch_bend.empty()) {
        int bend_size = static_cast<int>(params.pitch_bend.size());
        for (int i = 0; i < output_samples; ++i) {
            // 서브프레임 인덱스 (0~bend_size-1)
            int fi = static_cast<int>(
                static_cast<double>(i) / output_samples * bend_size);
            fi = std::min(fi, bend_size - 1);
            double cents = params.pitch_bend[fi] * 100.0 / 100.0; // int8 단위 = 반음/100
            // 실제로 int8_t 값 1 = 100 cents = 1 반음? or 1 cent?
            // UTAU 관례: 각 byte = semitone*10 (=10cents 단위)
            // → cents = byte_val * 10
            cents = params.pitch_bend[fi] * 10.0;
            f0[i] = params.target_hz * std::pow(2.0, cents / 1200.0);
        }
    }

    // ── modulation 진동 적용 (vibrato) ─────────────────────────────────
    // modulation > 0: ±modulation×0.01 반음의 비브라토 (5.5Hz 속도)
    if (params.modulation > 0) {
        double depth_semitones = params.modulation * 0.01;
        double vib_rate_hz     = 5.5;
        for (int i = 0; i < output_samples; ++i) {
            double t   = static_cast<double>(i) / sample_rate;
            double mod = std::sin(2.0 * math::PI * vib_rate_hz * t);
            f0[i] *= std::pow(2.0, mod * depth_semitones / 12.0);
        }
    }

    return f0;
}

} // namespace resamp::synth
