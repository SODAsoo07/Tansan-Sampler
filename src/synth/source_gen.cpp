#include "source_gen.hpp"
#include "util/math_util.hpp"
#include <cmath>
#include <random>
#include <algorithm>

namespace resamp::synth {

// ── LF 글로탈 펄스: 위상 [0,1) 기반 직접 계산 ────────────────────────────
// 정수 period 기반 대비 장점: 연속 위상 → 피치 정확도 완벽, 전환 시 위상 점프 없음
static float lf_at_phase(double phase, double tension) {
    constexpr double Tp = 0.4;   // 개방(상승) 구간 비율
    constexpr double Tn = 0.16;  // 폐쇄(하강) 구간 비율
    constexpr double Te = Tp + Tn;

    double ten   = std::max(0.0, std::min(1.0, tension));
    double power = 1.0 + ten * 2.0;   // tension↑ → 더 뾰족한 펄스

    if (phase < Tp) {
        // 개방 구간: sin² (power 지수로 파형 형태 조절)
        return static_cast<float>(
            std::pow(std::sin(math::PI * phase / Tp), power));
    } else if (phase < Te) {
        // 급폐쇄 구간: 감쇠 sin (성대 충격)
        double t     = (phase - Tp) / Tn;
        double decay = std::exp(-ten * 3.0 * t);
        return static_cast<float>(-decay * std::sin(math::PI * t));
    }
    return 0.0f;  // 폐쇄 구간 (무음)
}

std::vector<float> generate_source(const std::vector<double>& f0_contour,
                                   int sample_rate,
                                   const SynthParams& params) {
    int N = static_cast<int>(f0_contour.size());
    std::vector<float> out(N, 0.0f);

    // 파라미터 변환
    float tension = (params.tension + 100) / 200.0f;   // -100~+100 → 0~1

    // B 플래그: 0=무성(순수), 100=강한 기식음
    // B=0 이 기본값 → 노이즈 없음; B>0 일수록 숨소리 증가
    float aspiration = std::max(0.0f, params.brightness / 100.0f) * 0.4f;
    // H 플래그: 배음 수 제한 (소스 LPF cutoff)
    float hf_cutoff = params.harmonics / 100.0f;  // 0→최소 배음, 1→전대역

    // 노이즈 생성기 (재현 가능한 시드)
    std::mt19937 rng(42);
    std::normal_distribution<float> nd(0.0f, 1.0f);

    // ── 연속 위상 누적 (Phase Accumulator) ──────────────────────────────
    // 정수 반올림 없이 정확한 F0 유지
    double phase_accum = 0.0;  // [0.0, 1.0)

    for (int i = 0; i < N; ++i) {
        double f0    = f0_contour[i];
        bool voiced  = (f0 > 50.0);

        if (voiced) {
            // 위상 진행 (이 샘플의 F0로 step 계산)
            phase_accum += f0 / static_cast<double>(sample_rate);
            if (phase_accum >= 1.0) phase_accum -= 1.0;

            float lf_val  = lf_at_phase(phase_accum, tension);
            float noise_s = nd(rng);
            out[i] = (1.0f - aspiration) * lf_val
                   + aspiration          * noise_s * 0.3f;
        } else {
            // 무성음: 백색 노이즈, 위상 리셋
            out[i]       = nd(rng) * 0.3f;
            phase_accum  = 0.0;
        }
    }

    // ── H 플래그: 1차 IIR LPF (배음 수 제한) ────────────────────────────
    // alpha=0 방어: 최소 0.05 보장 (완전 무음 방지)
    if (hf_cutoff < 0.98f) {
        float alpha = (hf_cutoff < 0.05f) ? 0.05f : hf_cutoff;
        float prev  = (N > 0) ? out[0] : 0.0f;  // 첫 샘플로 초기화 (DC 점프 방지)
        for (int i = 0; i < N; ++i) {
            out[i] = prev = prev + alpha * (out[i] - prev);
        }
    }

    return out;
}

} // namespace resamp::synth
