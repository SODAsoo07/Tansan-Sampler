#include "world_synth.hpp"
#include "util/math_util.hpp"

#include "world/dio.h"
#include "world/stonemask.h"
#include "world/cheaptrick.h"
#include "world/d4c.h"
#include "world/synthesis.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace resamp::synth {

// ── 분석: DIO+StoneMask → CheapTrick → D4C ────────────────────────────────
//
// 핵심 전략 (피치 변조 시 envelope modulation 아티팩트 제거):
//   - anchor F0와 실제 F0가 미세하게 어긋날 때 생기는 harmonic 잔류를 회피.
//   - 분석 F0는 DIO 후 StoneMask로 refine해 프레임 jitter와 octave 흔들림 억제.
//   - refine 결과에 대해 최소한(3점 median) 정제로 spike만 추가 억제.
//   - q1 강화로 envelope 쪽 smoothing을 늘려 modulation 아티팩트 추가 억제.
WorldAnalysis world_analyze(
    const std::vector<float>& signal,
    int                       sample_rate)
{
    WorldAnalysis w;
    w.fs           = sample_rate;
    // 2.5ms: F0/envelope 시간 해상도 (피치 벤드 jitter 감소)
    w.frame_period = 2.5;

    int x_length = static_cast<int>(signal.size());
    if (x_length < 1) return w;

    // float → double
    std::vector<double> x(x_length);
    for (int i = 0; i < x_length; ++i) x[i] = static_cast<double>(signal[i]);

    // ── 1. DIO + StoneMask: 분석 F0 추정/정제 ──────────────────────────
    DioOption dio_opt;
    InitializeDioOption(&dio_opt);
    dio_opt.frame_period = w.frame_period;
    dio_opt.f0_floor     = 71.0;
    dio_opt.f0_ceil      = 800.0;

    int f0_length = GetSamplesForDIO(sample_rate, x_length, w.frame_period);
    w.n_frames = f0_length;
    std::vector<double> dio_f0(f0_length, 0.0);
    std::vector<double> refined_f0(f0_length, 0.0);
    w.temporal_positions.assign(f0_length, 0.0);

    Dio(x.data(), x_length, sample_rate, &dio_opt,
        w.temporal_positions.data(), dio_f0.data());
    StoneMask(x.data(), x_length, sample_rate,
              w.temporal_positions.data(), dio_f0.data(), f0_length,
              refined_f0.data());

    // ── 2. 분석용 F0: StoneMask 결과 + 3점 median(유성 구간만) ────────
    //   IIR은 쓰지 않고, spike성 jitter만 억제해 envelope 추정 안정성 확보.
    auto median3 = [](double a, double b, double c) {
        if (a > b) std::swap(a, b);
        if (b > c) std::swap(b, c);
        if (a > b) std::swap(a, b);
        return b;
    };

    // StoneMask 출력 정합성 보정: 비정상 값은 dio_f0 또는 0으로 폴백
    std::vector<double> env_f0(f0_length, 0.0);
    for (int i = 0; i < f0_length; ++i) {
        double v = refined_f0[i];
        if (!std::isfinite(v) || v < 0.0 || v > 2000.0) v = dio_f0[i];
        if (!std::isfinite(v) || v < 0.0 || v > 2000.0) v = 0.0;
        env_f0[i] = v;
    }
    for (int i = 1; i + 1 < f0_length; ++i) {
        double a = env_f0[i - 1];
        double b = env_f0[i];
        double c = env_f0[i + 1];
        if (a >= 50.0 && b >= 50.0 && c >= 50.0) {
            env_f0[i] = median3(a, b, c);
        }
    }
    w.f0 = std::move(env_f0);

    // ── 3. CheapTrick: 스펙트럼 포락 (envelope smoothing 강화) ──────
    //   q1 = -0.30: 기본 -0.15 대비 envelope smoothing 2배.
    //   harmonic 잔류 추가 억제 → 합성 F0가 분석 F0와 다를 때 modulation
    //   아티팩트 추가 감소. 약한 muffling이 있을 수 있으나 명확도 손실 없음.
    CheapTrickOption ct_opt;
    InitializeCheapTrickOption(sample_rate, &ct_opt);
    ct_opt.q1       = -0.30;
    ct_opt.f0_floor = 71.0;
    ct_opt.fft_size = GetFFTSizeForCheapTrick(sample_rate, &ct_opt);
    w.fft_size = ct_opt.fft_size;

    int spec_dim = w.fft_size / 2 + 1;
    w.spectrogram.assign(f0_length, std::vector<double>(spec_dim, 0.0));
    std::vector<double*> spec_ptrs(f0_length);
    for (int i = 0; i < f0_length; ++i) spec_ptrs[i] = w.spectrogram[i].data();

    CheapTrick(x.data(), x_length, sample_rate,
               w.temporal_positions.data(), w.f0.data(), f0_length,
               &ct_opt, spec_ptrs.data());

    // ── 4. D4C: 비주기성 (DIO+StoneMask 기반 F0 사용) ─────────────────
    D4COption d4c_opt;
    InitializeD4COption(&d4c_opt);
    d4c_opt.threshold = 0.85;

    w.aperiodicity.assign(f0_length, std::vector<double>(spec_dim, 0.0));
    std::vector<double*> ap_ptrs(f0_length);
    for (int i = 0; i < f0_length; ++i) ap_ptrs[i] = w.aperiodicity[i].data();

    D4C(x.data(), x_length, sample_rate,
        w.temporal_positions.data(), w.f0.data(), f0_length,
        w.fft_size, &d4c_opt, ap_ptrs.data());

    std::cerr << "[Resamp] WORLD analyzed: n_frames=" << f0_length
              << " fft_size=" << w.fft_size << '\n';
    return w;
}

// ── 시간 매핑: 출력 시간(ms) → 소스 시간(ms) ────────────────────────────
// UTAU 자음/모음 처리:
//   - 자음 영역: velocity 비율로 시간 스케일
//   - 연결(transition) 구간: 1회 통과
//   - 안정 모음 구간: 루프
static void map_out_time_to_src(
    double out_time_ms,
    double consonant_scale,
    double consonant_src_ms,
    double consonant_tgt_ms,
    double transition_src_len_ms,
    double transition_tgt_len_ms,
    double loop_start_ms,
    double loop_len_ms,
    double& src_time_ms,
    bool&   in_vowel_loop)
{
    if (out_time_ms <= consonant_tgt_ms) {
        src_time_ms = out_time_ms / consonant_scale;
        in_vowel_loop = false;
    } else {
        double vowel_time = out_time_ms - consonant_tgt_ms;

        // 연결 구간은 한 번만 통과
        if (transition_src_len_ms > 0.0 && vowel_time <= transition_tgt_len_ms) {
            double t = vowel_time / std::max(1.0e-6, transition_tgt_len_ms);
            t = std::clamp(t, 0.0, 1.0);
            src_time_ms = consonant_src_ms + transition_src_len_ms * t;
            in_vowel_loop = false;
            return;
        }

        // 이후는 안정 모음 구간만 루프
        if (loop_len_ms > 0.0) {
            double loop_time = vowel_time - transition_tgt_len_ms;
            double wrapped = std::fmod(loop_time, loop_len_ms);
            if (wrapped < 0.0) wrapped += loop_len_ms;
            src_time_ms = loop_start_ms + wrapped;
            in_vowel_loop = true;
        } else {
            // 루프를 쓰지 않는 경우에는 tail을 시간 순서대로 계속 진행.
            // (정지 프레임 고정은 연결부 툭툭거림/메아리 유발)
            double tail_time = std::max(0.0, vowel_time - transition_tgt_len_ms);
            src_time_ms = consonant_src_ms + transition_src_len_ms + tail_time;
            in_vowel_loop = false;
        }
    }
}

// 샘플 단위 target F0에서 특정 중심 샘플 주변을 읽어 frame용 F0를 추정.
// 선형 점샘플링보다 derivative 노이즈에 강하고, IIR 없이도 "삐용" 잔류를 줄인다.
// 로그 도메인 가중 평균(= geometric mean)으로 계산해 피치 비율 보존.
static double sample_frame_f0_from_contour(
    const std::vector<double>& target_f0,
    double                     center_sample,
    int                        half_window_samples)
{
    const int n = static_cast<int>(target_f0.size());
    if (n <= 0) return 0.0;

    int c = static_cast<int>(std::round(center_sample));
    c = std::max(0, std::min(c, n - 1));

    if (half_window_samples <= 0) {
        double v = target_f0[c];
        return (v >= 50.0) ? v : 0.0;
    }

    int a = std::max(0, c - half_window_samples);
    int b = std::min(n - 1, c + half_window_samples);

    double sum_w   = 0.0;
    double sum_log = 0.0;
    for (int s = a; s <= b; ++s) {
        double v = target_f0[s];
        if (v < 50.0) continue;

        // 삼각 윈도우: 중심 가중치 최대, 가장자리 최소
        double dist = std::abs(s - c);
        double w = 1.0 - dist / (half_window_samples + 1.0);
        if (w < 0.0) w = 0.0;
        sum_w   += w;
        sum_log += w * std::log(v);
    }

    if (sum_w <= 0.0) return 0.0;
    return std::exp(sum_log / sum_w);
}

// voiced 구간에서만 적용하는 zero-phase(대칭 FIR) log-F0 smoothing.
// IIR 지연/위상왜곡 없이 프레임 간 미세 F0 jitter를 줄인다.
static void smooth_out_f0_log_zero_phase(
    std::vector<double>& f0,
    int                  radius)
{
    if (radius <= 0 || f0.empty()) return;
    std::vector<double> in = f0;
    int n = static_cast<int>(f0.size());
    for (int i = 0; i < n; ++i) {
        if (in[i] < 50.0) {
            f0[i] = 0.0;
            continue;
        }
        double sum_w = 0.0;
        double sum_l = 0.0;
        for (int k = -radius; k <= radius; ++k) {
            int j = i + k;
            if (j < 0 || j >= n) continue;
            if (in[j] < 50.0) continue;
            double w = static_cast<double>(radius + 1 - std::abs(k));
            sum_w += w;
            sum_l += w * std::log(in[j]);
        }
        if (sum_w > 0.0) f0[i] = std::exp(sum_l / sum_w);
    }
}

// 프레임 단위 F0 안정화:
// 로그 도메인 slew 제한(급격한 pitch jump 억제)
static void stabilize_out_f0(
    std::vector<double>& f0,
    double               frame_period_ms,
    double               max_cents_per_ms)
{
    if (f0.empty() || frame_period_ms <= 0.0) return;
    int n = static_cast<int>(f0.size());

    // cents/ms 기준 slew 제한 (양방향 적용으로 위상 편향 완화)
    double max_step_cents = max_cents_per_ms * frame_period_ms;
    double max_ratio = std::pow(2.0, max_step_cents / 1200.0);
    double min_ratio = 1.0 / max_ratio;

    // forward
    for (int k = 1; k < n; ++k) {
        double p = f0[k - 1];
        double c = f0[k];
        if (p >= 50.0 && c >= 50.0) {
            double r = c / p;
            if (r > max_ratio) c = p * max_ratio;
            else if (r < min_ratio) c = p * min_ratio;
            f0[k] = c;
        }
    }
    // backward
    for (int k = n - 2; k >= 0; --k) {
        double c = f0[k];
        double n1 = f0[k + 1];
        if (c >= 50.0 && n1 >= 50.0) {
            double r = c / n1;
            if (r > max_ratio) c = n1 * max_ratio;
            else if (r < min_ratio) c = n1 * min_ratio;
            f0[k] = c;
        }
    }
}

// ── 합성: 시간 매핑 + 플래그 적용 + WORLD Synthesis ─────────────────────
std::vector<float> world_render(
    const WorldAnalysis&       src,
    const std::vector<double>& target_f0_per_sample,
    const RenderParams&        params,
    const SynthParams&         sp,
    int                        output_samples)
{
    if (output_samples < 1)         return std::vector<float>();
    if (src.n_frames < 2)           return std::vector<float>(output_samples, 0.0f);
    if (src.fft_size <= 0)          return std::vector<float>(output_samples, 0.0f);

    int fs             = src.fs;
    double anal_period = src.frame_period; // 분석 프레임 주기 (2.5 ms)
    double out_total_ms = output_samples * 1000.0 / std::max(1, fs);
    // 합성 프레임 주기:
    // 변조가 큰 노트는 촘촘하게(품질 우선), 평탄/저변조 노트는 조금 넓혀 속도 개선.
    bool has_f0_mod = false;
    double frame_period = 0.5;
    int spec_dim       = src.fft_size / 2 + 1;
    double src_total_ms = src.n_frames * anal_period;
    // UTAU velocity 관례:
    // 값이 클수록 자음이 더 빠르게(짧게) 지나가야 하므로 역비율 사용.
    // v=100 -> 1.0, v=200 -> 0.5, v=50 -> 2.0
    double vel = static_cast<double>(std::max(1, params.velocity));
    double consonant_scale = std::clamp(100.0 / vel, 0.25, 4.0);
    double consonant_src_ms = std::clamp(params.consonant_ms, 0.0, src_total_ms);
    double consonant_tgt_ms = consonant_src_ms * consonant_scale;

    // CVVC 연결부 에코 방지:
    // 전체 후반부를 루프하지 않고, 연결 구간(1회 통과) + 안정 유성 구간(루프)로 분리.
    int n_frames = src.n_frames;
    int start_fi = static_cast<int>(std::round(consonant_src_ms / anal_period));
    start_fi = std::clamp(start_fi, 0, n_frames - 1);

    // 분석 F0의 유/무성 마스크를 보정해 프레임 단위 깜빡임(chatter) 억제.
    std::vector<double> src_voicing(n_frames, 0.0);
    for (int fi = 0; fi < n_frames; ++fi)
        src_voicing[fi] = (src.f0[fi] >= 55.0) ? 1.0 : 0.0;
    {
        // voiced 사이의 짧은 unvoiced gap 메우기 (<= 2프레임)
        int run_s = -1;
        for (int fi = 0; fi <= n_frames; ++fi) {
            bool voiced = (fi < n_frames) ? (src_voicing[fi] >= 0.5) : true;
            if (!voiced) {
                if (run_s < 0) run_s = fi;
            } else if (run_s >= 0) {
                int run_e = fi - 1;
                bool left_voiced  = (run_s - 1 >= 0) && (src_voicing[run_s - 1] >= 0.5);
                bool right_voiced = (fi < n_frames) && (src_voicing[fi] >= 0.5);
                int len = run_e - run_s + 1;
                if (left_voiced && right_voiced && len <= 2) {
                    for (int k = run_s; k <= run_e; ++k) src_voicing[k] = 1.0;
                }
                run_s = -1;
            }
        }
    }
    {
        // unvoiced 사이의 1프레임 voiced spike 제거
        int run_s = -1;
        for (int fi = 0; fi <= n_frames; ++fi) {
            bool voiced = (fi < n_frames) ? (src_voicing[fi] >= 0.5) : false;
            if (voiced) {
                if (run_s < 0) run_s = fi;
            } else if (run_s >= 0) {
                int run_e = fi - 1;
                bool left_unvoiced  = (run_s - 1 >= 0) && (src_voicing[run_s - 1] < 0.5);
                bool right_unvoiced = (fi < n_frames) && (src_voicing[fi] < 0.5);
                int len = run_e - run_s + 1;
                if (left_unvoiced && right_unvoiced && len <= 1) {
                    for (int k = run_s; k <= run_e; ++k) src_voicing[k] = 0.0;
                }
                run_s = -1;
            }
        }
    }

    int best_s = -1, best_e = -1;
    int cur_s = -1;
    for (int fi = start_fi; fi < n_frames; ++fi) {
        bool voiced = (src_voicing[fi] >= 0.5);
        if (voiced) {
            if (cur_s < 0) cur_s = fi;
        } else if (cur_s >= 0) {
            int cur_e = fi - 1;
            if (best_s < 0 || (cur_e - cur_s) > (best_e - best_s)) {
                best_s = cur_s;
                best_e = cur_e;
            }
            cur_s = -1;
        }
    }
    if (cur_s >= 0) {
        int cur_e = n_frames - 1;
        if (best_s < 0 || (cur_e - cur_s) > (best_e - best_s)) {
            best_s = cur_s;
            best_e = cur_e;
        }
    }

    int loop_start_fi = start_fi;
    int loop_end_fi   = std::max(start_fi + 1, n_frames - 1);
    int min_loop_frames = std::max(8, static_cast<int>(std::round(45.0 / anal_period))); // >=45ms
    if (best_s >= 0 && best_e > best_s && (best_e - best_s + 1) >= min_loop_frames) {
        int edge = std::max(1, static_cast<int>(std::round(10.0 / anal_period))); // 10ms edge trim
        loop_start_fi = std::min(best_e - 1, best_s + edge);
        loop_end_fi   = std::max(loop_start_fi + 1, best_e - edge);
    } else {
        // voiced run이 불안정하면 tail release를 제외한 보수적 루프
        int head = std::max(1, static_cast<int>(std::round(18.0 / anal_period)));
        int tail = std::max(1, static_cast<int>(std::round(20.0 / anal_period)));
        loop_start_fi = std::min(n_frames - 2, start_fi + head);
        loop_end_fi   = std::max(loop_start_fi + 1, n_frames - 1 - tail);
    }
    loop_start_fi = std::clamp(loop_start_fi, 0, n_frames - 2);
    loop_end_fi   = std::clamp(loop_end_fi, loop_start_fi + 1, n_frames - 1);

    // loop 경계를 spectral flux 기준으로 추가 트림해 CVVC 연결 잔향(메아리) 억제
    auto frame_flux = [&](int fi0, int fi1) {
        fi0 = std::clamp(fi0, 0, n_frames - 1);
        fi1 = std::clamp(fi1, 0, n_frames - 1);
        if (fi0 == fi1) return 0.0;
        const auto& a = src.spectrogram[fi0];
        const auto& b = src.spectrogram[fi1];
        double sum = 0.0;
        for (int k = 1; k < spec_dim; ++k) {
            double va = std::max(1.0e-12, a[k]);
            double vb = std::max(1.0e-12, b[k]);
            sum += std::fabs(std::log(vb) - std::log(va));
        }
        return sum / std::max(1, spec_dim - 1);
    };

    if (loop_end_fi - loop_start_fi >= 2) {
        std::vector<double> flux_vals;
        flux_vals.reserve(std::max(0, loop_end_fi - loop_start_fi));
        for (int fi = loop_start_fi; fi < loop_end_fi; ++fi) {
            flux_vals.push_back(frame_flux(fi, fi + 1));
        }
        if (!flux_vals.empty()) {
            auto mid = flux_vals.begin() + flux_vals.size() / 2;
            std::nth_element(flux_vals.begin(), mid, flux_vals.end());
            double med_flux = *mid;
            double thr_flux = std::max(0.01, med_flux * 1.55);
            int min_core = std::max(6, static_cast<int>(std::round(28.0 / anal_period))); // >=28ms
            while ((loop_end_fi - loop_start_fi + 1) > min_core &&
                   frame_flux(loop_start_fi, loop_start_fi + 1) > thr_flux) {
                ++loop_start_fi;
            }
            while ((loop_end_fi - loop_start_fi + 1) > min_core &&
                   frame_flux(loop_end_fi - 1, loop_end_fi) > thr_flux) {
                --loop_end_fi;
            }
            // 경계 추가 가드는 루프 길이가 충분할 때만 적용
            if ((loop_end_fi - loop_start_fi + 1) > (min_core + 6)) {
                ++loop_start_fi;
                --loop_end_fi;
            }
        }
    }
    loop_start_fi = std::clamp(loop_start_fi, 0, n_frames - 2);
    loop_end_fi   = std::clamp(loop_end_fi, loop_start_fi + 1, n_frames - 1);

    // 루프 경계 mismatch 최소화:
    // end->start 스펙트럼 점프를 줄여 반복 시 "툭툭" 끊기는 인상을 완화.
    if (loop_end_fi - loop_start_fi >= 2) {
        int min_core = std::max(6, static_cast<int>(std::round(28.0 / anal_period)));
        int search_w = std::max(1, static_cast<int>(std::round(8.0 / anal_period))); // ~8ms
        int best_ls = loop_start_fi;
        int best_le = loop_end_fi;
        double best_cost = 1.0e30;
        for (int ds = -search_w; ds <= search_w; ++ds) {
            int ls = std::clamp(loop_start_fi + ds, 0, n_frames - 2);
            for (int de = -search_w; de <= search_w; ++de) {
                int le = std::clamp(loop_end_fi + de, ls + 1, n_frames - 1);
                if ((le - ls + 1) < min_core) continue;
                double wrap_jump = frame_flux(le, ls);
                double edge_l = frame_flux(ls, ls + 1);
                double edge_r = frame_flux(le - 1, le);
                // wrap 불연속을 우선 최소화, 내부 급변 구간은 보조 패널티.
                double cost = wrap_jump + 0.55 * (edge_l + edge_r);
                if (cost < best_cost) {
                    best_cost = cost;
                    best_ls = ls;
                    best_le = le;
                }
            }
        }
        loop_start_fi = best_ls;
        loop_end_fi   = best_le;
    }

    double loop_start_ms = loop_start_fi * anal_period;
    double loop_end_ms   = loop_end_fi * anal_period;
    double loop_len_ms   = std::max(0.0, loop_end_ms - loop_start_ms);
    bool has_vowel_loop  = loop_len_ms > (2.0 * anal_period);
    double transition_src_len_ms = std::max(0.0, loop_start_ms - consonant_src_ms);

    // 루프가 유효하지 않으면 one-pass tail로 강제.
    // (짧은/불안정 루프의 반복이 VC/자음 포함 노트에서 툭툭 끊김을 유발)
    if (!has_vowel_loop) {
        loop_start_ms = std::clamp(consonant_src_ms, 0.0, src_total_ms);
        loop_end_ms   = src_total_ms;
        loop_len_ms   = 0.0;
        transition_src_len_ms = 0.0;
        loop_start_fi = std::clamp(static_cast<int>(std::round(loop_start_ms / anal_period)), 0, n_frames - 1);
        loop_end_fi   = loop_start_fi;
    }

    // 출력 길이가 소스의 자음 이후 길이보다 짧거나 같으면 루프가 필요 없다.
    // 이 경우 루프 경로를 완전히 끄고 one-pass로만 진행해 끊김 가능성 최소화.
    double required_after_consonant_ms = std::max(0.0, out_total_ms - consonant_tgt_ms);
    double available_after_consonant_ms = std::max(0.0, src_total_ms - consonant_src_ms);
    bool no_loop_needed = (required_after_consonant_ms <= (available_after_consonant_ms + 1.0));
    if (no_loop_needed) {
        has_vowel_loop = false;
        loop_len_ms = 0.0;
        loop_start_ms = std::clamp(consonant_src_ms, 0.0, src_total_ms);
        loop_end_ms = loop_start_ms;
        transition_src_len_ms = 0.0;
        loop_start_fi = std::clamp(static_cast<int>(std::round(loop_start_ms / anal_period)), 0, n_frames - 1);
        loop_end_fi = loop_start_fi;
    }

    // 연결부(자음→모음)의 유성 비율을 보고 길이 압축을 다르게 적용.
    int trans_a = std::clamp(static_cast<int>(std::round(consonant_src_ms / anal_period)), 0, n_frames - 1);
    int trans_b = std::clamp(static_cast<int>(std::round((consonant_src_ms + transition_src_len_ms) / anal_period)),
                             trans_a, n_frames - 1);
    double transition_voiced_ratio = 1.0;
    if (trans_b >= trans_a) {
        int vcnt = 0;
        int tcnt = trans_b - trans_a + 1;
        for (int fi = trans_a; fi <= trans_b; ++fi)
            if (src_voicing[fi] >= 0.5) ++vcnt;
        if (tcnt > 0) transition_voiced_ratio = static_cast<double>(vcnt) / tcnt;
    }

    // 연결부는 너무 짧으면 끊김/툭툭거림이 생길 수 있어
    // 과도한 단축은 피하고, 무성 연결부에서는 충분히 길게 유지.
    // Tr 플래그(transition length scale)를 곱해 사용자 제어 허용.
    double transition_tgt_len_ms = 0.0;
    if (transition_src_len_ms > 0.0) {
        double scale  = (transition_voiced_ratio < 0.45) ? 1.00 : 0.85;
        double min_ms = (transition_voiced_ratio < 0.45) ? 12.0 : 5.0;
        double max_ms = (transition_voiced_ratio < 0.45) ? 52.0 : 35.0;
        double tr_scale_local = std::clamp(sp.transition_length / 100.0, 0.35, 2.2);
        transition_tgt_len_ms = std::clamp(transition_src_len_ms * scale * tr_scale_local,
                                           min_ms * tr_scale_local, max_ms * tr_scale_local);
    }
    // 타겟 F0가 거의 평탄한 노트면 강제 평탄화 (비브라토 없는 음정 떨림 억제).
    double voiced_min = 1.0e18;
    double voiced_max = 0.0;
    double voiced_sum = 0.0;
    int voiced_cnt = 0;
    for (double v : target_f0_per_sample) {
        if (v >= 50.0) {
            voiced_min = std::min(voiced_min, v);
            voiced_max = std::max(voiced_max, v);
            voiced_sum += v;
            ++voiced_cnt;
        }
    }
    double flat_f0_hz = (voiced_cnt > 0) ? (voiced_sum / voiced_cnt) : 0.0;
    double voiced_span_cents = 0.0;
    if (voiced_cnt > 1 && voiced_min >= 1.0 && voiced_max > voiced_min) {
        voiced_span_cents = 1200.0 * std::log2(voiced_max / voiced_min);
    }
    // 실제 타겟 F0 변화량 기준으로 modulation 여부 판정.
    // (pit 배열 존재 여부만으로 판정하면 비브라토가 과도하게 눌릴 수 있음)
    has_f0_mod = (voiced_span_cents >= 8.0);
    bool flat_target_f0 = (voiced_cnt > 0 && voiced_span_cents <= 3.0);
    if (has_f0_mod)      frame_period = 0.50; // 피치 추종성 우선
    else if (flat_target_f0) frame_period = 0.80; // 평탄 노트 속도 우선
    else                 frame_period = 0.65; // 일반 노트 절충

    double seam_ms = 0.0;
    if (has_vowel_loop) {
        // loop 경계 전후에서 end->start 파라미터를 부드럽게 잇는 크로스페이드 폭.
        seam_ms = std::clamp(loop_len_ms * 0.12, 3.0, 12.0);
    }

    // 안정화 모드:
    // 음색이 노트마다 랜덤하게 꺼지는 현상을 막기 위해
    // transition voiced ratio 의존을 줄이고 보수적 상수 혼합으로 고정.
    bool use_stable_vowel_env = has_vowel_loop || flat_target_f0;
    bool has_consonant_head = (consonant_src_ms >= 1.0);
    double anchor_mix_cap = flat_target_f0 ? 0.60 : (has_f0_mod ? 0.20 : (has_consonant_head ? 0.28 : 0.46));
    if (has_consonant_head) anchor_mix_cap *= 0.75;
    if (no_loop_needed) anchor_mix_cap *= 0.80;
    // Cs: 높을수록 연결 안정성 강화(과도한 변화 억제)
    double cs_local = std::clamp((sp.consonant_stability - 50) / 50.0, -1.0, 1.0);
    if (cs_local >= 0.0) anchor_mix_cap *= (1.0 + 0.22 * cs_local);
    else                 anchor_mix_cap *= (1.0 + 0.12 * cs_local);
    if (anchor_mix_cap < 0.05) use_stable_vowel_env = false;
    std::vector<double> vowel_spec_anchor(spec_dim, 0.0);
    std::vector<double> vowel_ap_anchor(spec_dim, 0.0);
    if (use_stable_vowel_env) {
        int a = loop_start_fi;
        int b = loop_end_fi;
        int count = 0;

        // voiced 우선 평균
        for (int fi = a; fi <= b; ++fi) {
            if (fi < 0 || fi >= src.n_frames) continue;
            if (src.f0[fi] < 50.0) continue;
            for (int k = 0; k < spec_dim; ++k) {
                vowel_spec_anchor[k] += src.spectrogram[fi][k];
                vowel_ap_anchor[k]   += src.aperiodicity[fi][k];
            }
            ++count;
        }
        // voiced가 거의 없으면 전체 평균으로 폴백
        if (count == 0) {
            for (int fi = a; fi <= b; ++fi) {
                if (fi < 0 || fi >= src.n_frames) continue;
                for (int k = 0; k < spec_dim; ++k) {
                    vowel_spec_anchor[k] += src.spectrogram[fi][k];
                    vowel_ap_anchor[k]   += src.aperiodicity[fi][k];
                }
                ++count;
            }
        }
        if (count > 0) {
            double inv = 1.0 / count;
            for (int k = 0; k < spec_dim; ++k) {
                vowel_spec_anchor[k] *= inv;
                vowel_ap_anchor[k]   *= inv;
            }
        } else {
            use_stable_vowel_env = false;
        }
    }

    // 출력 프레임 수 (합성 frame_period ms 간격)
    int out_n_frames = static_cast<int>(std::ceil(output_samples * 1000.0 /
                                                  (frame_period * fs))) + 2;

    // ── Gender 플래그: 포먼트 frequency warp 비율 ────────────────────
    // formant_ratio: 타겟 포먼트 / 원본 포먼트
    //   g=-100 → ratio≈1.395 (포먼트 ↑, 여성화)
    //   g=    0 → ratio=1.0   (변경 없음)
    //   g=+100 → ratio≈0.717 (포먼트 ↓, 남성화)
    double formant_ratio = std::exp(-static_cast<double>(sp.gender) / 260.0);
    bool   do_warp       = std::fabs(formant_ratio - 1.0) > 0.005;
    double gender_norm = std::clamp(sp.gender / 100.0, -1.0, 1.0);
    double gender_eff  = (gender_norm >= 0.0) ? std::pow(gender_norm, 0.80)
                                              : -std::pow(-gender_norm, 0.80);

    // ── Brightness: 스펙트럼 기울기 (B=50 기본) ───────────────────────
    // power 도메인에서 freq에 대해 선형 dB 기울기 적용
    double brightness_tilt_db = (sp.brightness - 50) / 50.0 * 18.0;
    // Hu 방향 반전:
    // +Hu = brighter, -Hu = husky
    double hu = std::clamp(-sp.husky_tone / 100.0, -1.0, 1.0);
    double hu_eff = (hu >= 0.0) ? std::pow(hu, 0.78) : -std::pow(-hu, 0.78);
    double mo = std::clamp(sp.mouth_open / 100.0, -1.0, 1.0);
    double mo_eff = (mo >= 0.0) ? std::pow(mo, 0.78) : -std::pow(-mo, 0.78);
    double mo_pos_ctrl = std::max(0.0, mo_eff);
    double mo_neg_ctrl = std::max(0.0, -mo_eff);
    // Mo 전용 포먼트 이동:
    // +Mo는 F1/F2/F3를 위로, -Mo는 아래로 미는 방향.
    // blend는 극단값에서도 과변형을 막기 위해 상한을 둔다.
    double mo_formant_shift = +0.12 * mo_pos_ctrl - 0.20 * mo_neg_ctrl;
    double mo_formant_ratio = std::clamp(std::exp(mo_formant_shift), 0.78, 1.18);
    double mo_formant_blend = (std::fabs(mo_eff) > 0.01)
        ? std::clamp(0.08 + 0.34 * mo_pos_ctrl + 0.44 * mo_neg_ctrl, 0.0, 0.58)
        : 0.0;
    double growl_amt = std::pow(std::clamp(sp.growl / 100.0, 0.0, 1.0), 0.60);
    // Tract simulator parameters (AVOX Throat 계열 세분화)
    double vtl = std::clamp(sp.tract_length / 100.0, -1.0, 1.0);
    double vtr = std::clamp(sp.tract_resonance / 100.0, -1.0, 1.0);
    double vtw = std::clamp(sp.tract_focus / 100.0, -1.0, 1.0);
    // 체감 강도를 높이기 위해 저/중값 구간 응답을 더 키움.
    double vc_amt = std::pow(std::clamp(sp.tract_constriction / 100.0, 0.0, 1.0), 0.38);
    double vtl_raw_eff = (vtl >= 0.0)
        ? std::clamp(1.18 * std::pow(vtl, 0.52), 0.0, 1.0)
        : -std::pow(-vtl, 0.58);
    double vtl_eff = vtl_raw_eff;
    if (std::fabs(vtl_raw_eff) > 1.0e-4) {
        // Gender가 담당하는 전역 성별 이동 성분을 일부 제거해,
        // Vtl은 "성도 길이/포먼트 간격 변화" 캐릭터를 더 분명히 남긴다.
        vtl_eff = std::clamp(vtl_raw_eff - 0.54 * gender_eff * std::fabs(vtl_raw_eff), -1.0, 1.0);
    }
    double vtr_eff = (vtr >= 0.0) ? std::pow(vtr, 0.34) : -std::pow(-vtr, 0.34);
    double vtw_eff = (vtw >= 0.0) ? std::pow(vtw, 0.26) : -std::pow(-vtw, 0.28);
    double nn = std::clamp(sp.nasal_coupling / 100.0, -1.0, 1.0);
    double nn_pos_eff = std::pow(std::max(0.0, nn), 0.40);
    double nn_neg_eff = std::pow(std::max(0.0, -nn), 0.48);
    double nn_amt = std::max(nn_pos_eff, nn_neg_eff);
    // 기본 톤 캘리브레이션: 고역/잔향 과강조 완화
    double global_hi_tilt_db = -2.7;
    double global_hi_ap_trim = 0.060;

    // ── Tension: 발성 강도/이완 근사 (체감 강화) ───────────────────────
    // +값: 더 pressed/firm (주기성↑, 존재감↑, breathiness↓)
    // -값: 더 relaxed/breathy (주기성↓, 거칠기/숨소리↑, 존재감↓)
    double tension = std::clamp(sp.tension / 100.0, -1.0, 1.0);
    double tension_eff = (tension >= 0.0)
                       ? std::pow(tension, 0.70)
                       : -std::pow(-tension, 0.70);

    // ── Voice Color(c): 중고역 성분 컬러링 ───────────────────────────
    double vc = std::clamp(sp.voice_color / 100.0, -1.0, 1.0);
    double vc_eff = (vc >= 0.0) ? std::pow(vc, 0.70) : -std::pow(-vc, 0.70);
    double voice_color_db = vc_eff * 14.0; // 체감 강화를 위해 ±14dB

    // ── Breathiness(Bh): airy 질감 제어 (+추가 / -억제) ───────────────
    double bh = std::clamp(sp.breathiness / 100.0, -1.0, 1.0);
    double breathiness_eff = (bh >= 0.0) ? std::pow(bh, 0.72) : -std::pow(-bh, 0.72);

    // ── 추가 플래그 제어량 ───────────────────────────────────────────
    // Tr: transition 길이 스케일
    double tr_scale = std::clamp(sp.transition_length / 100.0, 0.35, 2.2);
    // Cs: 자음/연결 안정화 강도
    double cs = std::clamp((sp.consonant_stability - 50) / 50.0, -1.0, 1.0);
    double cs_pos = std::max(0.0, cs);
    double cs_neg = std::max(0.0, -cs);
    // At: 어택 선명도/완화
    double at = std::clamp(sp.attack / 100.0, -1.0, 1.0);
    // Rl: 릴리즈 airy 강도
    double rl = std::clamp(sp.release_air / 100.0, 0.0, 1.0);
    // Ns: 노이즈 톤 컬러
    double ns = std::clamp(sp.noise_color / 100.0, -1.0, 1.0);

    // 출력 프레임별 F0/envelope/AP 구성
    std::vector<double> out_f0(out_n_frames, 0.0);
    std::vector<std::vector<double>> out_spec(out_n_frames, std::vector<double>(spec_dim, 0.0));
    std::vector<std::vector<double>> out_ap  (out_n_frames, std::vector<double>(spec_dim, 0.0));
    // frame별 반복 할당 방지
    std::vector<double> warp_buf;
    if (do_warp) warp_buf.assign(spec_dim, 0.0);
    std::vector<double> mo_warp_buf;
    if (mo_formant_blend > 1.0e-4) mo_warp_buf.assign(spec_dim, 0.0);
    std::vector<double> tract_warp_buf;
    if (std::fabs(vtl_eff) > 0.01) tract_warp_buf.assign(spec_dim, 0.0);
    std::vector<double> tract_formant_warp_buf;
    if (std::fabs(vtr_eff) > 0.01) {
        tract_formant_warp_buf.assign(spec_dim, 0.0);
    }
    std::vector<double> hu_warp_buf;
    if (hu_eff > 0.01) hu_warp_buf.assign(spec_dim, 0.0);
    const int frame_f0_half_win =
        has_f0_mod ? std::max(1, static_cast<int>(std::round(fs * 0.0005))) : 0; // ±0.5ms

    double inv_ratio = 1.0 / formant_ratio;
    double init_voiced = 1.0;
    if (n_frames > 0) {
        int a = std::clamp(start_fi, 0, n_frames - 1);
        int b = std::clamp(a + 2, a, n_frames - 1);
        double s = 0.0;
        int c = 0;
        for (int fi = a; fi <= b; ++fi) { s += src_voicing[fi]; ++c; }
        if (c > 0) init_voiced = std::clamp(s / c, 0.0, 1.0);
    }
    double voiced_state = init_voiced; // frame 간 voicedness 상태 (AP 제어 안정화용)
    // tract articulation state (프레임 간 관성)
    double tract_c1_state = -1.0;
    double tract_c2_state = -1.0;
    double tract_c3_state = -1.0;
    double tract_q_state  = -1.0;
    auto smooth_alpha_ms = [](double dt_ms, double tau_ms) {
        tau_ms = std::max(1.0e-3, tau_ms);
        double a = 1.0 - std::exp(-dt_ms / tau_ms);
        return std::clamp(a, 0.02, 1.0);
    };

    for (int i = 0; i < out_n_frames; ++i) {
        double out_time_ms = i * frame_period;

        // 1. 시간 매핑 → 소스 분석 프레임 인덱스 (보간용 fractional)
        double src_time_ms = 0.0;
        bool in_vowel_loop = false;
        map_out_time_to_src(out_time_ms,
                            consonant_scale,
                            consonant_src_ms,
                            consonant_tgt_ms,
                            transition_src_len_ms,
                            transition_tgt_len_ms,
                            loop_start_ms,
                            loop_len_ms,
                            src_time_ms,
                            in_vowel_loop);
        src_time_ms = std::clamp(src_time_ms, 0.0, std::max(0.0, src_total_ms - 1.0e-6));
        double src_fi_d    = src_time_ms / anal_period;   // 분석 프레임 주기 사용
        int    src_fi      = static_cast<int>(src_fi_d);
        int    src_fi2     = src_fi + 1;
        if (in_vowel_loop && has_vowel_loop) {
            if (src_fi < loop_start_fi) src_fi = loop_start_fi;
            if (src_fi > loop_end_fi) src_fi = loop_end_fi;
            src_fi2 = src_fi + 1;
            if (src_fi2 > loop_end_fi) src_fi2 = loop_start_fi;
        } else {
            if (src_fi < 0)            src_fi = 0;
            if (src_fi >= src.n_frames) src_fi = src.n_frames - 1;
            src_fi2 = std::min(src_fi + 1, src.n_frames - 1);
        }
        double frac        = src_fi_d - src_fi;
        if (frac < 0.0) frac = 0.0;
        if (frac > 1.0) frac = 1.0;

        // 2. F0: 샘플 단위 컨투어를 frame 중심 주변에서 로그평균 샘플링
        //    (IIR 없이 derivative 노이즈 억제, pitch ratio 보존).
        if (flat_target_f0) {
            out_f0[i] = flat_f0_hz;
        } else {
            double sf   = out_time_ms * fs / 1000.0;
            out_f0[i] = sample_frame_f0_from_contour(target_f0_per_sample, sf, frame_f0_half_win);
        }

        // source voicedness 추정:
        // F0를 0으로 끊지 않고, 무성성은 AP 쪽에서 처리해 끊김(click) 억제.
        double sv0 = src_voicing[src_fi];
        double sv1 = src_voicing[src_fi2];
        double sv  = sv0 * (1.0 - frac) + sv1 * frac;
        double voiced;
        if (sv <= 0.02) voiced = 0.0;
        else if (sv >= 0.35) voiced = 1.0;
        else {
            double x = (sv - 0.02) / (0.35 - 0.02); // 0..1
            x = std::clamp(x, 0.0, 1.0);
            voiced = x * x * (3.0 - 2.0 * x);       // smoothstep
        }
        bool in_consonant = (out_time_ms <= consonant_tgt_ms);
        bool in_transition = (out_time_ms > consonant_tgt_ms) &&
                             (out_time_ms <= consonant_tgt_ms + transition_tgt_len_ms);
        double voiced_floor = 0.0;
        if (in_transition) {
            voiced_floor = (transition_voiced_ratio < 0.45) ? 0.20 : 0.08;
        }
        double voiced_eff = std::max(voiced, voiced_floor);
        // 프레임 단위 voiced/unvoiced 채터 억제
        double alpha = (in_consonant || in_transition) ? 0.18 : 0.30;
        alpha *= std::clamp(1.0 - 0.35 * cs_pos + 0.25 * cs_neg, 0.55, 1.35);
        voiced_state += alpha * (voiced_eff - voiced_state);
        voiced_eff = std::max(voiced_eff, voiced_state);

        // F0는 note contour를 최대한 유지.
        // (자음/무성 처리는 AP에서 담당해 click/랜덤 저음색을 줄임)
        double raw_target_f0 = out_f0[i];
        out_f0[i] = (raw_target_f0 >= 50.0) ? raw_target_f0 : 0.0;

        // 3. envelope/AP 시간 보간 (linear)
        const auto& s1 = src.spectrogram[src_fi];
        const auto& s2 = src.spectrogram[src_fi2];
        const auto& a1 = src.aperiodicity[src_fi];
        const auto& a2 = src.aperiodicity[src_fi2];
        for (int k = 0; k < spec_dim; ++k) {
            out_spec[i][k] = s1[k] * (1.0 - frac) + s2[k] * frac;
            out_ap[i][k]   = a1[k] * (1.0 - frac) + a2[k] * frac;
        }

        // 루프 경계 seam crossfade:
        // end->start에서 파라미터가 급점프하지 않도록 boundary 인접 구간을 양방향 블렌드.
        if (in_vowel_loop && has_vowel_loop && seam_ms > 0.5 && loop_len_ms > (2.0 * seam_ms + anal_period)) {
            double loop_time = out_time_ms - consonant_tgt_ms - transition_tgt_len_ms;
            double wrapped = std::fmod(loop_time, loop_len_ms);
            if (wrapped < 0.0) wrapped += loop_len_ms;

            double w_wrap = 0.0;
            double alt_src_time_ms = src_time_ms;
            if (wrapped < seam_ms) {
                w_wrap = 1.0 - std::clamp(wrapped / seam_ms, 0.0, 1.0);
                alt_src_time_ms = loop_end_ms - (seam_ms - wrapped);
            } else if (wrapped > (loop_len_ms - seam_ms)) {
                double d = loop_len_ms - wrapped;
                w_wrap = 1.0 - std::clamp(d / seam_ms, 0.0, 1.0);
                alt_src_time_ms = loop_start_ms + (seam_ms - d);
            }

            if (w_wrap > 1.0e-4) {
                alt_src_time_ms = std::clamp(alt_src_time_ms, 0.0, std::max(0.0, src_total_ms - 1.0e-6));
                double afid = alt_src_time_ms / anal_period;
                int af0 = static_cast<int>(afid);
                int af1 = af0 + 1;
                if (af0 < loop_start_fi) af0 = loop_start_fi;
                if (af0 > loop_end_fi) af0 = loop_end_fi;
                af1 = af0 + 1;
                if (af1 > loop_end_fi) af1 = loop_start_fi;
                double at = std::clamp(afid - af0, 0.0, 1.0);
                const auto& as1 = src.spectrogram[af0];
                const auto& as2 = src.spectrogram[af1];
                const auto& aa1 = src.aperiodicity[af0];
                const auto& aa2 = src.aperiodicity[af1];
                for (int k = 0; k < spec_dim; ++k) {
                    double sp_alt = as1[k] * (1.0 - at) + as2[k] * at;
                    double ap_alt = aa1[k] * (1.0 - at) + aa2[k] * at;
                    out_spec[i][k] = out_spec[i][k] * (1.0 - w_wrap) + sp_alt * w_wrap;
                    out_ap[i][k]   = out_ap[i][k]   * (1.0 - w_wrap) + ap_alt * w_wrap;
                }
            }
        }

        // 연결부/자음의 무성성은 AP 상승으로 처리해 파형 불연속을 줄인다.
        {
            double uv_boost = 0.0;
            if (in_consonant || in_transition) {
                double unvoiced = 1.0 - voiced_eff;
                double base_boost = in_consonant ? 0.15 : 0.06;
                uv_boost = base_boost * (0.20 + 0.80 * unvoiced * unvoiced);
                if (transition_voiced_ratio < 0.40 && in_transition) uv_boost += 0.03;
                uv_boost *= std::clamp(1.0 - 0.55 * cs_pos + 0.35 * cs_neg, 0.45, 1.65);
            }
            uv_boost = std::clamp(uv_boost, 0.0, 0.18);
            if (uv_boost > 0.01) {
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                    double shaped = uv_boost * (0.62 + 0.38 * fn); // 고역 쪽 조금 더
                    out_ap[i][k] = std::clamp(out_ap[i][k] + shaped, 0.0, 1.0);
                }
            }
        }

        // 모음 루프에서는 anchor envelope/AP로 고정해 주기성 modulation 억제.
        // 단, 자음→모음 진입 초반에는 짧게 블렌딩해 이음새를 완화.
        bool apply_anchor_env = use_stable_vowel_env &&
                                (in_vowel_loop ||
                                 (flat_target_f0 &&
                                  out_time_ms >= (consonant_tgt_ms + transition_tgt_len_ms)));
        if (apply_anchor_env) {
            double w_anchor = 1.0;
            double vowel_elapsed_ms = out_time_ms - consonant_tgt_ms;
            if (!flat_target_f0 && vowel_elapsed_ms < seam_ms) {
                double u = std::clamp(vowel_elapsed_ms / std::max(0.1, seam_ms), 0.0, 1.0);
                // smoothstep
                w_anchor = u * u * (3.0 - 2.0 * u);
            }
            w_anchor *= anchor_mix_cap;
            for (int k = 0; k < spec_dim; ++k) {
                out_spec[i][k] = out_spec[i][k] * (1.0 - w_anchor) + vowel_spec_anchor[k] * w_anchor;
                out_ap[i][k]   = out_ap[i][k]   * (1.0 - w_anchor) + vowel_ap_anchor[k]   * w_anchor;
            }
        }

        // 4. Gender warp: envelope frequency 축 리샘플링
        //    target_freq = src_freq * formant_ratio
        //    → src_k = k / formant_ratio
        if (do_warp) {
            for (int k = 0; k < spec_dim; ++k) {
                double src_k = k * inv_ratio;
                if (src_k < 0.0)              src_k = 0.0;
                if (src_k > spec_dim - 1)     src_k = spec_dim - 1;
                int    sk  = static_cast<int>(src_k);
                int    sk2 = std::min(sk + 1, spec_dim - 1);
                double f   = src_k - sk;
                warp_buf[k]  = out_spec[i][sk] * (1.0 - f) + out_spec[i][sk2] * f;
            }
            for (int k = 0; k < spec_dim; ++k) out_spec[i][k] = warp_buf[k];
        }

        // 5. Brightness 기울기 (power 도메인에서 dB 선형)
        if (std::fabs(brightness_tilt_db) > 0.5) {
            for (int k = 0; k < spec_dim; ++k) {
                double freq_norm = static_cast<double>(k) / std::max(1, spec_dim - 1);
                double tilt_db   = brightness_tilt_db * (freq_norm - 0.5);
                double gain_pow  = std::pow(10.0, tilt_db / 10.0); // power
                out_spec[i][k] *= gain_pow;
            }
        }

        // 5.3. Mouth Open(Mo): 입 열림(+)/입 닫힘(-) 톤 이동
        // AP는 유지하고 스펙트럼만 이동한 뒤 에너지 정규화.
        if (std::fabs(mo_eff) > 0.01) {
            // (A) 포먼트 중심 이동(주파수 축 워핑)
            // k -> k / ratio 로 샘플링하면 ratio>1일 때 포먼트가 위로 이동.
            if (!mo_warp_buf.empty()) {
                double inv_mo_ratio = 1.0 / mo_formant_ratio;
                for (int k = 0; k < spec_dim; ++k) {
                    double src_k = k * inv_mo_ratio;
                    src_k = std::clamp(src_k, 0.0, static_cast<double>(spec_dim - 1));
                    int sk = static_cast<int>(src_k);
                    int sk2 = std::min(sk + 1, spec_dim - 1);
                    double f = src_k - sk;
                    mo_warp_buf[k] = out_spec[i][sk] * (1.0 - f) + out_spec[i][sk2] * f;
                }
                for (int k = 0; k < spec_dim; ++k) {
                    out_spec[i][k] = out_spec[i][k] * (1.0 - mo_formant_blend)
                                   + mo_warp_buf[k] * mo_formant_blend;
                }
            }

            // (B) 밴드별 기울기: +Mo(개방) / -Mo(닫힘) 캐릭터 분리
            double e0 = 0.0;
            double e1 = 0.0;
            double mo_pos = std::max(0.0, mo_eff);
            double mo_neg = std::max(0.0, -mo_eff);
            for (int k = 0; k < spec_dim; ++k) {
                double p0 = std::max(0.0, out_spec[i][k]);
                e0 += p0;

                double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                double hz = fn * (fs * 0.5);
                double x_f1 = std::log2((hz + 120.0) / 780.0);
                double f1 = std::exp(-0.5 * (x_f1 * x_f1) / (0.72 * 0.72));
                double x_f2 = std::log2((hz + 120.0) / 1650.0);
                double f2 = std::exp(-0.5 * (x_f2 * x_f2) / (0.82 * 0.82));
                double x_f3 = std::log2((hz + 120.0) / 2850.0);
                double f3 = std::exp(-0.5 * (x_f3 * x_f3) / (0.88 * 0.88));
                double x_lip = std::log2((hz + 120.0) / 4300.0);
                double lip = std::exp(-0.5 * (x_lip * x_lip) / (0.92 * 0.92));
                double x_mud = std::log2((hz + 120.0) / 360.0);
                double mud = std::exp(-0.5 * (x_mud * x_mud) / (0.95 * 0.95));
                double x_box = std::log2((hz + 120.0) / 1200.0);
                double box = std::exp(-0.5 * (x_box * x_box) / (0.85 * 0.85));
                double x_mumble = std::log2((hz + 120.0) / 2350.0);
                double mumble_notch = std::exp(-0.5 * (x_mumble * x_mumble) / (0.60 * 0.60));
                double x_muffle_hi = std::log2((hz + 120.0) / 4100.0);
                double muffle_hi = std::exp(-0.5 * (x_muffle_hi * x_muffle_hi) / (0.78 * 0.78));
                // +Mo: 개방감/선명도는 유지하되 이전보다 약하게
                // -Mo: 단순 감쇠보다 딕션 대역(2~5k)을 더 눌러 웅얼거리는 캐릭터를 만든다.
                double shape_open = (0.72 * f1 + 0.98 * f2 + 1.06 * f3 + 0.80 * lip
                                   - 0.30 * mud - 0.12 * box);
                double shape_close = (0.90 * mud + 0.74 * box + 0.58 * f1
                                    - 1.24 * f2 - 1.62 * f3 - 1.48 * lip
                                    - 0.90 * mumble_notch - 0.72 * muffle_hi);
                double db = mo_pos * 8.2 * shape_open + mo_neg * 9.6 * shape_close;
                db = std::clamp(db, -13.0, 13.0);
                double gain_pow = std::pow(10.0, db / 10.0);
                out_spec[i][k] = p0 * gain_pow;
                e1 += out_spec[i][k];
            }
            if (e0 > 1.0e-12 && e1 > 1.0e-12) {
                double norm = std::clamp(e0 / e1, 0.55, 1.90);
                for (int k = 0; k < spec_dim; ++k) out_spec[i][k] *= norm;
            }
        }

        // 5.4. Tract Simulator Layer:
        // Vtl/Vtr/Vtw/Vc/Nn + Mo를 결합해 성도/공명 변형을 세분화.
        if (std::fabs(vtl_eff) > 0.01 ||
            std::fabs(vtr_eff) > 0.01 ||
            std::fabs(vtw_eff) > 0.01 ||
            vc_amt > 0.01 || nn_amt > 0.01 || std::fabs(mo_eff) > 0.01) {
            auto smoothstep01 = [](double x) {
                x = std::clamp(x, 0.0, 1.0);
                return x * x * (3.0 - 2.0 * x);
            };

            double voiced_gate = 0.18 + 0.82 * voiced_eff;
            double region_gate = in_consonant ? 0.30 : (in_transition ? 0.58 : 1.00);
            double vowel_enter_ms = consonant_tgt_ms + transition_tgt_len_ms;
            double ramp_in = smoothstep01((out_time_ms - vowel_enter_ms + 4.0) / 24.0);
            double ramp_out = smoothstep01((out_total_ms - out_time_ms) / 26.0);
            double time_gate = std::clamp(ramp_in * ramp_out, 0.18, 1.0);

            double vtl_drive_base = std::pow(std::clamp(std::fabs(vtl_eff), 0.0, 1.0), 0.72);
            double vtl_drive = vtl_drive_base * voiced_gate * region_gate * time_gate;
            double vtr_drive = std::pow(std::clamp(std::fabs(vtr_eff), 0.0, 1.0), 0.42);
            double vtw_drive = std::pow(std::clamp(std::fabs(vtw_eff), 0.0, 1.0), 0.34);
            double vc_drive = std::pow(std::clamp(vc_amt, 0.0, 1.0), 0.50);
            double nn_drive = std::pow(std::clamp(nn_amt, 0.0, 1.0), 0.50);
            double mo_drive = std::pow(std::clamp(std::fabs(mo_eff), 0.0, 1.0), 0.92);
            double vtr_floor = 0.20 + 0.26 * std::pow(std::clamp(std::fabs(vtr_eff), 0.0, 1.0), 0.70);
            double vc_floor = 0.26 + 0.24 * std::pow(std::clamp(vc_amt, 0.0, 1.0), 0.72);
            double vtr_gate = std::clamp((0.34 + 0.66 * voiced_eff) *
                                         (in_consonant ? 0.56 : (in_transition ? 0.84 : 1.00)) *
                                         (0.56 + 0.44 * time_gate), vtr_floor, 1.00);
            double vc_gate = std::clamp((0.44 + 0.56 * voiced_eff) *
                                        (in_consonant ? 0.72 : (in_transition ? 0.90 : 1.00)) *
                                        (0.62 + 0.38 * time_gate), vc_floor, 1.00);
            vtr_drive *= vtr_gate;
            vc_drive *= vc_gate;

            // 독립 강도 스케일:
            // 공통 게이트 대신 파라미터별 강도를 따로 적용해 캐릭터 분리를 확보.
            constexpr double k_vtr = 5.85;
            constexpr double k_vtw = 5.35;
            constexpr double k_vc  = 4.75;
            constexpr double k_nn  = 3.95;
            constexpr double k_mo  = 1.36;

            double mix_vtr = ((std::fabs(vtr_eff) > 0.01) ? 0.18 : 0.0) + 0.92 * vtr_drive;
            double mix_vtw = ((std::fabs(vtw_eff) > 0.01) ? 0.17 : 0.0) + 0.96 * vtw_drive;
            double mix_vc  = (vc_drive > 0.01 ? 0.14 : 0.0) + 0.86 * vc_drive;
            double mix_nn  = (nn_drive > 0.01 ? 0.12 : 0.0) + 0.72 * nn_drive;
            double mix_mo  = (mo_drive > 0.01 ? 0.05 : 0.0) + 0.34 * mo_drive;
            double tract_mix_boost = 1.14;
            mix_vtr *= (1.04 + 0.24 * vtr_drive);
            mix_vtw *= (1.10 + 0.42 * vtw_drive);
            mix_vc  *= (1.00 + 0.20 * vc_drive);
            mix_nn  *= (1.04 + 0.32 * nn_drive);
            mix_vtr *= tract_mix_boost;
            mix_vtw *= tract_mix_boost;
            mix_vc  *= tract_mix_boost;
            mix_nn  *= tract_mix_boost;
            mix_vtr = std::clamp(mix_vtr, 0.0, 1.00);
            mix_vtw = std::clamp(mix_vtw, 0.0, 1.00);
            mix_vc  = std::clamp(mix_vc,  0.0, 0.97);
            mix_nn  = std::clamp(mix_nn,  0.0, 0.95);
            mix_mo  = std::clamp(mix_mo,  0.0, 0.48);

            double tract_db_boost = 1.18;
            double boost_vtr = tract_db_boost * (1.30 + 0.58 * vtr_drive);
            double boost_vtw = tract_db_boost * (1.56 + 0.96 * vtw_drive);
            double boost_vc  = tract_db_boost * (1.14 + 0.34 * vc_drive);
            double boost_nn  = tract_db_boost * (1.18 + 0.56 * nn_drive);

            auto sat_db = [](double x, double lim) {
                double l = std::max(1.0e-6, lim);
                return l * std::tanh(x / l);
            };
            auto apply_module = [](double p, double db, double mix, double depth, double gmin, double gmax) {
                double g = std::pow(10.0, (depth * db) / 10.0);
                g = std::clamp(g, gmin, gmax);
                return p * ((1.0 - mix) + mix * g);
            };
            if (std::fabs(vtl_eff) > 0.01 && !tract_warp_buf.empty()) {
                // Vtl: 비균일 성도 길이 워프
                // - 저역(F1 근방)은 덜 움직이고, 중고역(F2/F3)은 더 이동
                // - 자음/무성 구간에서는 게이트로 약화
                double vtl_pos = std::max(0.0, vtl_eff);
                double vtl_strength = vtl_eff * (0.58 + 1.18 * vtl_drive + 0.20 * vtl_pos);
                double vtl_blend = std::clamp(0.32 + 0.92 * vtl_drive + 0.08 * vtl_pos, 0.12, 0.98);
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                    double band_weight = 0.28 + 0.94 * fn + 0.32 * fn * fn;
                    double local_ratio = std::clamp(std::exp(-(vtl_strength * band_weight) / 2.10), 0.68, 1.52);
                    double src_k = k * (1.0 / local_ratio);
                    src_k = std::clamp(src_k, 0.0, static_cast<double>(spec_dim - 1));
                    int sk = static_cast<int>(src_k);
                    int sk2 = std::min(sk + 1, spec_dim - 1);
                    double f = src_k - sk;
                    tract_warp_buf[k] = out_spec[i][sk] * (1.0 - f) + out_spec[i][sk2] * f;
                }
                for (int k = 0; k < spec_dim; ++k) {
                    out_spec[i][k] = out_spec[i][k] * (1.0 - vtl_blend) + tract_warp_buf[k] * vtl_blend;
                }
            }

            // Vtr 전용 포먼트 워프:
            // 파라미터 캐릭터 분리를 위해 "포먼트 위치 이동"은 Vtr이 전담한다.
            if (!tract_formant_warp_buf.empty()) {
                double tract_formant_gate = std::clamp(
                    (0.28 + 0.72 * voiced_eff) *
                    (in_consonant ? 0.44 : (in_transition ? 0.76 : 1.0)) *
                    (0.52 + 0.48 * time_gate),
                    0.20, 1.0);
                double vtr_pos = std::max(0.0, vtr_eff);
                double vtr_neg = std::max(0.0, -vtr_eff);
                double vtr_shift_low = ((0.34 * vtr_pos) - (0.78 * vtr_neg)) * (0.40 + 0.60 * vtr_drive);
                double vtr_shift_high = ((1.08 * vtr_pos) - (0.52 * vtr_neg)) * (0.44 + 0.56 * vtr_drive);

                double shift_low  = (0.96 * vtr_shift_low) * tract_formant_gate;
                double shift_high = (1.08 * vtr_shift_high) * tract_formant_gate;
                double ratio_low  = std::clamp(std::exp(shift_low * 0.21), 0.78, 1.34);
                double ratio_high = std::clamp(std::exp(shift_high * 0.30), 0.66, 1.58);
                double tract_formant_blend = std::clamp(
                    (0.14 + 0.72 * std::fabs(vtr_eff)) * tract_formant_gate,
                    0.0, 0.90);
                if ((std::fabs(ratio_low - 1.0) > 0.003 || std::fabs(ratio_high - 1.0) > 0.003) &&
                    tract_formant_blend > 0.01) {
                    for (int k = 0; k < spec_dim; ++k) {
                        double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                        double u = std::clamp((fn - 0.16) / 0.70, 0.0, 1.0);
                        u = u * u * (3.0 - 2.0 * u);
                        double local_ratio = ratio_low * (1.0 - u) + ratio_high * u;
                        double src_k = k * (1.0 / local_ratio);
                        src_k = std::clamp(src_k, 0.0, static_cast<double>(spec_dim - 1));
                        int sk = static_cast<int>(src_k);
                        int sk2 = std::min(sk + 1, spec_dim - 1);
                        double f = src_k - sk;
                        tract_formant_warp_buf[k] = out_spec[i][sk] * (1.0 - f) + out_spec[i][sk2] * f;
                    }
                    for (int k = 0; k < spec_dim; ++k) {
                        out_spec[i][k] = out_spec[i][k] * (1.0 - tract_formant_blend)
                                       + tract_formant_warp_buf[k] * tract_formant_blend;
                    }
                }
            }

            double e0 = 0.0;
            double e1 = 0.0;
            double vtr_pos = std::max(0.0, vtr_eff);
            double vtr_neg = std::max(0.0, -vtr_eff);
            double mo_tr = mo_eff;
            double mo_open = std::max(0.0, mo_tr);
            double mo_close = std::max(0.0, -mo_tr);

            // 현실감 강화를 위한 articulation inertia:
            // - formant bandwidth(Q)와 center를 프레임 간 관성으로 이동
            // - 물리적으로 과도한 포먼트 겹침을 spacing 제약으로 방지
            double focus_sigma_target =
                std::clamp(0.90 - 1.20 * (vtw_eff * (0.24 + 0.76 * vtw_drive)), 0.16, 2.20);
            if (tract_q_state < 0.0) tract_q_state = focus_sigma_target;
            double focus_tau_ms = in_consonant ? 11.0 : (in_transition ? 15.0 : 22.0);
            focus_tau_ms *= std::clamp(1.10 - 0.35 * voiced_eff, 0.80, 1.30);
            double a_q = smooth_alpha_ms(frame_period, focus_tau_ms);
            tract_q_state += a_q * (focus_sigma_target - tract_q_state);
            double focus_sigma = std::clamp(tract_q_state, 0.16, 2.20);

            double vtr_form = (1.10 * vtr_pos - 0.82 * vtr_neg) * (0.92 + 0.64 * vtr_drive);
            double c1_target = std::clamp(
                720.0 + 2320.0 * vtr_form + 220.0 * mo_open - 560.0 * mo_close,
                150.0, 3000.0);
            double c2_target = std::clamp(
                1580.0 + 3480.0 * vtr_form + 200.0 * mo_open - 760.0 * mo_close,
                380.0, 7600.0);
            double c3_target = std::clamp(
                2820.0 + 4420.0 * vtr_form - 700.0 * mo_close,
                680.0, 11200.0);

            double min12 = std::clamp(560.0 + 280.0 * (1.0 - voiced_eff), 440.0, 980.0);
            double min23 = std::clamp(760.0 + 320.0 * (1.0 - voiced_eff), 620.0, 1450.0);
            if (c2_target < c1_target + min12) c2_target = c1_target + min12;
            if (c3_target < c2_target + min23) c3_target = c2_target + min23;
            if (c3_target > 11200.0) {
                double over = c3_target - 11200.0;
                c3_target = 11200.0;
                c2_target -= 0.58 * over;
                c1_target -= 0.32 * over;
            }
            c1_target = std::clamp(c1_target, 150.0, 3000.0);
            c2_target = std::clamp(c2_target, std::max(380.0, c1_target + min12), 7600.0);
            c3_target = std::clamp(c3_target, std::max(680.0, c2_target + min23), 11200.0);

            if (tract_c1_state < 0.0) {
                tract_c1_state = c1_target;
                tract_c2_state = c2_target;
                tract_c3_state = c3_target;
            }
            double tract_tau_ms = in_consonant ? 9.0 : (in_transition ? 12.0 : 18.0);
            if (has_f0_mod) tract_tau_ms *= 0.78;
            tract_tau_ms *= std::clamp(1.08 - 0.28 * voiced_eff, 0.82, 1.22);
            double a_form = smooth_alpha_ms(frame_period, tract_tau_ms);
            tract_c1_state += a_form * (c1_target - tract_c1_state);
            tract_c2_state += a_form * (c2_target - tract_c2_state);
            tract_c3_state += a_form * (c3_target - tract_c3_state);

            double c1 = tract_c1_state;
            double c2 = tract_c2_state;
            double c3 = tract_c3_state;
            if (c2 < c1 + min12) c2 = c1 + min12;
            if (c3 < c2 + min23) c3 = c2 + min23;
            c1 = std::clamp(c1, 150.0, 3000.0);
            c2 = std::clamp(c2, std::max(380.0, c1 + min12), 7600.0);
            c3 = std::clamp(c3, std::max(680.0, c2 + min23), 11200.0);

            for (int k = 0; k < spec_dim; ++k) {
                double p0 = std::max(0.0, out_spec[i][k]);
                e0 += p0;

                double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                double hz = fn * (fs * 0.5);
                double x1 = std::log2((hz + 120.0) / c1);
                double x2 = std::log2((hz + 120.0) / c2);
                double x3 = std::log2((hz + 120.0) / c3);
                double f1 = std::exp(-0.5 * (x1 * x1) / (focus_sigma * focus_sigma));
                double f2 = std::exp(-0.5 * (x2 * x2) / (focus_sigma * focus_sigma));
                double f3 = std::exp(-0.5 * (x3 * x3) / (focus_sigma * focus_sigma));

                // 상대 포먼트 좌표계:
                // 고정 Hz 중심 대신 c1/c2/c3 기반으로 마스크 중심을 움직여
                // 모음이 달라도 캐릭터 일관성을 유지한다.
                double constr_low_c = std::clamp(0.58 * c2, 700.0, 1900.0);
                double constr_mid_c = std::clamp(0.92 * c2, 1100.0, 2700.0);
                double constr_hi_c  = std::clamp(0.66 * c3, 1800.0, 4600.0);
                double constr_top_c = std::clamp(0.94 * c3, 2600.0, 7600.0);
                double constr_low_bw = std::clamp(0.44 * constr_low_c, 360.0, 820.0);
                double constr_mid_bw = std::clamp(0.36 * constr_mid_c, 320.0, 760.0);
                double constr_hi_bw  = std::clamp(0.30 * constr_hi_c, 420.0, 1100.0);
                double constr_top_bw = std::clamp(0.28 * constr_top_c, 520.0, 1450.0);
                double constr_hi = std::exp(-0.5 * std::pow((hz - constr_hi_c) / constr_hi_bw, 2.0));
                double constr_top = std::exp(-0.5 * std::pow((hz - constr_top_c) / constr_top_bw, 2.0));
                double constr_mid = std::exp(-0.5 * std::pow((hz - constr_mid_c) / constr_mid_bw, 2.0));
                double constr_low = std::exp(-0.5 * std::pow((hz - constr_low_c) / constr_low_bw, 2.0));

                // 비성 공명/반공명 (상대 포먼트 좌표계)
                double nasal_low_c = std::clamp(0.34 * c2, 250.0, 650.0);
                double nasal_form_c = std::clamp(0.62 * c2, 700.0, 1550.0);
                double nasal_high_c = std::clamp(0.86 * c3, 1800.0, 3600.0);
                double nasal_notch1_c = std::clamp(0.30 * c2, 420.0, 900.0);
                double nasal_notch2_c = std::clamp(0.78 * c3, 1700.0, 3400.0);
                double nasal_bridge_c = std::clamp(0.52 * c3, 1100.0, 2400.0);
                double nasal_low = std::exp(-0.5 * std::pow((hz - nasal_low_c) / std::clamp(0.45 * nasal_low_c, 140.0, 260.0), 2.0));
                double nasal_form = std::exp(-0.5 * std::pow((hz - nasal_form_c) / std::clamp(0.34 * nasal_form_c, 220.0, 460.0), 2.0));
                double nasal_high = std::exp(-0.5 * std::pow((hz - nasal_high_c) / std::clamp(0.30 * nasal_high_c, 420.0, 920.0), 2.0));
                double nasal_notch1 = std::exp(-0.5 * std::pow((hz - nasal_notch1_c) / std::clamp(0.34 * nasal_notch1_c, 170.0, 330.0), 2.0));
                double nasal_notch2 = std::exp(-0.5 * std::pow((hz - nasal_notch2_c) / std::clamp(0.30 * nasal_notch2_c, 400.0, 880.0), 2.0));
                double nasal_bridge = std::exp(-0.5 * std::pow((hz - nasal_bridge_c) / std::clamp(0.30 * nasal_bridge_c, 280.0, 620.0), 2.0));

                double vtw_notch_c = std::clamp(0.78 * c3, 2000.0, 5200.0);
                double vtw_spread_c = std::clamp(0.74 * c2, 900.0, 2600.0);
                double vtw_notch = std::exp(-0.5 * std::pow((hz - vtw_notch_c) / std::clamp(0.32 * vtw_notch_c, 760.0, 1400.0), 2.0));
                double vtw_spread = std::exp(-0.5 * std::pow((hz - vtw_spread_c) / std::clamp(0.46 * vtw_spread_c, 620.0, 1200.0), 2.0));
                double high = std::max(0.0, fn - 0.52);

                // Mo를 tract layer에 추가 반영 (기존 Mo 톤 이동과 병행)
                double db_mo = k_mo * (mo_open * (2.6 * f1 + 4.2 * f2 + 5.2 * f3 + 3.2 * high - 1.4 * constr_low)
                              + mo_close * (4.8 * constr_low + 3.8 * f1 - 7.8 * f2 - 9.8 * f3 - 10.4 * high
                                          - 2.2 * constr_top - 1.0 * constr_hi));
                db_mo = sat_db(db_mo, 11.2);
                // Vtr: +/- 방향을 분리해 서로 다른 성도 이동 캐릭터를 만든다.
                double vtr_p1 = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / std::max(180.0, c1 * 0.98)) / 0.88, 2.0));
                double vtr_p2 = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / std::max(260.0, c2 * 1.02)) / 0.68, 2.0));
                double vtr_p3 = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / std::max(420.0, c3 * 1.02)) / 0.72, 2.0));
                double vtr_body = std::exp(-0.5 * std::pow((hz - 620.0) / 360.0, 2.0));
                double vtr_hi_guard = std::exp(-0.5 * std::pow((hz - 6400.0) / 1500.0, 2.0));
                double vtr_front_shape = 1.28 * vtr_p2 + 1.64 * vtr_p3 - 0.86 * vtr_p1 - 0.30 * vtr_body - 0.34 * vtr_hi_guard;
                double vtr_back_shape = 1.18 * vtr_p1 + 0.62 * vtr_body - 1.08 * vtr_p2 - 1.26 * vtr_p3 + 0.22 * vtr_hi_guard;
                double vtr_shape = vtr_pos * vtr_front_shape + vtr_neg * vtr_back_shape;
                double db_vtr = k_vtr * boost_vtr * vtr_drive * (17.0 * vtr_shape);
                db_vtr = sat_db(db_vtr, 26.0);
                // Vtw: 공명 폭/포커스 (폭 확장/집중)
                double vtw_pos = std::max(0.0, vtw_eff);
                double vtw_neg = std::max(0.0, -vtw_eff);
                double db_vtw = k_vtw * boost_vtw * vtw_drive * (
                    vtw_pos * (26.8 * (1.86 * (f2 + f3) - 2.34 * vtw_notch - 0.62 * vtw_spread + 0.70 * high)) +
                    vtw_neg * (-21.4 * (1.42 * (f2 + f3) - 0.74 * vtw_notch - 0.92 * vtw_spread + 0.18 * high))
                );
                db_vtw = sat_db(db_vtw, 29.0);
                // Vc: 협착 (epilaryngeal twang + antiresonance)
                double vc_center = std::clamp(0.60 * c2 + 0.28 * c3 + 180.0, 1200.0, 5200.0);
                double vc_anti_center = std::clamp(0.56 * c2 + 180.0, 640.0, 2600.0);
                double vc_core_bw = std::clamp(0.72 - 0.22 * vc_drive, 0.40, 0.72);
                double vc_anti_bw = std::clamp(0.86 - 0.18 * vc_drive, 0.50, 0.86);
                double vc_core = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / vc_center) / vc_core_bw, 2.0));
                double vc_anti = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / vc_anti_center) / vc_anti_bw, 2.0));
                double vc_edge_c = std::clamp(0.96 * c3, 3000.0, 9000.0);
                double vc_box_c = std::clamp(0.72 * c2, 900.0, 2400.0);
                double vc_sizzle_c = std::clamp(1.28 * c3, 5200.0, 11000.0);
                double vc_edge = std::exp(-0.5 * std::pow((hz - vc_edge_c) / std::clamp(0.24 * vc_edge_c, 700.0, 1500.0), 2.0));
                double vc_box = std::exp(-0.5 * std::pow((hz - vc_box_c) / std::clamp(0.40 * vc_box_c, 460.0, 920.0), 2.0));
                double vc_sizzle_guard = std::exp(-0.5 * std::pow((hz - vc_sizzle_c) / std::clamp(0.20 * vc_sizzle_c, 980.0, 1900.0), 2.0));
                double vc_shape = 1.70 * vc_core + 1.02 * constr_hi + 0.86 * vc_edge + 0.50 * constr_top
                                - 1.34 * vc_anti - 0.72 * constr_mid - 0.44 * constr_low - 0.40 * vc_box - 0.52 * vc_sizzle_guard;
                double harmonic_gate = std::clamp(0.52 + 0.48 * (1.0 - out_ap[i][k]), 0.40, 1.0);
                double db_vc = k_vc * boost_vc * vc_drive * harmonic_gate * (13.0 * vc_shape);
                db_vc = sat_db(db_vc, 23.0);
                // Nn: 비성 결합 (nasal formant + anti-formant)
                double nasal_add_shape =
                    19.6 * nasal_low + 18.8 * nasal_form + 7.2 * nasal_high + 6.0 * nasal_bridge
                    - 20.0 * nasal_notch1 - 17.8 * nasal_notch2 - 3.2 * constr_hi;
                double nasal_reduce_shape =
                    -8.8 * nasal_low - 10.6 * nasal_form - 4.2 * nasal_high - 3.0 * nasal_bridge
                    + 6.6 * nasal_notch1 + 5.4 * nasal_notch2 + 2.8 * f2 + 2.1 * f3;
                double db_nn = k_nn * boost_nn *
                    (nn_pos_eff * nasal_add_shape + nn_neg_eff * nasal_reduce_shape);
                db_nn = sat_db(db_nn, 25.0);

                // Source(성대원) 레이어:
                // 필터 이동만으로는 캐릭터 분리가 약하므로, 하모닉 기울기/AP를 모듈별로 별도 제어.
                double f0_ref = std::max(80.0, out_f0[i]);
                double harm_idx = hz / f0_ref;
                double harm_hi = std::clamp((harm_idx - 3.0) / 16.0, 0.0, 1.0);
                double harm_mid = std::exp(-0.5 * std::pow((harm_idx - 8.0) / 4.2, 2.0));
                double src_voiced = std::clamp(0.24 + 0.76 * voiced_eff, 0.0, 1.0);

                double db_src_vtr =
                    vtr_pos * vtr_drive * (6.2 * (1.40 * harm_hi + 0.78 * f3 - 0.62 * f1))
                  - vtr_neg * vtr_drive * (6.8 * (1.12 * harm_hi + 0.72 * f2 + 0.52 * f3));
                db_src_vtr = sat_db(db_src_vtr, 12.0);

                double db_src_vtw = vtw_drive * (
                    vtw_pos * (7.8 * harm_mid - 4.9 * vtw_spread) +
                    vtw_neg * (-7.2 * harm_mid + 4.8 * vtw_spread)
                );
                db_src_vtw = sat_db(db_src_vtw, 13.0);

                double db_src_vc = vc_drive * src_voiced *
                    (4.9 * vc_core + 2.8 * constr_hi - 2.8 * constr_low - 2.4 * vc_anti);
                db_src_vc = sat_db(db_src_vc, 10.5);

                double db_src_nn = (0.30 + 0.70 * voiced_eff) *
                    (nn_pos_eff * (3.5 * nasal_form + 2.4 * nasal_low + 1.5 * nasal_bridge
                                  - 3.2 * nasal_notch1 - 2.6 * nasal_notch2)
                   + nn_neg_eff * (-2.2 * nasal_form - 1.6 * nasal_low - 1.0 * nasal_bridge
                                  + 1.8 * nasal_notch1 + 1.4 * nasal_notch2 + 1.2 * f2));
                db_src_nn = sat_db(db_src_nn, 10.8);

                double mix_src_vtr = std::clamp((0.16 + 0.66 * vtr_drive) * src_voiced, 0.0, 0.92);
                double mix_src_vtw = std::clamp((0.14 + 0.68 * vtw_drive) * src_voiced, 0.0, 0.94);
                double mix_src_vc  = std::clamp((0.13 + 0.62 * vc_drive) * src_voiced, 0.0, 0.86);
                double mix_src_nn  = std::clamp((0.11 + 0.60 * nn_drive) * (0.30 + 0.70 * voiced_eff), 0.0, 0.84);
                double tract_src_boost = 1.16;
                mix_src_vtr = std::clamp(mix_src_vtr * tract_src_boost, 0.0, 0.96);
                mix_src_vtw = std::clamp(mix_src_vtw * tract_src_boost, 0.0, 0.98);
                mix_src_vc  = std::clamp(mix_src_vc  * tract_src_boost, 0.0, 0.92);
                mix_src_nn  = std::clamp(mix_src_nn  * tract_src_boost, 0.0, 0.90);

                // 모듈별 독립 적용:
                // 공통 dB 합산 대신 각 모듈을 순차 blend 적용해 캐릭터 섞임을 줄인다.
                double p = p0;
                p = apply_module(p, db_mo,  mix_mo, 0.50, 0.58, 1.98);
                p = apply_module(p, db_vtr, mix_vtr, 1.36 + 0.36 * vtr_drive, 0.22, 5.20);
                p = apply_module(p, db_vtw, mix_vtw, 1.32 + 0.82 * vtw_drive, 0.18, 5.50);
                p = apply_module(p, db_vc,  mix_vc,  1.26 + 0.24 * vc_drive, 0.26, 4.60);
                p = apply_module(p, db_nn,  mix_nn,  1.10 + 0.42 * nn_drive, 0.24, 4.30);
                p = apply_module(p, db_src_vtr, mix_src_vtr, 0.88, 0.46, 3.05);
                p = apply_module(p, db_src_vtw, mix_src_vtw, 1.04, 0.40, 3.30);
                p = apply_module(p, db_src_vc,  mix_src_vc,  0.78, 0.50, 2.60);
                p = apply_module(p, db_src_nn,  mix_src_nn,  0.80, 0.50, 2.66);
                out_spec[i][k] = std::max(0.0, p);
                e1 += out_spec[i][k];

                double ap_delta = 0.0;
                // 모듈별 AP 연동 (source character 반영)
                double ap_vc = vc_drive * (0.007 + 0.026 * constr_hi + 0.010 * constr_top - 0.013 * constr_mid);
                double ap_nn = nn_pos_eff * (0.004 + 0.018 * nasal_form + 0.008 * nasal_low
                                            - 0.017 * nasal_notch1 - 0.012 * nasal_notch2)
                              - nn_neg_eff * (0.004 + 0.016 * nasal_form + 0.008 * nasal_low);
                double ap_src_vtr = (vtr_pos * 0.010 - vtr_neg * 0.014) * (0.30 + 0.70 * harm_hi);
                double ap_src_vtw = (-vtw_pos * (0.008 + 0.020 * harm_mid) + vtw_neg * (0.006 + 0.016 * harm_mid)) * src_voiced;
                double ap_src_vc = -vc_drive * src_voiced * (0.010 + 0.028 * vc_core + 0.014 * constr_hi);
                double ap_src_nn = nn_pos_eff * (0.006 + 0.020 * nasal_form + 0.010 * nasal_low
                                                - 0.012 * nasal_notch1)
                                  - nn_neg_eff * (0.006 + 0.018 * nasal_form + 0.008 * nasal_low);
                ap_delta += ap_vc * (0.08 + 0.18 * mix_vc);
                ap_delta += ap_nn * (0.08 + 0.18 * mix_nn);
                ap_delta += ap_src_vtr;
                ap_delta += ap_src_vtw;
                ap_delta += ap_src_vc;
                ap_delta += ap_src_nn;
                ap_delta = std::clamp(ap_delta, -0.024, 0.046);
                out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
            }
            if (e0 > 1.0e-12 && e1 > 1.0e-12) {
                double norm = e0 / e1;
                // 공통 정규화로 캐릭터가 죽는 현상을 막기 위해
                // 큰 loudness 드리프트가 있을 때만 약하게 보정.
                if (norm < 0.80 || norm > 1.22) {
                    norm = std::clamp(norm, 0.56, 1.78);
                    norm = 1.0 + 0.30 * (norm - 1.0);
                    norm = std::clamp(norm, 0.72, 1.34);
                    for (int k = 0; k < spec_dim; ++k) out_spec[i][k] *= norm;
                }
            }
        }

        // 5.5. Attack(At): 노트 초반 선명도/완화
        if (std::fabs(at) > 0.01) {
            double attack_win_ms = std::clamp(22.0 + tr_scale * 6.0, 18.0, 34.0);
            if (out_time_ms <= attack_win_ms) {
                double u = 1.0 - std::clamp(out_time_ms / attack_win_ms, 0.0, 1.0);
                double a_pos = std::max(0.0, at);
                double a_neg = std::max(0.0, -at);
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                    double hz = fn * (fs * 0.5);
                    double x = std::log2((hz + 120.0) / 3000.0);
                    double bell = std::exp(-0.5 * (x * x) / (0.75 * 0.75));
                    double db = u * (a_pos * (5.8 * bell + 2.6 * std::max(0.0, fn - 0.22))
                                   - a_neg * (4.2 * bell + 1.6 * std::max(0.0, fn - 0.16)));
                    out_spec[i][k] *= std::pow(10.0, db / 10.0);
                    double ap_delta = u * (-a_pos * (0.12 * bell + 0.05) + a_neg * (0.14 * bell + 0.07));
                    out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
                }
            }
        }

        // 6. Tension: 스펙트럼 + 비주기성(AP) 제어
        // T+ 최대에서도 볼륨 저하/먹먹함이 나지 않도록
        // 고역 선명도 보강 + 프레임 에너지 정규화를 함께 적용.
        if (std::fabs(tension_eff) > 0.01) {
            double t_pos = std::max(0.0, tension_eff);
            double t_neg = std::max(0.0, -tension_eff);
            double e0 = 0.0;
            double e1 = 0.0;
            for (int k = 0; k < spec_dim; ++k) {
                double fn = static_cast<double>(k) / std::max(1, spec_dim - 1); // 0..1
                double hz = fn * (fs * 0.5);
                double p0 = std::max(0.0, out_spec[i][k]);
                e0 += p0;

                // spectral effort:
                // - pressed(T+): 2~5k 존재감 + 상부 선명도 보강, 저중역 과중 억제
                // - relaxed(T-): 기존과 유사하게 존재감/긴장도 완화
                double x_pres = std::log2((hz + 120.0) / 2800.0);
                double presence = std::exp(-0.5 * (x_pres * x_pres) / (0.72 * 0.72));
                double x_mid = std::log2((hz + 120.0) / 1300.0);
                double mid = std::exp(-0.5 * (x_mid * x_mid) / (0.85 * 0.85));
                double x_low = std::log2((hz + 120.0) / 650.0);
                double low = std::exp(-0.5 * (x_low * x_low) / (0.95 * 0.95));
                double hi = std::clamp((fn - 0.42) / 0.58, 0.0, 1.0);
                double air = std::clamp((fn - 0.55) / 0.45, 0.0, 1.0);

                double db = t_pos * (11.6 * presence + 3.8 * mid + 5.9 * hi + 3.4 * air - 2.0 * low)
                          - t_neg * (14.6 * presence + 8.2 * std::max(0.0, fn - 0.10) + 5.2 * mid + 1.2 * low);
                out_spec[i][k] = p0 * std::pow(10.0, db / 10.0); // power
                e1 += out_spec[i][k];

                // aperiodicity effort:
                // - pressed(T+): 저/중역 AP 감소는 유지하되, 상부 AP를 약간 살려 먹먹함 방지
                // - relaxed(T-): 전대역 AP 증가
                double low_mid = std::exp(-0.5 * std::pow((fn - 0.18) / 0.20, 2.0));
                double ap_delta = 0.0;
                ap_delta -= t_pos * (0.18 * low_mid + 0.05 * (1.0 - hi));
                ap_delta += t_pos * (0.05 * hi);
                // T-에서 노이즈를 더 강하게 억제:
                // 특히 상부 AP를 줄여 hiss를 낮추고, 대신 스펙트럼 차이로 텐션 체감을 만든다.
                ap_delta -= t_neg * (0.06 + 0.08 * low_mid + 0.16 * hi);
                out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
            }

            // T+에서 과도한 피크 상승을 막기 위해
            // 프레임 에너지를 맞추되 crest를 약하게 가드.
            if (e0 > 1.0e-12 && e1 > 1.0e-12) {
                double norm = e0 / e1;
                norm = std::clamp(norm, 0.86, 1.24);
                // pressed(T+)에서 crest factor가 커지므로 소량 감쇠로 피크를 억제.
                double crest_guard = std::pow(10.0, (-0.8 * t_pos) / 10.0); // max -0.8dB
                norm *= crest_guard;
                // relaxed(T-)는 정규화로 효과가 상쇄되지 않도록
                // 상한을 낮추고 소폭 sag를 부여해 "힘 빠짐" 체감을 유지.
                double relax_sag = std::pow(10.0, (-1.1 * t_neg) / 10.0); // max -1.1dB
                norm *= relax_sag;
                double norm_hi = 1.24 - 0.38 * t_neg; // t-100 -> ~0.86
                if (norm_hi < 0.86) norm_hi = 0.86;
                norm = std::clamp(norm, 0.76, norm_hi);
                for (int k = 0; k < spec_dim; ++k) out_spec[i][k] *= norm;
            }
        }

        // 6.2. Growl(Gr): 저중역 rasp + 비주기성 강화
        if (growl_amt > 0.01) {
            double growl_voicing = 0.30 + 0.70 * voiced_eff;
            double gr_lfo = 0.82 + 0.18 * std::sin(2.0 * math::PI * (0.0072 * i));
            double g = growl_amt * growl_voicing * gr_lfo;
            for (int k = 0; k < spec_dim; ++k) {
                double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                double rasp1 = std::exp(-0.5 * std::pow((fn - 0.12) / 0.09, 2.0)); // ~1.4kHz
                double rasp2 = std::exp(-0.5 * std::pow((fn - 0.25) / 0.10, 2.0)); // ~2.9kHz
                double rasp3 = std::exp(-0.5 * std::pow((fn - 0.36) / 0.10, 2.0)); // ~4.1kHz
                double low = std::exp(-0.5 * std::pow((fn - 0.04) / 0.07, 2.0));
                double db = g * (5.8 * rasp1 + 3.8 * rasp2 + 1.8 * rasp3 - 1.6 * low);
                out_spec[i][k] *= std::pow(10.0, db / 10.0);
                double ap_delta = g * (0.09 + 0.26 * rasp1 + 0.18 * rasp2 + 0.10 * rasp3);
                out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
            }
        }

        // 6.5. Voice Color: 폼ант 존재감/톤 컬러링 (강화)
        if (std::fabs(voice_color_db) > 0.2) {
            for (int k = 0; k < spec_dim; ++k) {
                double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                double hz = fn * (fs * 0.5);
                double x1 = std::log2((hz + 120.0) / 2200.0);
                double bell1 = std::exp(-0.5 * (x1 * x1) / (0.85 * 0.85));
                double x2 = std::log2((hz + 120.0) / 4200.0);
                double bell2 = std::exp(-0.5 * (x2 * x2) / (0.90 * 0.90));
                double shelf = fn - 0.30;
                double db = voice_color_db * (0.90 * bell1 + 0.55 * bell2 + 0.50 * shelf);
                double gain_pow = std::pow(10.0, db / 10.0); // power
                out_spec[i][k] *= gain_pow;

                // 컬러 변화가 단순 EQ처럼 들리지 않게 AP도 약하게 연동.
                double ap_col = -vc_eff * (0.06 * bell1 + 0.03 * bell2);
                out_ap[i][k] = std::clamp(out_ap[i][k] + ap_col, 0.0, 1.0);
            }
        }

        // 6.7. Breathiness(Bh): airy 질감 강화(+) / 억제(-)
        if (std::fabs(breathiness_eff) > 0.01) {
            double bh_pos = std::max(0.0, breathiness_eff);
            double bh_neg = std::max(0.0, -breathiness_eff);
            for (int k = 0; k < spec_dim; ++k) {
                double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                double hz = fn * (fs * 0.5);
                double x_air = std::log2((hz + 120.0) / 4300.0);
                double air = std::exp(-0.5 * (x_air * x_air) / (0.95 * 0.95));
                double x_body = std::log2((hz + 120.0) / 1200.0);
                double body = std::exp(-0.5 * (x_body * x_body) / (0.95 * 0.95));

                // +Bh: airy 추가 / -Bh: 숨소리 억제 + 바디 복원
                double db = bh_pos * (2.6 * air - 1.8 * body + 1.3 * std::max(0.0, fn - 0.35))
                          + bh_neg * (1.5 * body - 2.9 * air - 1.7 * std::max(0.0, fn - 0.30));
                double gain_pow = std::pow(10.0, db / 10.0);
                out_spec[i][k] *= gain_pow;

                double ap_delta = bh_pos * (0.05 + 0.24 * fn + 0.30 * air)
                                - bh_neg * (0.06 + 0.22 * fn + 0.34 * air);
                out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
            }
        }

        // 7. Noise (N): +추가 / -억제
        {
            double nlev = std::clamp(sp.noise_level / 100.0, -1.0, 1.0);
            if (std::fabs(nlev) > 0.01) {
                double n_pos = std::max(0.0, nlev);
                double n_neg = std::max(0.0, -nlev);
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                    double ap_delta = 0.0;
                    ap_delta += n_pos * (0.10 + 0.62 * std::pow(fn, 0.70));
                    ap_delta -= n_neg * (0.08 + 0.58 * std::pow(fn, 0.92));
                    out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);

                    // N<0일 때 hiss가 과하게 남지 않도록 스펙트럼도 약하게 정돈.
                    if (n_neg > 0.01) {
                        double hi = std::clamp((fn - 0.45) / 0.55, 0.0, 1.0);
                        double body = std::exp(-0.5 * std::pow((fn - 0.20) / 0.20, 2.0));
                        double db = n_neg * (0.8 * body - 1.4 * hi);
                        out_spec[i][k] *= std::pow(10.0, db / 10.0);
                    }
                }
            }
        }

        // 8. Harmonics (H/Hr): 70 중립, 그 이상은 배음 강조
        {
            double h = std::clamp(static_cast<double>(sp.harmonics), 0.0, 100.0);
            constexpr double h0 = 70.0; // neutral
            if (std::fabs(h - h0) > 0.5) {
                if (h >= h0) {
                    double x = (h - h0) / (100.0 - h0); // 0..1
                    double boost = 1.0 + 1.20 * std::pow(x, 0.85); // 1.0..2.2
                    for (int k = 0; k < spec_dim; ++k) {
                        double harm = (1.0 - out_ap[i][k]) * boost;
                        harm = std::clamp(harm, 0.0, 1.0);
                        out_ap[i][k] = 1.0 - harm;

                        // 배음 강조가 단순 노이즈 감소로만 들리지 않도록 존재감 보강
                        double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                        double body = std::exp(-0.5 * std::pow((fn - 0.22) / 0.18, 2.0));
                        double pres = std::exp(-0.5 * std::pow((fn - 0.46) / 0.16, 2.0));
                        double db = std::pow(x, 0.90) * (1.2 * body + 1.6 * pres);
                        out_spec[i][k] *= std::pow(10.0, db / 10.0);
                    }
                } else {
                    double x = h / h0; // 0..1
                    double mild = std::clamp((h0 - h) / 30.0, 0.0, 1.0); // H40..69 구간
                    double atten = (h >= 40.0)
                        ? (1.0 - 0.34 * std::pow(mild, 0.88))
                        : std::pow(x, 1.18);
                    for (int k = 0; k < spec_dim; ++k) {
                        double harm = (1.0 - out_ap[i][k]) * atten;
                        harm = std::clamp(harm, 0.0, 1.0);
                        out_ap[i][k] = 1.0 - harm;
                        if (h >= 40.0) {
                            double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                            double body = std::exp(-0.5 * std::pow((fn - 0.20) / 0.22, 2.0));
                            double hi = std::clamp((fn - 0.48) / 0.52, 0.0, 1.0);
                            double db = mild * (0.9 * body - 0.6 * hi);
                            out_spec[i][k] *= std::pow(10.0, db / 10.0);
                        }
                    }
                }
            }
        }

        // 8.5. Noise Color(Ns): 노이즈 톤(밝기/어둠) 제어
        if (std::fabs(ns) > 0.01) {
            double n_pos = std::max(0.0, sp.noise_level / 100.0);
            double bh_pos = std::max(0.0, breathiness_eff);
            double ns_abs = std::fabs(ns);
            double base_presence = 0.40 + 0.40 * ns_abs;
            double noise_presence = std::max(base_presence, std::max({n_pos * 1.10, bh_pos * 0.95, rl * 0.90}));
            if (noise_presence > 0.01) {
                double e0 = 0.0;
                double e1 = 0.0;
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                    double hz = fn * (fs * 0.5);
                    double p0 = std::max(0.0, out_spec[i][k]);
                    e0 += p0;
                    double bright = std::exp(-0.5 * std::pow((hz - 5000.0) / 1550.0, 2.0));
                    double warm = std::exp(-0.5 * std::pow((hz - 520.0) / 310.0, 2.0));
                    double slope = fn - 0.42;
                    double db = ns * noise_presence *
                              (4.8 * bright - 4.1 * warm + 1.4 * slope);
                    out_spec[i][k] = p0 * std::pow(10.0, db / 10.0);
                    e1 += out_spec[i][k];
                    double ap_delta = ns * noise_presence *
                                    (0.14 * bright - 0.09 * warm + 0.04 * slope);
                    ap_delta = std::clamp(ap_delta, -0.12, 0.12);
                    out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
                }
                if (e0 > 1.0e-12 && e1 > 1.0e-12) {
                    double norm = std::clamp(e0 / e1, 0.72, 1.34);
                    norm = 1.0 + 0.72 * (norm - 1.0);
                    norm = std::clamp(norm, 0.86, 1.18);
                    for (int k = 0; k < spec_dim; ++k) out_spec[i][k] *= norm;
                }
            }
        }

        // 8.7. Release Air(Rl): 노트 말미 airy tail
        if (rl > 0.01) {
            double rel_win_ms = std::clamp(30.0 + tr_scale * 14.0, 24.0, 60.0);
            if (out_time_ms >= (out_total_ms - rel_win_ms)) {
                double v = (out_time_ms - (out_total_ms - rel_win_ms)) / std::max(1.0, rel_win_ms);
                v = std::clamp(v, 0.0, 1.0);
                double r = v * v * (3.0 - 2.0 * v); // smoothstep
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                    double db = rl * r * (1.6 * std::max(0.0, fn - 0.25));
                    out_spec[i][k] *= std::pow(10.0, db / 10.0);
                    double ap_delta = rl * r * (0.05 + 0.26 * fn);
                    out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
                }
            }
        }

        // 8.8. Airy-weak voice 보정:
        // 숨소리가 많은 약한 음성에서 과한 거칠기(상부 hiss)를 줄이고 바디를 보강.
        {
            double airy_mix = std::clamp(0.65 * std::max(0.0, breathiness_eff) +
                                         0.45 * std::max(0.0, sp.noise_level / 100.0), 0.0, 1.0);
            double weak_harm = std::clamp((75.0 - sp.harmonics) / 75.0, 0.0, 1.0);
            double ctrl = airy_mix * (0.35 + 0.65 * weak_harm);
            if (ctrl > 0.02) {
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                    double hi = std::clamp((fn - 0.50) / 0.50, 0.0, 1.0);
                    double body = std::exp(-0.5 * std::pow((fn - 0.20) / 0.20, 2.0));
                    double db = ctrl * (1.6 * body - 1.8 * hi);
                    out_spec[i][k] *= std::pow(10.0, db / 10.0);
                    double ap_delta = ctrl * (-0.08 * hi + 0.03 * body);
                    out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
                }
            }
        }

        // 8.9. Husky Tone(Hu): 파워/AP 유지형 톤 이동
        // +Hu: brighter(상부 명료도↑, 저중역 질감↓)
        // -Hu: husky(저중역 질감↑, 상부 밝기↓)
        if (std::fabs(hu_eff) > 0.01) {
            double hu_husky = std::max(0.0, hu_eff);   // 사용자가 Hu 음수로 준 경우
            double hu_bright = std::max(0.0, -hu_eff);
            if (hu_husky > 0.01 && !hu_warp_buf.empty()) {
                double hu_ratio = std::clamp(std::exp(-0.052 * hu_husky), 0.94, 1.0);
                double hu_blend = std::clamp(0.10 + 0.24 * hu_husky, 0.0, 0.34);
                double inv_hu_ratio = 1.0 / hu_ratio;
                for (int k = 0; k < spec_dim; ++k) {
                    double src_k = k * inv_hu_ratio;
                    src_k = std::clamp(src_k, 0.0, static_cast<double>(spec_dim - 1));
                    int sk = static_cast<int>(src_k);
                    int sk2 = std::min(sk + 1, spec_dim - 1);
                    double f = src_k - sk;
                    hu_warp_buf[k] = out_spec[i][sk] * (1.0 - f) + out_spec[i][sk2] * f;
                }
                for (int k = 0; k < spec_dim; ++k) {
                    out_spec[i][k] = out_spec[i][k] * (1.0 - hu_blend) + hu_warp_buf[k] * hu_blend;
                }
            }
            double e0 = 0.0;
            double e1 = 0.0;
            for (int k = 0; k < spec_dim; ++k) {
                double p0 = out_spec[i][k];
                if (p0 < 0.0) p0 = 0.0;
                e0 += p0;

                double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                double hz = fn * (fs * 0.5);
                double x_lm = std::log2((hz + 120.0) / 1100.0);
                double lowmid = std::exp(-0.5 * (x_lm * x_lm) / (0.90 * 0.90));
                double x_pr = std::log2((hz + 120.0) / 3600.0);
                double presence = std::exp(-0.5 * (x_pr * x_pr) / (0.85 * 0.85));
                double air = std::max(0.0, fn - 0.45);
                double throat_low = std::exp(-0.5 * std::pow((hz - 520.0) / 360.0, 2.0));
                double rasp = std::exp(-0.5 * std::pow((hz - 1650.0) / 760.0, 2.0));
                double shape = (1.25 * lowmid - 1.10 * presence - 0.75 * air)
                             + hu_husky * (0.42 * throat_low + 0.34 * rasp - 0.18 * presence)
                             - hu_bright * (0.20 * throat_low);
                double db = hu_eff * 8.5 * shape;
                double gain_pow = std::pow(10.0, db / 10.0);
                out_spec[i][k] = p0 * gain_pow;
                e1 += out_spec[i][k];

                double ap_delta = hu_husky * (0.018 * rasp + 0.010 * lowmid - 0.022 * air)
                                - hu_bright * (0.012 * lowmid);
                out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
            }
            // 프레임 에너지 정규화: 파워 변화 최소화
            if (e0 > 1.0e-12 && e1 > 1.0e-12) {
                double norm = e0 / e1;
                norm = std::clamp(norm, 0.6, 1.8);
                for (int k = 0; k < spec_dim; ++k) out_spec[i][k] *= norm;
            }
        }

        // 9. Global tone calibration: 고역/잔향 과강조 완화
        {
            for (int k = 0; k < spec_dim; ++k) {
                double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                double hz = fn * (fs * 0.5);
                double hi = std::clamp((fn - 0.34) / 0.66, 0.0, 1.0);
                double db = global_hi_tilt_db * hi;
                double puff = std::exp(-0.5 * std::pow((hz - 90.0) / 85.0, 2.0));
                double puff_gate = (in_consonant ? 1.0 : (in_transition ? 0.65 : 0.28));
                db -= puff_gate * 1.65 * puff;
                out_spec[i][k] *= std::pow(10.0, db / 10.0);

                double hiss_guard = std::pow(std::clamp((fn - 0.48) / 0.52, 0.0, 1.0), 1.15);
                double ap_cut = global_hi_ap_trim * hi * hi
                              + 0.018 * hiss_guard
                              + (0.010 + 0.024 * puff_gate) * puff;
                out_ap[i][k] = std::clamp(out_ap[i][k] - ap_cut, 0.0, 1.0);
            }
        }

        // 9.2. broadband noise/pop guard:
        // AP와 초저역이 프레임 단위로 튀는 경우만 약하게 눌러 바람/팝 노이즈를 줄인다.
        if (i > 0) {
            double guard_base = 0.018 + (in_consonant ? 0.030 : (in_transition ? 0.020 : 0.0));
            for (int k = 0; k < spec_dim; ++k) {
                double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                double hz = fn * (fs * 0.5);
                double hi = std::clamp((fn - 0.52) / 0.48, 0.0, 1.0);
                double low_puff = std::exp(-0.5 * std::pow((hz - 110.0) / 95.0, 2.0));
                double w_ap = std::clamp(guard_base + 0.030 * hi + 0.024 * low_puff, 0.0, 0.095);
                out_ap[i][k] = out_ap[i][k] * (1.0 - w_ap) + out_ap[i - 1][k] * w_ap;
                if (low_puff > 0.04) {
                    double w_spec = std::clamp((in_consonant ? 0.050 : 0.024) * low_puff, 0.0, 0.070);
                    out_spec[i][k] = out_spec[i][k] * (1.0 - w_spec) + out_spec[i - 1][k] * w_spec;
                }
            }
        }

        // 9.5. 연결부 연속성 스무딩:
        // consonant/transition 구간에서 프레임 간 급변을 완화해
        // 음소 연결의 "툭" 끊김과 인위적 경계감을 줄인다.
        if (i > 0) {
            double join_smooth = 0.0;
            if (in_consonant || in_transition) {
                join_smooth = 0.10 + 0.14 * cs_pos + 0.06 * cs_neg;
            } else if (!has_consonant_head && out_time_ms <= 8.0) {
                // 어두 무자음 진입의 미세 경계 완화
                join_smooth = 0.08;
            }
            join_smooth = std::clamp(join_smooth, 0.0, 0.30);
            if (join_smooth > 1.0e-4) {
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                    double w_spec = std::clamp(join_smooth * (0.92 - 0.32 * fn), 0.0, 0.35);
                    double w_ap   = std::clamp(join_smooth * (0.70 + 0.25 * fn), 0.0, 0.35);
                    out_spec[i][k] = out_spec[i][k] * (1.0 - w_spec) + out_spec[i - 1][k] * w_spec;
                    out_ap[i][k]   = out_ap[i][k]   * (1.0 - w_ap)   + out_ap[i - 1][k]   * w_ap;
                }
            }
        }
    }

    // 최종 F0 프레임 jitter 미세 억제 (zero-phase FIR).
    // flat note에서는 더 강하게, 변조 노트에서는 약하게.
    if (flat_target_f0) {
        smooth_out_f0_log_zero_phase(out_f0, 4);
    } else if (has_f0_mod) {
        smooth_out_f0_log_zero_phase(out_f0, 2);
    }
    // slew limiter 강도:
    // - flat note: 강하게(잔떨림 억제)
    // - bend/mod note: 약하게(음정 추종성 확보)
    double slew_cents_per_ms = flat_target_f0 ? 2.0 : (has_f0_mod ? 18.0 : 6.0);
    stabilize_out_f0(out_f0, frame_period, slew_cents_per_ms);

    // ── WORLD Synthesis ───────────────────────────────────────────────
    std::vector<double> y(output_samples, 0.0);
    std::vector<double*> spec_ptrs(out_n_frames);
    std::vector<double*> ap_ptrs  (out_n_frames);
    for (int i = 0; i < out_n_frames; ++i) {
        spec_ptrs[i] = out_spec[i].data();
        ap_ptrs[i]   = out_ap[i].data();
    }

    Synthesis(out_f0.data(), out_n_frames,
              const_cast<const double* const*>(spec_ptrs.data()),
              const_cast<const double* const*>(ap_ptrs.data()),
              src.fft_size, frame_period, fs, output_samples, y.data());

    // double → float
    std::vector<float> output(output_samples);
    for (int i = 0; i < output_samples; ++i) {
        double v = y[i];
        if (!std::isfinite(v)) v = 0.0;
        output[i] = static_cast<float>(v);
    }

    std::cerr << "[Resamp] WORLD synthesized: out_frames=" << out_n_frames
              << " warp_ratio=" << formant_ratio
              << " Mo=" << sp.mouth_open
              << " mo_eff=" << mo_eff
              << " mo_ratio=" << mo_formant_ratio
              << " mo_blend=" << mo_formant_blend
              << " Vtl=" << sp.tract_length
              << " Vtr=" << sp.tract_resonance
              << " Vtw=" << sp.tract_focus
              << " Vc=" << sp.tract_constriction
              << " Nn=" << sp.nasal_coupling
              << " g_eff=" << gender_eff
              << " vtl_eff=" << vtl_eff
              << " vtr_eff=" << vtr_eff
              << " vtw_eff=" << vtw_eff
              << " vc_amt=" << vc_amt
              << " nn_amt=" << nn_amt
              << " nn_pos=" << nn_pos_eff
              << " nn_neg=" << nn_neg_eff
              << " Tn=" << sp.tension
              << " t_eff=" << tension_eff
              << " Hu=" << sp.husky_tone
              << " hu_eff=" << hu_eff
              << " Ns=" << sp.noise_color
              << " ns_eff=" << ns
              << '\n';
    return output;
}

} // namespace resamp::synth
