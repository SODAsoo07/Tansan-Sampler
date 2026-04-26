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
    // 0.125ms는 프레임 간 미세 흔들림을 과하게 노출시키는 경우가 있어 0.5ms로 고정.
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
    // frame_period를 작게 할수록 WORLD 내부 F0 보간 오차가 줄어
    // 빠른 피치 변조에서의 모듈레이션성 아티팩트가 감소.
    int out_n_frames = static_cast<int>(std::ceil(output_samples * 1000.0 /
                                                  (frame_period * fs))) + 2;

    // ── Gender 플래그: 포먼트 frequency warp 비율 ────────────────────
    // formant_ratio: 타겟 포먼트 / 원본 포먼트
    //   g=-100 → ratio≈1.395 (포먼트 ↑, 여성화)
    //   g=    0 → ratio=1.0   (변경 없음)
    //   g=+100 → ratio≈0.717 (포먼트 ↓, 남성화)
    double formant_ratio = std::exp(-static_cast<double>(sp.gender) / 260.0);
    bool   do_warp       = std::fabs(formant_ratio - 1.0) > 0.005;

    // ── Brightness: 스펙트럼 기울기 (B=50 기본) ───────────────────────
    // power 도메인에서 freq에 대해 선형 dB 기울기 적용
    double brightness_tilt_db = (sp.brightness - 50) / 50.0 * 18.0;
    double hu = std::clamp(sp.husky_tone / 100.0, -1.0, 1.0);
    double hu_eff = (hu >= 0.0) ? std::pow(hu, 0.78) : -std::pow(-hu, 0.78);
    double mo = std::clamp((sp.mouth_open - 50) / 50.0, -1.0, 1.0);
    double mo_eff = (mo >= 0.0) ? std::pow(mo, 0.78) : -std::pow(-mo, 0.78);
    // 기본 톤 캘리브레이션: 고역/잔향 과강조 완화
    double global_hi_tilt_db = -2.4;
    double global_hi_ap_trim = 0.045;

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

    // ── Breathiness(Bh): airy 질감 전용 제어 ─────────────────────────
    // N과 달리 "노이즈 양"보다 "숨소리 질감"에 초점.
    double breathiness_amt = std::pow(std::clamp(sp.breathiness / 100.0, 0.0, 1.0), 0.72);

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
            int half_win = has_f0_mod ? std::max(1, static_cast<int>(std::round(fs * 0.0005))) : 0; // ±0.5ms
            out_f0[i] = sample_frame_f0_from_contour(target_f0_per_sample, sf, half_win);
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
            std::vector<double> warped(spec_dim);
            for (int k = 0; k < spec_dim; ++k) {
                double src_k = k * inv_ratio;
                if (src_k < 0.0)              src_k = 0.0;
                if (src_k > spec_dim - 1)     src_k = spec_dim - 1;
                int    sk  = static_cast<int>(src_k);
                int    sk2 = std::min(sk + 1, spec_dim - 1);
                double f   = src_k - sk;
                warped[k]  = out_spec[i][sk] * (1.0 - f) + out_spec[i][sk2] * f;
            }
            out_spec[i] = std::move(warped);
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

        // 5.3. Mouth Open(Mo): 입 열림(개방도) 톤 이동
        // AP(기식량)는 유지하고 스펙트럼만 이동한 뒤 에너지 정규화.
        if (std::fabs(mo_eff) > 0.01) {
            double e0 = 0.0;
            double e1 = 0.0;
            for (int k = 0; k < spec_dim; ++k) {
                double p0 = std::max(0.0, out_spec[i][k]);
                e0 += p0;

                double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                double hz = fn * (fs * 0.5);
                double x_f1 = std::log2((hz + 120.0) / 850.0);
                double f1 = std::exp(-0.5 * (x_f1 * x_f1) / (0.80 * 0.80));
                double x_f2 = std::log2((hz + 120.0) / 1900.0);
                double f2 = std::exp(-0.5 * (x_f2 * x_f2) / (0.90 * 0.90));
                double hi = std::max(0.0, fn - 0.45);
                // +Mo: 개방형(저/중포먼트 강조), -Mo: 덜 열린 밝은 톤
                double shape = (1.35 * f1 + 0.65 * f2 - 0.55 * hi);
                double db = mo_eff * 6.8 * shape;
                double gain_pow = std::pow(10.0, db / 10.0);
                out_spec[i][k] = p0 * gain_pow;
                e1 += out_spec[i][k];
            }
            if (e0 > 1.0e-12 && e1 > 1.0e-12) {
                double norm = std::clamp(e0 / e1, 0.65, 1.65);
                for (int k = 0; k < spec_dim; ++k) out_spec[i][k] *= norm;
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

        // 6. Tension: 스펙트럼 + 비주기성(AP) 동시 제어 (강화)
        if (std::fabs(tension_eff) > 0.01) {
            double t_pos = std::max(0.0, tension_eff);
            double t_neg = std::max(0.0, -tension_eff);
            for (int k = 0; k < spec_dim; ++k) {
                double fn = static_cast<double>(k) / std::max(1, spec_dim - 1); // 0..1
                double hz = fn * (fs * 0.5);

                // spectral effort:
                // - pressed: 1~4k presence 상승
                // - relaxed: presence 하강 + 기울기 완화
                double x_pres = std::log2((hz + 120.0) / 2500.0);
                double presence = std::exp(-0.5 * (x_pres * x_pres) / (0.75 * 0.75));
                double x_mid = std::log2((hz + 120.0) / 1300.0);
                double mid = std::exp(-0.5 * (x_mid * x_mid) / (0.85 * 0.85));
                double slope = fn - 0.22;
                double db = t_pos * (9.5 * presence + 4.2 * slope + 2.0 * mid)
                          - t_neg * (7.2 * presence + 3.5 * std::max(0.0, fn - 0.08) + 1.8 * mid);
                double gain_pow = std::pow(10.0, db / 10.0); // power
                out_spec[i][k] *= gain_pow;

                // aperiodicity effort:
                // - pressed: 저/중역 AP 감소(주기성 증가)
                // - relaxed: 전대역 AP 증가(숨/힘빠짐)
                double low_mid = std::exp(-0.5 * std::pow((fn - 0.18) / 0.20, 2.0));
                double hi = std::clamp((fn - 0.35) / 0.65, 0.0, 1.0);
                double ap_delta = 0.0;
                ap_delta -= t_pos * (0.26 * low_mid + 0.10 * (1.0 - hi));
                ap_delta += t_neg * (0.30 + 0.18 * low_mid + 0.14 * hi);
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

        // 6.7. Breathiness(Bh): airy 질감 강화 (N과 별도)
        if (breathiness_amt > 0.01) {
            for (int k = 0; k < spec_dim; ++k) {
                double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                double hz = fn * (fs * 0.5);
                double x_air = std::log2((hz + 120.0) / 4300.0);
                double air = std::exp(-0.5 * (x_air * x_air) / (0.95 * 0.95));
                double x_body = std::log2((hz + 120.0) / 1200.0);
                double body = std::exp(-0.5 * (x_body * x_body) / (0.95 * 0.95));

                // 중역 배음은 조금 눌러 airy 대비를 키우고, 상부 에너지는 살짝 부스트.
                double db = breathiness_amt * (2.4 * air - 1.8 * body + 1.2 * std::max(0.0, fn - 0.35));
                double gain_pow = std::pow(10.0, db / 10.0);
                out_spec[i][k] *= gain_pow;

                double ap_delta = breathiness_amt * (0.05 + 0.24 * fn + 0.30 * air);
                out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
            }
        }

        // 7. Noise (N): aperiodicity 부스트 (숨소리 추가)
        if (sp.noise_level > 0) {
            double ap_boost = sp.noise_level / 100.0 * 0.8;
            for (int k = 0; k < spec_dim; ++k)
                out_ap[i][k] = std::min(1.0, out_ap[i][k] + ap_boost);
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
                    double atten = std::pow(x, 1.25);
                    for (int k = 0; k < spec_dim; ++k) {
                        double harm = (1.0 - out_ap[i][k]) * atten;
                        harm = std::clamp(harm, 0.0, 1.0);
                        out_ap[i][k] = 1.0 - harm;
                    }
                }
            }
        }

        // 8.5. Noise Color(Ns): 노이즈 톤(밝기/어둠) 제어
        if (std::fabs(ns) > 0.01) {
            double noise_presence = std::max(0.12, std::max({sp.noise_level / 100.0, breathiness_amt, rl * 0.8}));
            if (noise_presence > 0.01) {
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = static_cast<double>(k) / std::max(1, spec_dim - 1);
                    double s = (fn - 0.45);
                    double db = ns * noise_presence * (2.8 * s);
                    out_spec[i][k] *= std::pow(10.0, db / 10.0);
                    double ap_delta = ns * noise_presence * (0.12 * s);
                    out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
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
            double airy_mix = std::clamp(0.65 * breathiness_amt + 0.45 * (sp.noise_level / 100.0), 0.0, 1.0);
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
        // +Hu: husky(저중역 질감↑, 상부 밝기↓)
        // -Hu: brighter(상부 명료도↑, 저중역 질감↓)
        if (std::fabs(hu_eff) > 0.01) {
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
                double shape = (1.25 * lowmid - 1.10 * presence - 0.75 * air);
                double db = hu_eff * 8.5 * shape;
                double gain_pow = std::pow(10.0, db / 10.0);
                out_spec[i][k] = p0 * gain_pow;
                e1 += out_spec[i][k];
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
                double hi = std::clamp((fn - 0.34) / 0.66, 0.0, 1.0);
                double db = global_hi_tilt_db * hi;
                out_spec[i][k] *= std::pow(10.0, db / 10.0);

                double ap_cut = global_hi_ap_trim * hi * hi;
                out_ap[i][k] = std::clamp(out_ap[i][k] - ap_cut, 0.0, 1.0);
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
              << " warp_ratio=" << formant_ratio << '\n';
    return output;
}

} // namespace resamp::synth
