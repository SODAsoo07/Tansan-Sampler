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
            src_time_ms = consonant_src_ms + transition_src_len_ms;
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
    // 합성 프레임 주기:
    // 0.125ms는 프레임 간 미세 흔들림을 과하게 노출시키는 경우가 있어 0.5ms로 고정.
    bool has_f0_mod = (params.modulation > 0) || !params.pitch_bend.empty();
    double frame_period = 0.5;
    int spec_dim       = src.fft_size / 2 + 1;
    double src_total_ms = src.n_frames * anal_period;
    double consonant_scale = std::max(0.01, params.velocity / 100.0);
    double consonant_src_ms = std::clamp(params.consonant_ms, 0.0, src_total_ms);
    double consonant_tgt_ms = consonant_src_ms * consonant_scale;

    // CVVC 연결부 에코 방지:
    // 전체 후반부를 루프하지 않고, 연결 구간(1회 통과) + 안정 유성 구간(루프)로 분리.
    int n_frames = src.n_frames;
    int start_fi = static_cast<int>(std::round(consonant_src_ms / anal_period));
    start_fi = std::clamp(start_fi, 0, n_frames - 1);

    int best_s = -1, best_e = -1;
    int cur_s = -1;
    for (int fi = start_fi; fi < n_frames; ++fi) {
        bool voiced = (src.f0[fi] >= 50.0);
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

    double loop_start_ms = loop_start_fi * anal_period;
    double loop_end_ms   = loop_end_fi * anal_period;
    double loop_len_ms   = std::max(0.0, loop_end_ms - loop_start_ms);
    bool has_vowel_loop  = loop_len_ms > (2.0 * anal_period);
    double transition_src_len_ms = std::max(0.0, loop_start_ms - consonant_src_ms);
    // 연결부는 너무 짧으면 끊김/툭툭거림이 생길 수 있어
    // 과도한 단축은 피하고, 중간 정도만 압축.
    double transition_tgt_len_ms = std::clamp(transition_src_len_ms * 0.85, 0.0, 35.0);
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
    bool flat_target_f0 = (!has_f0_mod && params.modulation <= 0 && voiced_cnt > 0 &&
                           (voiced_max - voiced_min) <= 0.45);

    double seam_ms = 0.0;
    if (has_vowel_loop) {
        // loop 경계 전후에서 end->start 파라미터를 부드럽게 잇는 크로스페이드 폭.
        seam_ms = std::clamp(loop_len_ms * 0.12, 3.0, 12.0);
    }

    // 안정화 모드: loop 구간(기본) + flat note(전체)에 anchor envelope/AP 적용.
    bool use_stable_vowel_env = has_vowel_loop || flat_target_f0;
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
    double formant_ratio = std::exp(-static_cast<double>(sp.gender) / 300.0);
    bool   do_warp       = std::fabs(formant_ratio - 1.0) > 0.005;

    // ── Brightness: 스펙트럼 기울기 (B=50 기본) ───────────────────────
    // power 도메인에서 freq에 대해 선형 dB 기울기 적용
    double brightness_tilt_db = (sp.brightness - 50) / 50.0 * 12.0;

    // ── Tension: 저역 부스트/감쇠 (간단 근사) ─────────────────────────
    double tension_lf_db = sp.tension / 100.0 * 6.0;  // ±6dB

    // 출력 프레임별 F0/envelope/AP 구성
    std::vector<double> out_f0(out_n_frames, 0.0);
    std::vector<std::vector<double>> out_spec(out_n_frames, std::vector<double>(spec_dim, 0.0));
    std::vector<std::vector<double>> out_ap  (out_n_frames, std::vector<double>(spec_dim, 0.0));

    double inv_ratio = 1.0 / formant_ratio;

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

        // source voicing 기반 F0 게이팅:
        // 무성/자음 구간은 음소 보호를 위해 mute하지만,
        // voiced로 판정된 구간에서는 F0를 감쇠하지 않아 보컬프라이 질감 방지.
        double sv0 = (src.f0[src_fi] >= 50.0) ? 1.0 : 0.0;
        double sv1 = (src.f0[src_fi2] >= 50.0) ? 1.0 : 0.0;
        double sv  = sv0 * (1.0 - frac) + sv1 * frac;
        // hard mute 대신 soft gate로 끊김 억제
        double vg;
        if (sv <= 0.03) vg = 0.0;
        else if (sv >= 0.25) vg = 1.0;
        else {
            double x = (sv - 0.03) / (0.25 - 0.03); // 0..1
            x = std::clamp(x, 0.0, 1.0);
            vg = x * x * (3.0 - 2.0 * x);           // smoothstep
        }
        out_f0[i] *= vg;
        if (vg < 0.01) out_f0[i] = 0.0;

        if (out_f0[i] < 50.0) out_f0[i] = 0.0;

        // 3. envelope/AP 시간 보간 (linear)
        const auto& s1 = src.spectrogram[src_fi];
        const auto& s2 = src.spectrogram[src_fi2];
        const auto& a1 = src.aperiodicity[src_fi];
        const auto& a2 = src.aperiodicity[src_fi2];
        for (int k = 0; k < spec_dim; ++k) {
            out_spec[i][k] = s1[k] * (1.0 - frac) + s2[k] * frac;
            out_ap[i][k]   = a1[k] * (1.0 - frac) + a2[k] * frac;
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

        // 6. Tension: 저역 강조/감쇠 (저역 1/8 대역에 한정 적용)
        if (std::fabs(tension_lf_db) > 0.5) {
            int lf_end = std::max(1, spec_dim / 8);
            double lf_gain = std::pow(10.0, tension_lf_db / 10.0);
            for (int k = 0; k < lf_end; ++k) {
                double w = 0.5 * (1.0 + std::cos(math::PI * static_cast<double>(k) / lf_end));
                double g = 1.0 + (lf_gain - 1.0) * w;
                out_spec[i][k] *= g;
            }
        }

        // 7. Noise (N): aperiodicity 부스트 (숨소리 추가)
        if (sp.noise_level > 0) {
            double ap_boost = sp.noise_level / 100.0 * 0.5;
            for (int k = 0; k < spec_dim; ++k)
                out_ap[i][k] = std::min(1.0, out_ap[i][k] + ap_boost);
        }

        // 8. Harmonics (Hr): 비주기성 스케일 (default 100=원본, 0=완전 노이즈)
        //    Hr<100 → AP↑ (조화성↓, 노이즈↑)
        if (sp.harmonics != 100) {
            double k_h = sp.harmonics / 100.0;
            for (int k = 0; k < spec_dim; ++k) {
                // (1 - ap)를 k_h배 → 조화 비율 조절
                double harm = (1.0 - out_ap[i][k]) * k_h;
                if (harm < 0.0) harm = 0.0;
                if (harm > 1.0) harm = 1.0;
                out_ap[i][k] = 1.0 - harm;
            }
        }
    }

    // 최종 F0 프레임 jitter 미세 억제 (zero-phase FIR).
    // flat note에서는 더 강하게, 변조 노트에서는 약하게.
    if (flat_target_f0) {
        smooth_out_f0_log_zero_phase(out_f0, 4);
    } else if (has_f0_mod) {
        smooth_out_f0_log_zero_phase(out_f0, 3);
    }
    // slew limiter 강도:
    // - flat note: 강하게(잔떨림 억제)
    // - bend/mod note: 약하게(음정 추종성 확보)
    double slew_cents_per_ms = flat_target_f0 ? 2.1 : (has_f0_mod ? 12.0 : 6.0);
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
