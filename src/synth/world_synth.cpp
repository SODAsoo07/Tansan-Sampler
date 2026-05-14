#include "world_synth.hpp"
#include "util/math_util.hpp"

#include "world/dio.h"
#include "world/stonemask.h"
#include "world/cheaptrick.h"
#include "world/d4c.h"
#include "world/synthesis.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace resamp::synth {
namespace fs = std::filesystem;

struct FrameMatrix {
    FrameMatrix(int rows, int cols)
        : cols(cols),
          data(static_cast<size_t>(std::max(0, rows)) * static_cast<size_t>(std::max(0, cols)), 0.0) {}

    double* operator[](int row) {
        return data.data() + static_cast<size_t>(row) * static_cast<size_t>(cols);
    }

    const double* operator[](int row) const {
        return data.data() + static_cast<size_t>(row) * static_cast<size_t>(cols);
    }

    int cols = 0;
    std::vector<double> data;
};

struct SpectralCurveLut {
    explicit SpectralCurveLut(const std::vector<double>& fn_lut,
                              const std::vector<double>& hz_lut)
        : presence_2800(fn_lut.size()),
          mid_1300(fn_lut.size()),
          body_900(fn_lut.size()),
          low_650(fn_lut.size()),
          low_mid_018(fn_lut.size()),
          hi_42(fn_lut.size()),
          air_55(fn_lut.size()),
          top_68(fn_lut.size()),
          global_hi_34(fn_lut.size()),
          global_hiss_48(fn_lut.size()),
          puff_90(fn_lut.size()),
          guard_hi_52(fn_lut.size()),
          low_puff_110(fn_lut.size())
    {
        for (size_t k = 0; k < fn_lut.size(); ++k) {
            double fn = fn_lut[k];
            double hz = hz_lut[k];

            double x_pres = std::log2((hz + 120.0) / 2800.0);
            presence_2800[k] = std::exp(-0.5 * (x_pres * x_pres) / (0.72 * 0.72));
            double x_mid = std::log2((hz + 120.0) / 1300.0);
            mid_1300[k] = std::exp(-0.5 * (x_mid * x_mid) / (0.85 * 0.85));
            double x_body = std::log2((hz + 120.0) / 900.0);
            body_900[k] = std::exp(-0.5 * (x_body * x_body) / (0.92 * 0.92));
            double x_low = std::log2((hz + 120.0) / 650.0);
            low_650[k] = std::exp(-0.5 * (x_low * x_low) / (0.95 * 0.95));
            low_mid_018[k] = std::exp(-0.5 * std::pow((fn - 0.18) / 0.20, 2.0));

            hi_42[k] = std::clamp((fn - 0.42) / 0.58, 0.0, 1.0);
            air_55[k] = std::clamp((fn - 0.55) / 0.45, 0.0, 1.0);
            top_68[k] = std::pow(std::clamp((fn - 0.68) / 0.32, 0.0, 1.0), 1.25);
            global_hi_34[k] = std::clamp((fn - 0.34) / 0.66, 0.0, 1.0);
            global_hiss_48[k] = std::pow(std::clamp((fn - 0.48) / 0.52, 0.0, 1.0), 1.15);
            puff_90[k] = std::exp(-0.5 * std::pow((hz - 90.0) / 85.0, 2.0));
            guard_hi_52[k] = std::clamp((fn - 0.52) / 0.48, 0.0, 1.0);
            low_puff_110[k] = std::exp(-0.5 * std::pow((hz - 110.0) / 95.0, 2.0));
        }
    }

    std::vector<double> presence_2800;
    std::vector<double> mid_1300;
    std::vector<double> body_900;
    std::vector<double> low_650;
    std::vector<double> low_mid_018;
    std::vector<double> hi_42;
    std::vector<double> air_55;
    std::vector<double> top_68;
    std::vector<double> global_hi_34;
    std::vector<double> global_hiss_48;
    std::vector<double> puff_90;
    std::vector<double> guard_hi_52;
    std::vector<double> low_puff_110;
};

static bool env_flag_enabled(const char* name, bool default_value) {
    const char* v = std::getenv(name);
    if (v == nullptr || *v == '\0') return default_value;
    std::string s(v);
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (s == "0" || s == "false" || s == "off" || s == "no") return false;
    if (s == "1" || s == "true" || s == "on" || s == "yes") return true;
    return default_value;
}

static bool verbose_log_enabled() {
    static bool enabled = env_flag_enabled("RESAMP_VERBOSE", false);
    return enabled;
}

static int64_t env_int64_clamped(const char* name, int64_t default_value, int64_t min_value, int64_t max_value) {
    const char* v = std::getenv(name);
    if (v == nullptr || *v == '\0') return default_value;
    try {
        size_t pos = 0;
        long long parsed = std::stoll(std::string(v), &pos, 10);
        if (pos == 0) return default_value;
        return std::clamp<int64_t>(static_cast<int64_t>(parsed), min_value, max_value);
    } catch (...) {
        return default_value;
    }
}

static bool file_exists_noerr(const fs::path& path) {
    std::error_code ec;
    return fs::exists(path, ec);
}

static fs::path find_voicebank_root_for_cache(const fs::path& source_wav_path) {
    std::error_code ec;
    fs::path cur = fs::absolute(source_wav_path, ec).parent_path();
    if (cur.empty()) cur = source_wav_path.parent_path();
    if (cur.empty()) cur = fs::current_path(ec);

    fs::path best_oto_root;
    for (int depth = 0; depth < 10 && !cur.empty(); ++depth) {
        if (file_exists_noerr(cur / "character.txt") ||
            file_exists_noerr(cur / "character.yaml") ||
            file_exists_noerr(cur / "character.yml") ||
            file_exists_noerr(cur / "prefix.map")) {
            return cur;
        }
        if (best_oto_root.empty() && file_exists_noerr(cur / "oto.ini")) {
            best_oto_root = cur;
        }

        fs::path parent = cur.parent_path();
        if (parent == cur || parent.empty()) break;
        cur = parent;
    }

    if (!best_oto_root.empty()) return best_oto_root;
    fs::path parent = source_wav_path.parent_path();
    if (!parent.empty()) return parent;
    return fs::path(".");
}

static int64_t analysis_cache_limit_bytes() {
    // Per voicebank cache cap. Override with RESAMP_CACHE_MAX_MB.
    int64_t mb = env_int64_clamped("RESAMP_CACHE_MAX_MB", 1024, 64, 65536);
    return mb * 1024ll * 1024ll;
}

struct CacheFileEntry {
    fs::path path;
    uintmax_t size = 0;
    int64_t time_key = 0;
};

static void prune_analysis_cache_dir(const fs::path& cache_root, int64_t max_bytes) {
    if (max_bytes <= 0) return;
    std::error_code ec;
    if (!fs::exists(cache_root, ec)) return;

    std::vector<CacheFileEntry> entries;
    uintmax_t total = 0;

    for (fs::directory_iterator it(cache_root, ec), end; !ec && it != end; it.increment(ec)) {
        const fs::path path = it->path();
        if (!it->is_regular_file(ec)) {
            ec.clear();
            continue;
        }
        if (path.extension() == ".tmp") {
            fs::remove(path, ec);
            ec.clear();
            continue;
        }
        if (path.extension() != ".rswa") continue;

        uintmax_t size = fs::file_size(path, ec);
        if (ec) {
            ec.clear();
            continue;
        }
        auto wt = fs::last_write_time(path, ec);
        int64_t time_key = 0;
        if (!ec) time_key = static_cast<int64_t>(wt.time_since_epoch().count());
        ec.clear();

        entries.push_back({path, size, time_key});
        total += size;
    }

    if (static_cast<int64_t>(total) <= max_bytes) return;
    std::sort(entries.begin(), entries.end(), [](const CacheFileEntry& a, const CacheFileEntry& b) {
        if (a.time_key != b.time_key) return a.time_key < b.time_key;
        return a.path.string() < b.path.string();
    });

    uintmax_t target = static_cast<uintmax_t>(std::max<int64_t>(0, (max_bytes * 9) / 10));
    int removed = 0;
    uintmax_t removed_bytes = 0;
    for (const auto& entry : entries) {
        if (total <= target) break;
        fs::remove(entry.path, ec);
        if (!ec) {
            total -= std::min(total, entry.size);
            removed_bytes += entry.size;
            ++removed;
        }
        ec.clear();
    }
    if (removed > 0 && verbose_log_enabled()) {
        std::cerr << "[Resamp] WORLD cache prune: removed=" << removed
                  << " bytes=" << removed_bytes
                  << " dir=" << cache_root.string() << '\n';
    }
}

static fs::path default_legacy_analysis_cache_root() {
    if (const char* local = std::getenv("LOCALAPPDATA"); local && *local) {
        return fs::path(local) / "Tansan-Sampler" / "analysis_cache";
    }
    return {};
}

static void prune_legacy_global_analysis_cache_once(const fs::path& active_cache_root, int64_t max_bytes) {
    static bool done = false;
    if (done) return;
    done = true;

    fs::path legacy = default_legacy_analysis_cache_root();
    if (legacy.empty()) return;

    std::error_code ec;
    fs::path legacy_abs = fs::weakly_canonical(legacy, ec);
    ec.clear();
    fs::path active_abs = fs::weakly_canonical(active_cache_root, ec);
    ec.clear();
    if (!legacy_abs.empty() && !active_abs.empty() && legacy_abs == active_abs) return;

    int64_t legacy_cap = std::min<int64_t>(max_bytes / 4, 256ll * 1024ll * 1024ll);
    legacy_cap = std::max<int64_t>(legacy_cap, 64ll * 1024ll * 1024ll);
    prune_analysis_cache_dir(legacy, legacy_cap);
}

static uint64_t fnv1a_append(uint64_t h, const void* data, size_t n) {
    const auto* p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < n; ++i) {
        h ^= static_cast<uint64_t>(p[i]);
        h *= 1099511628211ull;
    }
    return h;
}

static uint64_t fnv1a_u64(uint64_t h, uint64_t v) {
    return fnv1a_append(h, &v, sizeof(v));
}

static uint64_t fnv1a_i64(uint64_t h, int64_t v) {
    return fnv1a_append(h, &v, sizeof(v));
}

static std::string hex_u64(uint64_t v) {
    static const char* kHex = "0123456789abcdef";
    std::string out(16, '0');
    for (int i = 15; i >= 0; --i) {
        out[i] = kHex[v & 0xF];
        v >>= 4;
    }
    return out;
}

static double estimate_formant_peak_in_band(
    const std::vector<double>& spec,
    int                        fs,
    double                     lo_hz,
    double                     hi_hz,
    double                     fallback_hz)
{
    int spec_dim = static_cast<int>(spec.size());
    if (spec_dim < 5 || fs <= 0) return fallback_hz;
    double nyquist = fs * 0.5;
    int k0 = std::clamp(static_cast<int>(std::ceil(lo_hz / nyquist * (spec_dim - 1))), 1, spec_dim - 2);
    int k1 = std::clamp(static_cast<int>(std::floor(hi_hz / nyquist * (spec_dim - 1))), k0, spec_dim - 2);

    auto log_smooth = [&](int k) {
        double a = std::log(std::max(1.0e-12, spec[k - 1]));
        double b = std::log(std::max(1.0e-12, spec[k]));
        double c = std::log(std::max(1.0e-12, spec[k + 1]));
        return 0.25 * a + 0.50 * b + 0.25 * c;
    };

    int best_k = k0;
    double best_score = -1.0e300;
    bool found_local_peak = false;
    for (int k = k0; k <= k1; ++k) {
        double s0 = log_smooth(k - 1);
        double s1 = log_smooth(k);
        double s2 = log_smooth(k + 1);
        bool is_peak = (s1 >= s0 && s1 >= s2);
        if (!is_peak && found_local_peak) continue;
        if (is_peak && !found_local_peak) {
            found_local_peak = true;
            best_score = -1.0e300;
        }
        if (s1 > best_score) {
            best_score = s1;
            best_k = k;
        }
    }

    double peak_k = static_cast<double>(best_k);
    if (best_k > 1 && best_k < spec_dim - 2) {
        double y0 = log_smooth(best_k - 1);
        double y1 = log_smooth(best_k);
        double y2 = log_smooth(best_k + 1);
        double den = y0 - 2.0 * y1 + y2;
        if (std::fabs(den) > 1.0e-9) {
            double delta = 0.5 * (y0 - y2) / den;
            peak_k += std::clamp(delta, -0.45, 0.45);
        }
    }
    double hz = peak_k / static_cast<double>(spec_dim - 1) * nyquist;
    if (!std::isfinite(hz)) return fallback_hz;
    return std::clamp(hz, lo_hz, hi_hz);
}

static double formant_peak_prominence_score(
    const std::vector<double>& spec,
    int                        fs,
    double                     center_hz,
    double                     lo_hz,
    double                     hi_hz)
{
    int spec_dim = static_cast<int>(spec.size());
    if (spec_dim < 7 || fs <= 0 || center_hz <= 0.0) return 0.0;
    double nyquist = fs * 0.5;
    int k0 = std::clamp(static_cast<int>(std::ceil(lo_hz / nyquist * (spec_dim - 1))), 1, spec_dim - 2);
    int k1 = std::clamp(static_cast<int>(std::floor(hi_hz / nyquist * (spec_dim - 1))), k0, spec_dim - 2);
    int kc = std::clamp(static_cast<int>(std::round(center_hz / nyquist * (spec_dim - 1))), k0, k1);

    auto log_at = [&](int k) {
        k = std::clamp(k, 0, spec_dim - 1);
        return std::log(std::max(1.0e-12, spec[k]));
    };

    double peak = 0.25 * log_at(kc - 1) + 0.50 * log_at(kc) + 0.25 * log_at(kc + 1);
    int side_span = std::max(3, (k1 - k0 + 1) / 6);
    double side_sum = 0.0;
    int side_count = 0;
    for (int d = side_span; d <= side_span * 3; ++d) {
        int kl = kc - d;
        int kr = kc + d;
        if (kl >= k0) {
            side_sum += log_at(kl);
            ++side_count;
        }
        if (kr <= k1) {
            side_sum += log_at(kr);
            ++side_count;
        }
    }
    if (side_count <= 0) return 0.0;

    double prominence = peak - side_sum / static_cast<double>(side_count);
    return std::clamp((prominence - 0.08) / 0.56, 0.0, 1.0);
}

static double frame_log_spectral_energy(
    const std::vector<double>& spec,
    double                     lo_frac = 0.015,
    double                     hi_frac = 0.82)
{
    int spec_dim = static_cast<int>(spec.size());
    if (spec_dim <= 2) return -80.0;
    int k0 = std::clamp(static_cast<int>(std::round(lo_frac * (spec_dim - 1))), 1, spec_dim - 1);
    int k1 = std::clamp(static_cast<int>(std::round(hi_frac * (spec_dim - 1))), k0, spec_dim - 1);
    double sum = 0.0;
    int count = 0;
    for (int k = k0; k <= k1; ++k) {
        sum += std::max(1.0e-12, spec[k]);
        ++count;
    }
    double mean = sum / static_cast<double>(std::max(1, count));
    return 10.0 * std::log10(std::max(1.0e-12, mean));
}

static int find_last_energetic_frame(
    const WorldAnalysis& src,
    int                  first_frame,
    int                  fallback_frame,
    int                  max_frame = -1)
{
    int n_frames = src.n_frames;
    if (n_frames <= 0 || src.spectrogram.empty()) return fallback_frame;
    first_frame = std::clamp(first_frame, 0, n_frames - 1);
    fallback_frame = std::clamp(fallback_frame, first_frame, n_frames - 1);
    int last_frame = (max_frame >= 0)
        ? std::clamp(max_frame, first_frame, n_frames - 1)
        : (n_frames - 1);
    fallback_frame = std::min(fallback_frame, last_frame);

    std::vector<double> energies(last_frame + 1, -80.0);
    double ref = 0.0;
    int ref_count = 0;
    int ref_end = std::clamp(first_frame + 28, first_frame, last_frame);
    for (int fi = first_frame; fi <= ref_end; ++fi) {
        energies[fi] = frame_log_spectral_energy(src.spectrogram[fi]);
        ref += energies[fi];
        ++ref_count;
    }
    ref = (ref_count > 0) ? (ref / static_cast<double>(ref_count)) : -80.0;

    double max_energy = ref;
    for (int fi = ref_end + 1; fi <= last_frame; ++fi) {
        energies[fi] = frame_log_spectral_energy(src.spectrogram[fi]);
        max_energy = std::max(max_energy, energies[fi]);
    }
    ref = std::max(ref, max_energy - 10.0);
    double threshold = ref - 24.0;

    int last = fallback_frame;
    int run = 0;
    for (int fi = last_frame; fi >= first_frame; --fi) {
        double e = energies[fi];
        if (e > threshold) {
            ++run;
            if (run >= 2) {
                last = std::min(last_frame, fi + 1);
                break;
            }
        } else {
            run = 0;
        }
    }
    return std::clamp(last, first_frame, last_frame);
}

struct VocalizerTarget {
    double f1;
    double f2;
    double f3;
    double f4;
    bool nasal;
};

static VocalizerTarget vocalizer_target(int mode) {
    switch (mode) {
    case 1: return {850.0, 1150.0, 2500.0, 3700.0, false}; // 아: open/back
    case 2: return {520.0, 2050.0, 3000.0, 4300.0, false}; // 에: mid/front
    case 3: return {280.0, 2650.0, 3450.0, 5000.0, false}; // 이: close/front
    case 4: return {500.0,  820.0, 2350.0, 3350.0, false}; // 오: mid/round
    case 5: return {310.0,  620.0, 2150.0, 3200.0, false}; // 우: close/round
    case 6: return {640.0, 1350.0, 2450.0, 3650.0, false}; // 어: central
    case 7: return {280.0,  950.0, 2050.0, 3200.0, true};  // N
    default: return {0.0, 0.0, 0.0, 0.0, false};
    }
}

static void estimate_formant_peaks(WorldAnalysis& w) {
    static constexpr std::array<double, 4> kFallback = {700.0, 1500.0, 2600.0, 3800.0};
    if (w.n_frames <= 0 || w.spectrogram.empty()) return;
    w.formant_peaks.assign(w.n_frames, kFallback);
    w.formant_confidence.assign(w.n_frames, 0.0);
    for (int i = 0; i < w.n_frames; ++i) {
        const auto& spec = w.spectrogram[i];
        std::array<double, 4> p = {
            estimate_formant_peak_in_band(spec, w.fs, 180.0, 1100.0, kFallback[0]),
            estimate_formant_peak_in_band(spec, w.fs, 700.0, 2600.0, kFallback[1]),
            estimate_formant_peak_in_band(spec, w.fs, 1600.0, 3800.0, kFallback[2]),
            estimate_formant_peak_in_band(spec, w.fs, 2800.0, std::min(6200.0, w.fs * 0.46), kFallback[3]),
        };

        p[1] = std::max(p[1], p[0] + 260.0);
        p[2] = std::max(p[2], p[1] + 420.0);
        p[3] = std::max(p[3], p[2] + 520.0);
        p[3] = std::min(p[3], w.fs * 0.46);
        w.formant_peaks[i] = p;

        double voiced = (i < static_cast<int>(w.f0.size()) && w.f0[i] >= 55.0) ? 1.0 : 0.0;
        double ap_mean = 0.0;
        if (i < static_cast<int>(w.aperiodicity.size()) && !w.aperiodicity[i].empty()) {
            int spec_dim = static_cast<int>(w.aperiodicity[i].size());
            int k0 = std::clamp(static_cast<int>(0.08 * spec_dim), 0, spec_dim - 1);
            int k1 = std::clamp(static_cast<int>(0.55 * spec_dim), k0, spec_dim - 1);
            int count = 0;
            for (int k = k0; k <= k1; k += std::max(1, (k1 - k0) / 32)) {
                ap_mean += w.aperiodicity[i][k];
                ++count;
            }
            if (count > 0) ap_mean /= static_cast<double>(count);
        }
        double periodic_score = std::clamp(1.0 - 1.35 * ap_mean, 0.0, 1.0);
        double prom1 = formant_peak_prominence_score(spec, w.fs, p[0], 180.0, 1100.0);
        double prom2 = formant_peak_prominence_score(spec, w.fs, p[1], 700.0, 2600.0);
        double prom3 = formant_peak_prominence_score(spec, w.fs, p[2], 1600.0, 3800.0);
        double spacing12 = std::clamp((p[1] - p[0] - 220.0) / 780.0, 0.0, 1.0);
        double spacing23 = std::clamp((p[2] - p[1] - 340.0) / 1050.0, 0.0, 1.0);
        double spacing_score = std::min(spacing12, spacing23);
        double peak_score = 0.40 * prom1 + 0.35 * prom2 + 0.25 * prom3;
        double conf = (0.18 + 0.82 * voiced) * (0.26 + 0.74 * periodic_score) *
                      (0.22 + 0.78 * peak_score) * (0.55 + 0.45 * spacing_score);
        w.formant_confidence[i] = std::clamp(conf, 0.0, 1.0);
    }

    for (int pass = 0; pass < 2; ++pass) {
        auto prev = w.formant_peaks;
        auto prev_conf = w.formant_confidence;
        for (int i = 1; i + 1 < w.n_frames; ++i) {
            for (int j = 0; j < 4; ++j) {
                w.formant_peaks[i][j] = 0.25 * prev[i - 1][j] + 0.50 * prev[i][j] + 0.25 * prev[i + 1][j];
            }
            w.formant_confidence[i] = 0.25 * prev_conf[i - 1] + 0.50 * prev_conf[i] + 0.25 * prev_conf[i + 1];
        }
    }

    for (int i = 1; i + 1 < w.n_frames; ++i) {
        double jump = 0.0;
        for (int j = 0; j < 3; ++j) {
            double prev = std::max(1.0, w.formant_peaks[i - 1][j]);
            double next = std::max(1.0, w.formant_peaks[i + 1][j]);
            jump += std::fabs(std::log2(next / prev));
        }
        double continuity = std::clamp(1.0 - jump / 0.72, 0.0, 1.0);
        w.formant_confidence[i] *= (0.48 + 0.52 * continuity);
    }
}

static void fill_default_formant_peaks(WorldAnalysis& w) {
    w.formant_peaks.assign(w.n_frames, {700.0, 1500.0, 2600.0, 3800.0});
    w.formant_confidence.assign(w.n_frames, 1.0);
}

// ── 분석: DIO+StoneMask → CheapTrick → D4C ────────────────────────────────
//
// 핵심 전략 (피치 변조 시 envelope modulation 아티팩트 제거):
//   - anchor F0와 실제 F0가 미세하게 어긋날 때 생기는 harmonic 잔류를 회피.
//   - 분석 F0는 DIO 후 StoneMask로 refine해 프레임 jitter와 octave 흔들림 억제.
//   - refine 결과에 대해 최소한(3점 median) 정제로 spike만 추가 억제.
//   - q1 강화로 envelope 쪽 smoothing을 늘려 modulation 아티팩트 추가 억제.
WorldAnalysis world_analyze(
    const std::vector<float>& signal,
    int                       sample_rate,
    bool                      track_formants,
    double                    analysis_frame_period_ms)
{
    WorldAnalysis w;
    w.fs           = sample_rate;
    // 2.5ms: 기본 F0/envelope 시간 해상도. Fast 모드는 3.5ms로 분석 프레임 수를 줄인다.
    w.frame_period = std::clamp(analysis_frame_period_ms, 2.5, 4.5);

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

    if (track_formants) {
        estimate_formant_peaks(w);
    } else {
        fill_default_formant_peaks(w);
    }

    if (verbose_log_enabled()) {
        std::cerr << "[Resamp] WORLD analyzed: n_frames=" << f0_length
                  << " fft_size=" << w.fft_size
                  << " formants=" << (track_formants ? "tracked" : "default") << '\n';
    }
    return w;
}

static bool write_world_analysis_cache(const fs::path& path, const WorldAnalysis& w) {
    if (w.n_frames <= 0 || w.fft_size <= 0) return false;
    int spec_dim = w.fft_size / 2 + 1;
    if (spec_dim <= 0) return false;
    if (static_cast<int>(w.f0.size()) != w.n_frames) return false;
    if (static_cast<int>(w.temporal_positions.size()) != w.n_frames) return false;
    if (static_cast<int>(w.spectrogram.size()) != w.n_frames) return false;
    if (static_cast<int>(w.aperiodicity.size()) != w.n_frames) return false;
    if (static_cast<int>(w.formant_peaks.size()) != w.n_frames) return false;
    if (static_cast<int>(w.formant_confidence.size()) != w.n_frames) return false;

    fs::create_directories(path.parent_path());
    fs::path tmp = path;
    tmp += ".tmp";

    std::ofstream ofs(tmp, std::ios::binary | std::ios::trunc);
    if (!ofs) return false;

    const uint32_t magic = 0x41575352u; // "RSWA"
    const uint32_t version = 3u;
    const int32_t fs_i32 = static_cast<int32_t>(w.fs);
    const int32_t fft_i32 = static_cast<int32_t>(w.fft_size);
    const double frame_period = w.frame_period;
    const int32_t n_frames = static_cast<int32_t>(w.n_frames);
    const int32_t spec_i32 = static_cast<int32_t>(spec_dim);

    ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));
    ofs.write(reinterpret_cast<const char*>(&fs_i32), sizeof(fs_i32));
    ofs.write(reinterpret_cast<const char*>(&fft_i32), sizeof(fft_i32));
    ofs.write(reinterpret_cast<const char*>(&frame_period), sizeof(frame_period));
    ofs.write(reinterpret_cast<const char*>(&n_frames), sizeof(n_frames));
    ofs.write(reinterpret_cast<const char*>(&spec_i32), sizeof(spec_i32));

    ofs.write(reinterpret_cast<const char*>(w.f0.data()), sizeof(double) * w.f0.size());
    ofs.write(reinterpret_cast<const char*>(w.temporal_positions.data()), sizeof(double) * w.temporal_positions.size());
    for (int i = 0; i < w.n_frames; ++i) {
        if (static_cast<int>(w.spectrogram[i].size()) != spec_dim) return false;
        if (static_cast<int>(w.aperiodicity[i].size()) != spec_dim) return false;
        ofs.write(reinterpret_cast<const char*>(w.spectrogram[i].data()), sizeof(double) * spec_dim);
    }
    for (int i = 0; i < w.n_frames; ++i) {
        ofs.write(reinterpret_cast<const char*>(w.aperiodicity[i].data()), sizeof(double) * spec_dim);
    }
    for (int i = 0; i < w.n_frames; ++i) {
        ofs.write(reinterpret_cast<const char*>(w.formant_peaks[i].data()), sizeof(double) * 4);
    }
    ofs.write(reinterpret_cast<const char*>(w.formant_confidence.data()), sizeof(double) * w.formant_confidence.size());
    ofs.close();
    if (!ofs.good()) {
        std::error_code ec;
        fs::remove(tmp, ec);
        return false;
    }
    std::error_code ec;
    fs::rename(tmp, path, ec);
    if (ec) {
        fs::remove(path, ec);
        ec.clear();
        fs::rename(tmp, path, ec);
    }
    return !ec;
}

static bool read_world_analysis_cache(const fs::path& path, WorldAnalysis& out) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) return false;

    uint32_t magic = 0;
    uint32_t version = 0;
    int32_t fs_i32 = 0;
    int32_t fft_i32 = 0;
    double frame_period = 0.0;
    int32_t n_frames = 0;
    int32_t spec_i32 = 0;

    ifs.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    ifs.read(reinterpret_cast<char*>(&version), sizeof(version));
    ifs.read(reinterpret_cast<char*>(&fs_i32), sizeof(fs_i32));
    ifs.read(reinterpret_cast<char*>(&fft_i32), sizeof(fft_i32));
    ifs.read(reinterpret_cast<char*>(&frame_period), sizeof(frame_period));
    ifs.read(reinterpret_cast<char*>(&n_frames), sizeof(n_frames));
    ifs.read(reinterpret_cast<char*>(&spec_i32), sizeof(spec_i32));

    if (!ifs) return false;
    if (magic != 0x41575352u || version != 3u) return false;
    if (fs_i32 <= 0 || fft_i32 <= 0 || n_frames <= 0 || spec_i32 <= 1) return false;
    if (fft_i32 / 2 + 1 != spec_i32) return false;
    if (n_frames > 20000 || spec_i32 > 65536) return false;

    WorldAnalysis w;
    w.fs = fs_i32;
    w.fft_size = fft_i32;
    w.frame_period = frame_period;
    w.n_frames = n_frames;
    w.f0.assign(n_frames, 0.0);
    w.temporal_positions.assign(n_frames, 0.0);
    w.spectrogram.assign(n_frames, std::vector<double>(spec_i32, 0.0));
    w.aperiodicity.assign(n_frames, std::vector<double>(spec_i32, 0.0));
    w.formant_peaks.assign(n_frames, {700.0, 1500.0, 2600.0, 3800.0});
    w.formant_confidence.assign(n_frames, 0.0);

    ifs.read(reinterpret_cast<char*>(w.f0.data()), sizeof(double) * w.f0.size());
    ifs.read(reinterpret_cast<char*>(w.temporal_positions.data()), sizeof(double) * w.temporal_positions.size());
    for (int i = 0; i < n_frames; ++i) {
        ifs.read(reinterpret_cast<char*>(w.spectrogram[i].data()), sizeof(double) * spec_i32);
    }
    for (int i = 0; i < n_frames; ++i) {
        ifs.read(reinterpret_cast<char*>(w.aperiodicity[i].data()), sizeof(double) * spec_i32);
    }
    for (int i = 0; i < n_frames; ++i) {
        ifs.read(reinterpret_cast<char*>(w.formant_peaks[i].data()), sizeof(double) * 4);
    }
    ifs.read(reinterpret_cast<char*>(w.formant_confidence.data()), sizeof(double) * w.formant_confidence.size());
    if (!ifs.good()) return false;

    out = std::move(w);
    return true;
}

WorldAnalysis world_analyze_cached(
    const std::vector<float>& signal,
    int                       sample_rate,
    const std::string&        source_wav_path,
    int                       src_start_sample,
    int                       src_end_sample,
    bool                      track_formants,
    double                    analysis_frame_period_ms)
{
    if (!env_flag_enabled("RESAMP_ANALYSIS_CACHE", true)) {
        return world_analyze(signal, sample_rate, track_formants, analysis_frame_period_ms);
    }

    fs::path cache_root;
    if (const char* env_dir = std::getenv("RESAMP_CACHE_DIR"); env_dir && *env_dir) {
        cache_root = fs::path(env_dir);
    } else {
        fs::path voicebank_root = find_voicebank_root_for_cache(fs::path(source_wav_path));
        cache_root = voicebank_root / ".tansan-sampler" / "analysis_cache";
    }
    int64_t cache_max_bytes = analysis_cache_limit_bytes();
    prune_legacy_global_analysis_cache_once(cache_root, cache_max_bytes);

    uint64_t h = 1469598103934665603ull;
    std::string key_path = source_wav_path;
    std::transform(key_path.begin(), key_path.end(), key_path.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    h = fnv1a_append(h, key_path.data(), key_path.size());
    h = fnv1a_i64(h, static_cast<int64_t>(sample_rate));
    h = fnv1a_i64(h, static_cast<int64_t>(src_start_sample));
    h = fnv1a_i64(h, static_cast<int64_t>(src_end_sample));
    h = fnv1a_i64(h, track_formants ? 1 : 0);
    h = fnv1a_i64(h, static_cast<int64_t>(std::llround(analysis_frame_period_ms * 1000.0)));
    h = fnv1a_i64(h, static_cast<int64_t>(signal.size()));

    std::error_code ec;
    fs::path src_path(source_wav_path);
    if (fs::exists(src_path, ec)) {
        auto fsize = fs::file_size(src_path, ec);
        if (!ec) h = fnv1a_u64(h, static_cast<uint64_t>(fsize));
        ec.clear();
        auto wtime = fs::last_write_time(src_path, ec);
        if (!ec) {
            auto cnt = wtime.time_since_epoch().count();
            h = fnv1a_i64(h, static_cast<int64_t>(cnt));
        }
    }

    fs::path cache_file = cache_root / (hex_u64(h) + ".rswa");

    WorldAnalysis cached;
    if (read_world_analysis_cache(cache_file, cached)) {
        std::error_code touch_ec;
        fs::last_write_time(cache_file, fs::file_time_type::clock::now(), touch_ec);
        prune_analysis_cache_dir(cache_root, cache_max_bytes);
        if (verbose_log_enabled()) {
            std::cerr << "[Resamp] WORLD cache hit: " << cache_file.filename().string()
                      << " dir=" << cache_root.string() << '\n';
        }
        return cached;
    }

    auto analyzed = world_analyze(signal, sample_rate, track_formants, analysis_frame_period_ms);
    if (write_world_analysis_cache(cache_file, analyzed)) {
        prune_analysis_cache_dir(cache_root, cache_max_bytes);
        if (verbose_log_enabled()) {
            std::cerr << "[Resamp] WORLD cache store: " << cache_file.filename().string()
                      << " dir=" << cache_root.string()
                      << " max_mb=" << (cache_max_bytes / (1024 * 1024)) << '\n';
        }
    }
    return analyzed;
}

// ── 시간 매핑: 출력 시간(ms) → 소스 시간(ms) ────────────────────────────
// UTAU 자음/모음 처리:
//   - 자음 영역: velocity 비율로 시간 스케일
//   - 연결(transition) 구간: 1회 통과
//   - 안정 모음 구간: 루프
static void map_out_time_to_src(
    double out_time_ms,
    double source_origin_ms,
    double consonant_scale,
    double consonant_src_ms,
    double consonant_tgt_ms,
    double transition_src_len_ms,
    double transition_tgt_len_ms,
    double loop_start_ms,
    double loop_len_ms,
    double src_total_ms,
    int    loop_mode,
    double& src_time_ms,
    bool&   in_vowel_loop)
{
    if (out_time_ms <= consonant_tgt_ms) {
        src_time_ms = source_origin_ms + out_time_ms / consonant_scale;
        in_vowel_loop = false;
    } else {
        double vowel_time = out_time_ms - consonant_tgt_ms;

        // 연결 구간은 한 번만 통과
        if (transition_src_len_ms > 0.0 && vowel_time <= transition_tgt_len_ms) {
            double t = vowel_time / std::max(1.0e-6, transition_tgt_len_ms);
            t = std::clamp(t, 0.0, 1.0);
            t = t * t * (3.0 - 2.0 * t);
            src_time_ms = consonant_src_ms + transition_src_len_ms * t;
            in_vowel_loop = false;
            return;
        }

        // 이후는 안정 모음 구간만 루프.
        // loop_mode 3은 내부 자동 모드: 처음에는 stretch처럼 진행하고,
        // 소스 루프 끝을 넘는 과장 길이부터 mirrored loop로 왕복한다.
        if (loop_len_ms > 0.0) {
            double loop_time = vowel_time - transition_tgt_len_ms;
            double wrapped = std::fmod(loop_time, loop_len_ms);
            if (wrapped < 0.0) wrapped += loop_len_ms;
            bool auto_stretch_then_mirror = (loop_mode == 3);
            if (auto_stretch_then_mirror && loop_time <= loop_len_ms) {
                src_time_ms = loop_start_ms + std::clamp(loop_time, 0.0, loop_len_ms);
                in_vowel_loop = false;
                return;
            }
            if (loop_mode == 2 || auto_stretch_then_mirror) {
                double phase_time = auto_stretch_then_mirror
                    ? std::max(0.0, loop_time - loop_len_ms)
                    : loop_time;
                double cycle = std::fmod(loop_time, loop_len_ms * 2.0);
                if (auto_stretch_then_mirror) cycle = std::fmod(phase_time, loop_len_ms * 2.0);
                if (cycle < 0.0) cycle += loop_len_ms * 2.0;
                if (auto_stretch_then_mirror) {
                    wrapped = (cycle <= loop_len_ms) ? (loop_len_ms - cycle) : (cycle - loop_len_ms);
                } else {
                    wrapped = (cycle <= loop_len_ms) ? cycle : (2.0 * loop_len_ms - cycle);
                }
            }
            src_time_ms = loop_start_ms + wrapped;
            in_vowel_loop = true;
        } else {
            // 루프를 쓰지 않는 경우에는 tail을 시간 순서대로 계속 진행.
            // 길이가 부족하면 마지막 프레임 고정보다 tail 구간을 점진 압축해 거칠어짐을 줄인다.
            double tail_time = std::max(0.0, vowel_time - transition_tgt_len_ms);
            double tail_base = consonant_src_ms + transition_src_len_ms;
            double source_tail = std::max(0.0, src_total_ms - tail_base);
            if (source_tail > 1.0) {
                double guard_ms = std::clamp(source_tail * 0.45, 20.0, 180.0);
                double normal_until = std::max(0.0, source_tail - guard_ms);
                if (tail_time <= normal_until) {
                    src_time_ms = tail_base + tail_time;
                } else {
                    double u = tail_time - normal_until;
                    double tau = std::max(35.0, guard_ms * 0.55);
                    src_time_ms = src_total_ms - guard_ms * std::exp(-u / tau);
                }
            } else {
                src_time_ms = tail_base + tail_time;
            }
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

static std::array<double, 4> interpolate_formant_peaks(
    const WorldAnalysis& src,
    int                  fi0,
    int                  fi1,
    double               frac)
{
    std::array<double, 4> fallback = {720.0, 1580.0, 2820.0, 4100.0};
    if (static_cast<int>(src.formant_peaks.size()) != src.n_frames || src.n_frames <= 0) {
        return fallback;
    }
    fi0 = std::clamp(fi0, 0, src.n_frames - 1);
    fi1 = std::clamp(fi1, 0, src.n_frames - 1);
    frac = std::clamp(frac, 0.0, 1.0);
    std::array<double, 4> out = fallback;
    for (int j = 0; j < 4; ++j) {
        double a = src.formant_peaks[fi0][j];
        double b = src.formant_peaks[fi1][j];
        double v = a * (1.0 - frac) + b * frac;
        if (std::isfinite(v) && v > 40.0) out[j] = v;
    }
    out[0] = std::clamp(out[0], 150.0, 1400.0);
    out[1] = std::clamp(std::max(out[1], out[0] + 260.0), 520.0, 3200.0);
    out[2] = std::clamp(std::max(out[2], out[1] + 420.0), 1100.0, 4600.0);
    out[3] = std::clamp(std::max(out[3], out[2] + 520.0), 2200.0, src.fs * 0.46);
    return out;
}

static double interpolate_formant_confidence(
    const WorldAnalysis& src,
    int                  fi0,
    int                  fi1,
    double               frac)
{
    if (static_cast<int>(src.formant_confidence.size()) != src.n_frames || src.n_frames <= 0) {
        return 0.35;
    }
    fi0 = std::clamp(fi0, 0, src.n_frames - 1);
    fi1 = std::clamp(fi1, 0, src.n_frames - 1);
    frac = std::clamp(frac, 0.0, 1.0);
    double a = src.formant_confidence[fi0];
    double b = src.formant_confidence[fi1];
    double v = a * (1.0 - frac) + b * frac;
    if (!std::isfinite(v)) return 0.35;
    return std::clamp(v, 0.0, 1.0);
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

static double log_freq_bell(double hz, double center_hz, double width_oct) {
    center_hz = std::max(40.0, center_hz);
    width_oct = std::max(0.08, width_oct);
    double x = std::log2((hz + 120.0) / center_hz);
    return std::exp(-0.5 * (x * x) / (width_oct * width_oct));
}

static double hz_bell(double hz, double center_hz, double width_hz) {
    width_hz = std::max(1.0, width_hz);
    double x = (hz - center_hz) / width_hz;
    return std::exp(-0.5 * x * x);
}

static double hash_noise_signed(int i, int salt) {
    uint32_t x = static_cast<uint32_t>(i) * 747796405u + static_cast<uint32_t>(salt) * 2891336453u + 277803737u;
    x = ((x >> ((x >> 28) + 4)) ^ x) * 277803737u;
    x = (x >> 22) ^ x;
    return static_cast<double>(x) / static_cast<double>(UINT32_MAX) * 2.0 - 1.0;
}

// THROAT식 성도 플래그 전용 tube-response 근사.
// 실제 파형 필터 대신 5구간 관 단면적에서 얻은 공명/반공명 곡선을 WORLD envelope에 곱한다.
static double tract_tube_response_db(
    double hz,
    int    fs,
    double vtl_eff,
    double vtr_eff,
    double vtw_eff,
    double vc_drive,
    double nn_pos_eff,
    double nn_neg_eff,
    double mo_eff,
    double voiced_eff,
    bool   in_consonant,
    bool   in_transition)
{
    if (hz < 20.0 || fs <= 0) return 0.0;

    double vtl_pos = std::max(0.0, vtl_eff);
    double vtl_neg = std::max(0.0, -vtl_eff);
    double vtr_pos = std::max(0.0, vtr_eff);
    double vtr_neg = std::max(0.0, -vtr_eff);
    double vtw_pos = std::max(0.0, vtw_eff);
    double vtw_neg = std::max(0.0, -vtw_eff);
    double mo_open = std::max(0.0, mo_eff);
    double mo_close = std::max(0.0, -mo_eff);

    double region_gate = in_consonant ? 0.30 : (in_transition ? 0.62 : 1.0);
    double drive = (0.20 + 0.80 * voiced_eff) * region_gate;

    // glottis -> lower pharynx -> tongue/oral constriction -> front oral -> lips
    double a0 = std::clamp(0.88 + 0.10 * voiced_eff - 0.22 * vc_drive, 0.36, 1.40);
    double a1 = std::clamp(1.30 + 0.28 * vtl_pos - 0.16 * vtl_neg - 0.18 * vtr_pos + 0.12 * vtr_neg, 0.48, 2.25);
    double a2 = std::clamp(1.08 + 0.18 * vtr_pos - 0.20 * vtr_neg - 0.42 * vc_drive + 0.22 * mo_open - 0.12 * mo_close, 0.26, 2.10);
    double a3 = std::clamp(1.00 + 0.16 * vtw_neg - 0.14 * vtw_pos + 0.36 * mo_open - 0.28 * mo_close, 0.30, 2.25);
    double a4 = std::clamp(0.72 + 0.62 * mo_open - 0.34 * mo_close - 0.10 * vc_drive, 0.24, 2.10);

    auto refl = [](double left, double right) {
        double den = std::max(1.0e-6, left + right);
        return (right - left) / den;
    };
    double r01 = refl(a0, a1);
    double r12 = refl(a1, a2);
    double r23 = refl(a2, a3);
    double r34 = refl(a3, a4);

    // 길이가 길수록 기본 관 공명은 내려간다. Vtr은 앞/뒤 공명 이동만 추가한다.
    double length_cm = 17.0 * std::clamp(std::exp(0.22 * vtl_eff), 0.72, 1.32);
    double c_cm_s = 35000.0;
    double q = std::clamp(0.58 - 0.24 * vtw_pos + 0.32 * vtw_neg, 0.26, 1.08);
    double f1 = (1.0 * c_cm_s) / (4.0 * length_cm) * std::exp(0.10 * mo_open - 0.10 * mo_close - 0.04 * vtr_neg);
    double f2 = (3.0 * c_cm_s) / (4.0 * length_cm) * std::exp(0.18 * vtr_eff + 0.07 * mo_open - 0.13 * mo_close);
    double f3 = (5.0 * c_cm_s) / (4.0 * length_cm) * std::exp(0.24 * vtr_eff - 0.06 * mo_close);
    f1 = std::clamp(f1, 180.0, 1150.0);
    f2 = std::clamp(f2, 640.0, 4200.0);
    f3 = std::clamp(f3, 1200.0, 7600.0);

    double b1 = log_freq_bell(hz, f1, q * 1.08);
    double b2 = log_freq_bell(hz, f2, q);
    double b3 = log_freq_bell(hz, f3, q * 0.94);
    double lip = log_freq_bell(hz, std::clamp(1.24 * f3, 2800.0, 9200.0), 0.72 + 0.25 * vtw_neg);
    double low_puff = hz_bell(hz, 120.0, 110.0);
    double high = std::clamp((hz / (fs * 0.5) - 0.46) / 0.54, 0.0, 1.0);

    double mismatch = std::clamp(std::fabs(r01) + std::fabs(r12) + std::fabs(r23) + std::fabs(r34), 0.0, 1.35);
    double db = 0.0;
    db += mismatch * (2.8 * std::fabs(r01) * b1 + 3.6 * std::fabs(r12) * b2 +
                      3.8 * std::fabs(r23) * b3 + 2.0 * std::fabs(r34) * lip);
    db += vtl_pos * (1.4 * b1 + 2.1 * b2 + 2.6 * b3 - 1.1 * high);
    db -= vtl_neg * (1.0 * b1 - 1.8 * b2 - 2.2 * b3);
    db += vtw_pos * (2.8 * (b2 + b3) - 2.1 * log_freq_bell(hz, std::sqrt(f2 * f3), 1.10));
    db += vtw_neg * (-1.8 * (b2 + b3) + 1.6 * log_freq_bell(hz, std::sqrt(f2 * f3), 1.30));
    db += vc_drive * (4.4 * log_freq_bell(hz, std::clamp(0.70 * f2 + 0.36 * f3, 1600.0, 5200.0), 0.50)
                    - 2.6 * log_freq_bell(hz, std::clamp(0.52 * f2, 520.0, 2100.0), 0.70)
                    - 1.1 * low_puff);
    db += nn_pos_eff * (2.8 * hz_bell(hz, 320.0, 170.0) + 3.4 * log_freq_bell(hz, 0.62 * f2, 0.52)
                      - 3.5 * log_freq_bell(hz, 0.42 * f2, 0.42) - 2.6 * log_freq_bell(hz, 0.76 * f3, 0.46));
    db += nn_neg_eff * (-2.0 * hz_bell(hz, 320.0, 170.0) - 2.8 * log_freq_bell(hz, 0.62 * f2, 0.52)
                      + 1.8 * b2 + 1.4 * b3);
    db += mo_open * (1.6 * b1 + 1.8 * b2 + 1.2 * high);
    db -= mo_close * (1.4 * b2 + 2.1 * b3 + 1.6 * high);

    db *= drive;
    return std::clamp(db, -11.5, 11.5);
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
    auto smoothstep01_time = [](double x) {
        x = std::clamp(x, 0.0, 1.0);
        return x * x * (3.0 - 2.0 * x);
    };
    double timing_short_note_amt = smoothstep01_time((360.0 - out_total_ms) / 240.0);
    double timing_ultra_short_amt = smoothstep01_time((190.0 - out_total_ms) / 120.0);
    // 합성 프레임 주기:
    // 변조가 큰 노트는 촘촘하게(품질 우선), 평탄/저변조 노트는 조금 넓혀 속도 개선.
    bool has_f0_mod = false;
    double frame_period = 0.5;
    bool fast_flags_mode = env_flag_enabled("RESAMP_FAST_FLAGS", true);
    // 합성 프레임 주기를 넓히면 일부 음원에서 무플래그 상태도 치지직거릴 수 있다.
    // 기본 렌더는 품질 우선으로 기존 주기를 사용하고, 속도 실험은 명시적으로 켠다.
    bool fast_timing_mode = (sp.fast_mode > 0) || env_flag_enabled("RESAMP_FAST_TIMING", false);
    int spec_dim       = src.fft_size / 2 + 1;
    std::vector<double> fn_lut(spec_dim, 0.0);
    std::vector<double> hz_lut(spec_dim, 0.0);
    {
        double nyquist = fs * 0.5;
        int den = std::max(1, spec_dim - 1);
        for (int k = 0; k < spec_dim; ++k) {
            double fn = static_cast<double>(k) / den;
            fn_lut[k] = fn;
            hz_lut[k] = fn * nyquist;
        }
    }
    SpectralCurveLut curve_lut(fn_lut, hz_lut);
    double src_total_ms = src.n_frames * anal_period;
    double source_origin_ms = std::clamp(params.source_origin_ms, 0.0, std::max(0.0, src_total_ms - anal_period));
    // UTAU velocity 관례:
    // 값이 클수록 자음이 더 빠르게(짧게) 지나가야 하므로 역비율 사용.
    // v=100 -> 1.0, v=200 -> 0.5, v=50 -> 2.0
    double vel = static_cast<double>(std::max(1, params.velocity));
    double consonant_scale = std::clamp(100.0 / vel, 0.25, 4.0);
    double consonant_src_dur_ms = std::clamp(params.consonant_ms, 0.0,
                                             std::max(0.0, src_total_ms - source_origin_ms));
    double consonant_src_ms = std::clamp(source_origin_ms + consonant_src_dur_ms, 0.0, src_total_ms);
    double consonant_tgt_ms = consonant_src_dur_ms * consonant_scale;
    if (consonant_src_dur_ms > 1.0 && timing_short_note_amt > 0.001) {
        // 짧은 노트에서 fixed consonant가 출력 전체를 차지하면 tail/fry/breath와
        // 안정 모음부가 사라진다. 소스 fixed consonant 끝까지는 도달하되,
        // 타겟 시간만 압축해 최소 모음 공간을 확보한다.
        double min_vowel_room_ms = std::clamp(out_total_ms * (0.24 + 0.22 * timing_short_note_amt),
                                              14.0, 92.0);
        min_vowel_room_ms = std::min(min_vowel_room_ms, std::max(0.0, out_total_ms - 6.0));
        double max_consonant_tgt_ms = std::max(4.0, out_total_ms - min_vowel_room_ms);
        if (consonant_tgt_ms > max_consonant_tgt_ms) {
            consonant_tgt_ms = max_consonant_tgt_ms;
            consonant_scale = std::clamp(consonant_tgt_ms / consonant_src_dur_ms, 0.08, 4.0);
        }
    }

    // 재생 범위는 main.cpp에서 alias offset..cutoff로 잘라 들어온다.
    // 스트레치/루프 가능 범위는 oto의 fixed consonant end..cutoff start 전체를 따른다.
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

    int loop_start_fi = std::clamp(start_fi, 0, n_frames - 2);
    bool vowel_like_alias = consonant_src_ms <= 12.0;
    if (vowel_like_alias && n_frames > 4) {
        // fixed consonant가 0에 가깝게 잡힌 alias는 loop 시작점이 offset과 같아져
        // vowel onset/VCV 앞부분까지 반복될 수 있다. 이런 경우에는 초입을 1회 통과
        // transition으로 남기고, loop는 조금 안정된 뒤에서 시작한다.
        double onset_guard_ms = std::clamp(src_total_ms * 0.16, 18.0, 52.0);
        int guarded_start_fi = std::clamp(
            start_fi + static_cast<int>(std::ceil(onset_guard_ms / anal_period)),
            start_fi,
            n_frames - 2);
        bool has_voiced_after_guard = false;
        for (int fi = guarded_start_fi + 1; fi < n_frames; ++fi) {
            if (src_voicing[fi] >= 0.5) {
                has_voiced_after_guard = true;
                break;
            }
        }
        if (guarded_start_fi > loop_start_fi && has_voiced_after_guard) {
            loop_start_fi = guarded_start_fi;
        }
    }

    // ── 루프 끝 결정: 마지막 유성 프레임까지만 ─────────────────────────
    // [근본 원인 수정]
    // 기존: loop_end_fi = n_frames - 1 → 소스 전체(데케이/릴리즈 포함)가 루프됨.
    //
    // oto.ini의 고정영역 끝(loop_start_fi) 이후에서
    // 마지막으로 유성(F0 ≥ 55Hz)인 프레임을 역방향으로 탐색해
    // 루프 끝을 해당 프레임으로 제한한다.
    // 이 프레임 이후는 모음 데케이/릴리즈(무성 테일)로 간주하며 루프에서 제외한다.
    //
    // 예외: 소스가 완전 무성(마찰음 단독 등)이면 폴백으로 n_frames-1 사용.
    int loop_end_fi;
    {
        int last_voiced = -1;
        for (int fi = n_frames - 1; fi > loop_start_fi; --fi) {
            if (src_voicing[fi] >= 0.5) {
                last_voiced = fi;
                break;
            }
        }
        if (last_voiced > loop_start_fi) {
            loop_end_fi = last_voiced;
        } else {
            // 고정영역 이후에 유성 구간이 없는 경우 (마찰음 등) → 전체 범위 폴백
            loop_end_fi = n_frames - 1;
        }

        // Whisper/숨 섞인 단독 모음은 DIO/StoneMask가 후반부 F0를 무성으로
        // 잘못 끊는 경우가 있다. 고정자음이 사실상 없는 alias에서는 F0 끝점만
        // 믿지 말고 스펙트럼 에너지로 아직 유지되는 모음 구간을 보조 판정한다.
        double voiced_span_ms = std::max(0.0, (loop_end_fi - loop_start_fi) * anal_period);
        bool suspicious_short_voicing =
            vowel_like_alias && src_total_ms >= 180.0 &&
            voiced_span_ms < std::min(220.0, src_total_ms * 0.42);
        if (suspicious_short_voicing) {
            // Energy fallback must not scan the whole loose VCV tail. Otherwise a later
            // syllable inside offset..cutoff can be mistaken for the same sustained vowel.
            int energy_horizon = std::clamp(
                loop_start_fi + static_cast<int>(std::ceil(320.0 / anal_period)),
                loop_start_fi + 1,
                n_frames - 1);
            int energy_end = find_last_energetic_frame(src, loop_start_fi,
                                                       loop_end_fi, energy_horizon);
            if (energy_end > loop_end_fi) loop_end_fi = energy_end;
        }
        loop_end_fi = std::clamp(loop_end_fi, loop_start_fi + 1, n_frames - 1);
    }

    double loop_start_ms = loop_start_fi * anal_period;
    double loop_end_ms   = loop_end_fi * anal_period;
    double loop_len_ms   = std::max(0.0, loop_end_ms - loop_start_ms);
    bool has_vowel_loop  = loop_len_ms > (2.0 * anal_period);
    double transition_src_len_ms = std::max(0.0, loop_start_ms - consonant_src_ms);

    // [버그 수정: VCV 연속음 및 느슨한 컷오프 방어]
    // oto.ini의 컷오프가 다음 음절까지 포함하도록 멀리 설정된 경우(VCV에서 흔함),
    // 루프 가능 구간이 매우 길어져(예: 2초) "루프 없이 1:1로 재생해도 되겠다"고 착각하게 됩니다.
    // 그 결과 긴 노트를 재생할 때 다음 음절(뒷부분 음성)이 그대로 노출되는 치명적 오류가 발생했습니다.
    // 이를 방지하기 위해 루프 가용 길이를 절대적으로 제한하여,
    // 노트가 길어지면 항상 루프/미러링이 발동되도록 강제합니다.
    if (has_vowel_loop && loop_len_ms > 1.0) {
        constexpr double stable_loop_cap_ms = 320.0;
        double floor_ms = std::max(loop_len_ms * 0.25, 40.0);
        floor_ms = std::min(floor_ms, stable_loop_cap_ms);
        double stable_loop_ms = std::clamp(loop_len_ms * 0.44, floor_ms, stable_loop_cap_ms);
        if (loop_len_ms > stable_loop_ms + anal_period) {
            loop_len_ms = stable_loop_ms;
            loop_end_ms = loop_start_ms + loop_len_ms;
            loop_end_fi = std::clamp(static_cast<int>(std::round(loop_end_ms / anal_period)),
                                     loop_start_fi + 1, n_frames - 1);
            loop_end_ms = loop_end_fi * anal_period;
            loop_len_ms = std::max(anal_period, loop_end_ms - loop_start_ms);
        }
    }

    // ── 유성 끝 보존 ────────────────────────────────────────────────────
    // 위에서 안전하게 제한된 loop_end_ms를 src_voiced_end_ms로 저장합니다.
    // 이 값이 map_out_time_to_src의 한계선이 되므로, 절대 다음 음절로 넘어가지 않습니다.
    double src_voiced_end_ms = loop_end_ms; // has_vowel_loop=false면 아래에서 src_total_ms로 폴백

    // 루프가 유효하지 않으면 one-pass tail로 강제.
    if (!has_vowel_loop) {
        loop_start_ms = std::clamp(consonant_src_ms, 0.0, src_total_ms);
        loop_end_ms   = src_total_ms;
        loop_len_ms   = 0.0;
        transition_src_len_ms = 0.0;
        loop_start_fi = std::clamp(static_cast<int>(std::round(loop_start_ms / anal_period)), 0, n_frames - 1);
        loop_end_fi   = loop_start_fi;
        // 완전 무성 소스(마찰음 등)는 src_total_ms 전체를 허용
        src_voiced_end_ms = src_total_ms;
    }

    // 출력 길이가 소스의 자음 이후 유성 길이보다 짧거나 같으면 루프가 필요 없다.
    double required_after_consonant_ms = std::max(0.0, out_total_ms - consonant_tgt_ms);
    double available_after_consonant_ms = std::max(0.0, src_voiced_end_ms - consonant_src_ms);
    bool no_loop_needed = (required_after_consonant_ms <= (available_after_consonant_ms + 1.0));
    double stretch_overrun_ms = required_after_consonant_ms - available_after_consonant_ms;
    double long_loop_stress = std::clamp(stretch_overrun_ms / std::max(120.0, available_after_consonant_ms), 0.0, 1.0);
    double auto_loop_margin_ms = std::clamp(available_after_consonant_ms * 0.18, 28.0, 180.0);
    bool auto_stretch_mirror_loop =
        (sp.loop_mode == 0 && has_vowel_loop && stretch_overrun_ms > auto_loop_margin_ms);
    bool micro_alias_stretch = false;

    if (has_vowel_loop && sp.loop_mode != 2) {
        double micro_limit_ms = std::clamp(104.0 + 0.25 * consonant_src_dur_ms, 104.0, 124.0);
        bool tiny_loop = loop_len_ms <= micro_limit_ms;
        bool tiny_available = available_after_consonant_ms <= std::max(128.0, micro_limit_ms + 28.0);
        bool needs_sustain = required_after_consonant_ms > std::max(24.0, available_after_consonant_ms * 0.78);
        micro_alias_stretch = tiny_loop && tiny_available && needs_sustain;
        if (micro_alias_stretch) {
            // 아주 짧은 loop 구간을 반복하면 같은 envelope 조각이 들쭉날쭉하게 반복된다.
            // 이 경우에는 loop를 끄고, 자음 뒤의 짧은 유성 구간을 one-pass stretch로 늘린다.
            // src_voiced_end_ms는 유지해서 cutoff 뒤나 다음 발음으로 넘어가지 않게 한다.
            has_vowel_loop = false;
            loop_len_ms = 0.0;
            loop_start_ms = std::clamp(consonant_src_ms, 0.0, src_total_ms);
            loop_end_ms = loop_start_ms;
            transition_src_len_ms = 0.0;
            loop_start_fi = std::clamp(static_cast<int>(std::round(loop_start_ms / anal_period)), 0, n_frames - 1);
            loop_end_fi = loop_start_fi;
        }
    }
    
    // extreme_length_loop 로직은 위로 이동되어 통합됨

    if (no_loop_needed) {
        has_vowel_loop = false;
        loop_len_ms = 0.0;
        loop_start_ms = std::clamp(consonant_src_ms, 0.0, src_total_ms);
        loop_end_ms = loop_start_ms;
        transition_src_len_ms = 0.0;
        loop_start_fi = std::clamp(static_cast<int>(std::round(loop_start_ms / anal_period)), 0, n_frames - 1);
        loop_end_fi = loop_start_fi;
    }
    int effective_loop_mode = sp.loop_mode;
    double short_stretch_first_amt = smoothstep01_time((660.0 - out_total_ms) / 360.0);
    bool auto_short_stretch_first =
        (sp.loop_mode == 1 && has_vowel_loop && !no_loop_needed &&
         short_stretch_first_amt > 0.02 &&
         required_after_consonant_ms > std::max(12.0, loop_len_ms * 0.35) &&
         required_after_consonant_ms <= std::max(available_after_consonant_ms + auto_loop_margin_ms,
                                                 loop_len_ms * (2.15 + 0.95 * short_stretch_first_amt)));
    bool auto_natural_mirror_loop =
        (sp.loop_mode == 1 && has_vowel_loop && !no_loop_needed && !auto_short_stretch_first &&
         long_loop_stress > 0.30 &&
         required_after_consonant_ms > std::max(available_after_consonant_ms + auto_loop_margin_ms,
                                                loop_len_ms * 1.65));
    if (auto_stretch_mirror_loop || auto_short_stretch_first || auto_natural_mirror_loop) {
        effective_loop_mode = 3; // internal: one-pass stretch, then mirrored loop on overrun
    }
    if (sp.loop_mode == 0 && !auto_stretch_mirror_loop) {
        has_vowel_loop = false;
        loop_len_ms = 0.0;
        loop_start_ms = std::clamp(consonant_src_ms, 0.0, src_total_ms);
        loop_end_ms = loop_start_ms;
        transition_src_len_ms = 0.0;
        loop_start_fi = std::clamp(static_cast<int>(std::round(loop_start_ms / anal_period)), 0, n_frames - 1);
        loop_end_fi = loop_start_fi;
    }
    double loop_wander_amt = 0.0;
    if ((auto_natural_mirror_loop || auto_short_stretch_first) && has_vowel_loop && loop_len_ms > 70.0) {
        loop_wander_amt = long_loop_stress * smoothstep01_time((loop_len_ms - 70.0) / 130.0);
        if (auto_short_stretch_first && !auto_natural_mirror_loop) {
            loop_wander_amt *= 0.45 * short_stretch_first_amt;
        }
    }

    double loop_endpoint_match_amt = 0.0;
    double loop_endpoint_width_ms = 0.0;
    double loop_start_gain_db = 0.0;
    double loop_end_gain_db = 0.0;
    double loop_endpoint_sustain_amt = 0.0;
    std::vector<double> loop_mid_spec;
    std::vector<double> loop_mid_ap;
    double endpoint_limit_ms =
        220.0 + 520.0 * smoothstep01_time((long_loop_stress - 0.18) / 0.62);
    if (has_vowel_loop && loop_start_fi < loop_end_fi &&
        loop_len_ms > (2.0 * anal_period) && loop_len_ms <= endpoint_limit_ms) {
        double e_start = frame_log_spectral_energy(src.spectrogram[loop_start_fi]);
        double e_end = frame_log_spectral_energy(src.spectrogram[loop_end_fi]);
        double e_mid = 0.5 * (e_start + e_end);
        double energy_diff = std::fabs(e_end - e_start);

        double f0_diff_cents = 0.0;
        double f0_start = src.f0[loop_start_fi];
        double f0_end = src.f0[loop_end_fi];
        if (f0_start >= 55.0 && f0_end >= 55.0) {
            f0_diff_cents = std::fabs(1200.0 * std::log2(f0_end / f0_start));
        }

        double short_loop_amt = smoothstep01_time((220.0 - loop_len_ms) / 160.0);
        double sustained_loop_amt =
            smoothstep01_time((long_loop_stress - 0.18) / 0.48) *
            smoothstep01_time((endpoint_limit_ms - loop_len_ms) / 220.0);
        loop_endpoint_sustain_amt = sustained_loop_amt;
        double energy_risk = std::clamp((energy_diff - 1.6) / 7.2, 0.0, 1.0);
        double f0_risk = std::clamp((f0_diff_cents - 28.0) / 155.0, 0.0, 1.0);
        double mismatch_risk = std::max(std::max(energy_risk, f0_risk), 0.55 * sustained_loop_amt);
        loop_endpoint_match_amt = std::clamp(
            std::max(short_loop_amt, 0.95 * sustained_loop_amt) * mismatch_risk,
            0.0,
            0.88);

        if (loop_endpoint_match_amt > 1.0e-4) {
            double width_ratio = (short_loop_amt > sustained_loop_amt)
                ? 0.34
                : (0.22 + 0.10 * long_loop_stress);
            double width_cap = 32.0 + 46.0 * sustained_loop_amt;
            loop_endpoint_width_ms = std::clamp(loop_len_ms * width_ratio, 2.0 * anal_period, width_cap);
            loop_endpoint_width_ms = std::min(loop_endpoint_width_ms, std::max(anal_period, loop_len_ms * 0.48));
            loop_start_gain_db = std::clamp((e_mid - e_start) * loop_endpoint_match_amt, -6.0, 3.0);
            loop_end_gain_db = std::clamp((e_mid - e_end) * loop_endpoint_match_amt, -6.0, 3.0);

            loop_mid_spec.assign(spec_dim, 0.0);
            loop_mid_ap.assign(spec_dim, 0.0);
            const auto& ss = src.spectrogram[loop_start_fi];
            const auto& se = src.spectrogram[loop_end_fi];
            const auto& as = src.aperiodicity[loop_start_fi];
            const auto& ae = src.aperiodicity[loop_end_fi];
            for (int k = 0; k < spec_dim; ++k) {
                loop_mid_spec[k] = std::sqrt(std::max(1.0e-12, ss[k]) * std::max(1.0e-12, se[k]));
                loop_mid_ap[k] = 0.5 * (as[k] + ae[k]);
            }
        }
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

    double transition_pitch_span_cents = 0.0;
    if (timing_short_note_amt > 0.001) {
        double local_min_f0 = 1.0e18;
        double local_max_f0 = 0.0;
        for (double v : target_f0_per_sample) {
            if (v >= 50.0) {
                local_min_f0 = std::min(local_min_f0, v);
                local_max_f0 = std::max(local_max_f0, v);
            }
        }
        if (local_min_f0 < 1.0e17 && local_max_f0 > local_min_f0) {
            transition_pitch_span_cents = 1200.0 * std::log2(local_max_f0 / local_min_f0);
        }
    }
    double short_pitch_move_amt =
        timing_short_note_amt * smoothstep01_time((transition_pitch_span_cents - 22.0) / 110.0);

    // 연결부는 너무 짧으면 끊김/툭툭거림이 생길 수 있어
    // 과도한 단축은 피하고, 무성 연결부에서는 충분히 길게 유지.
    double transition_tgt_len_ms = 0.0;
    if (transition_src_len_ms > 0.0) {
        double scale  = (transition_voiced_ratio < 0.45) ? 1.10 : 0.98;
        double min_ms = (transition_voiced_ratio < 0.45) ? 18.0 : 10.0;
        double max_ms = (transition_voiced_ratio < 0.45) ? 70.0 : 54.0;
        if (short_pitch_move_amt > 0.001) {
            double q = short_pitch_move_amt;
            scale *= (1.0 - 0.38 * q);
            min_ms *= (1.0 - 0.52 * q);
            max_ms *= (1.0 - 0.58 * q);
            min_ms = std::max((transition_voiced_ratio < 0.45) ? 8.0 : 4.0, min_ms);
            max_ms = std::max(min_ms + 2.0, max_ms);
        }
        transition_tgt_len_ms = std::clamp(transition_src_len_ms * scale, min_ms, max_ms);
        if (timing_short_note_amt > 0.001) {
            double remaining_ms = std::max(0.0, out_total_ms - consonant_tgt_ms);
            double cap_ratio = 0.34 + 0.12 * timing_ultra_short_amt;
            cap_ratio *= (1.0 - 0.58 * short_pitch_move_amt);
            double short_cap_ms = std::max(3.0, remaining_ms * std::clamp(cap_ratio, 0.14, 0.46));
            transition_tgt_len_ms = std::min(transition_tgt_len_ms, short_cap_ms);
        }
    }
    double articulation_ratio = (out_total_ms > 1.0)
        ? std::clamp((consonant_tgt_ms + transition_tgt_len_ms) / out_total_ms, 0.0, 1.0)
        : 1.0;
    double dense_articulation_amt = std::max(
        timing_short_note_amt,
        smoothstep01_time((articulation_ratio - 0.36) / 0.34));
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
    has_f0_mod = (voiced_span_cents >= 8.0 ||
                  sp.tremolo > 0 ||
                  sp.voiced_growl > 0 ||
                  sp.fry_head > 0 ||
                  sp.fry_tail > 0);
    bool flat_target_f0 = (voiced_cnt > 0 && voiced_span_cents <= 3.0);
    bool dynamic_texture_flags =
        sp.growl > 0 || sp.end_breath > 0 || sp.attack != 0 ||
        sp.noise_level != 0 || sp.noise_color != 0;
    bool timing_sensitive_note =
        timing_short_note_amt > 0.04 ||
        transition_tgt_len_ms > 18.0 ||
        (consonant_tgt_ms > 42.0 && out_total_ms < 480.0);
    if (has_f0_mod) {
        frame_period = fast_timing_mode ? 0.70 : 0.50; // 피치/프라이/트레몰로/유성 그로울은 추종성 우선
    } else if (!fast_timing_mode) {
        frame_period = flat_target_f0 ? 0.80 : 0.65; // 기존 품질 기준
    } else if (flat_target_f0) {
        frame_period = (timing_sensitive_note || dynamic_texture_flags) ? 0.80 : 0.95;
    } else {
        frame_period = (timing_sensitive_note || dynamic_texture_flags) ? 0.65 : 0.74;
    }

    double seam_ms = 0.0;
    if (has_vowel_loop) {
        // loop 경계 전후에서 end->start 파라미터를 부드럽게 잇는 크로스페이드 폭.
        double seam_cap_ms = 34.0 + 24.0 * long_loop_stress;
        seam_ms = std::clamp(loop_len_ms * (0.18 + 0.11 * long_loop_stress), 6.0, seam_cap_ms);
        seam_ms = std::min(seam_ms, std::max(0.0, loop_len_ms * 0.42));
        if (auto_stretch_mirror_loop) seam_ms = std::max(seam_ms, 12.0);
        seam_ms *= std::clamp(1.0 - 0.38 * dense_articulation_amt, 0.52, 1.0);
    }

    // 안정화 모드:
    // 음색이 노트마다 랜덤하게 꺼지는 현상을 막기 위해
    // transition voiced ratio 의존을 줄이고 보수적 상수 혼합으로 고정.
    bool use_stable_vowel_env = has_vowel_loop || (flat_target_f0 && dense_articulation_amt < 0.64);
    bool has_consonant_head = (consonant_src_ms >= 1.0);
    double anchor_mix_cap = flat_target_f0 ? 0.60 : (has_f0_mod ? 0.20 : (has_consonant_head ? 0.28 : 0.46));
    if (has_consonant_head) anchor_mix_cap *= 0.75;
    if (no_loop_needed) anchor_mix_cap *= 0.80;
    if (auto_stretch_mirror_loop || auto_short_stretch_first || auto_natural_mirror_loop)
        anchor_mix_cap *= (1.0 - 0.55 * long_loop_stress);
    else if (has_vowel_loop)
        anchor_mix_cap *= (1.0 - 0.18 * long_loop_stress);
    // Cs: 높을수록 연결 안정성 강화(과도한 변화 억제)
    double cs_local = std::clamp((sp.consonant_stability - 50) / 50.0, -1.0, 1.0);
    if (cs_local >= 0.0) anchor_mix_cap *= (1.0 + 0.42 * cs_local);
    else                 anchor_mix_cap *= (1.0 + 0.24 * cs_local);
    anchor_mix_cap *= std::clamp(1.0 - 0.56 * dense_articulation_amt, 0.30, 1.0);
    if (auto_stretch_mirror_loop || auto_short_stretch_first || auto_natural_mirror_loop) {
        anchor_mix_cap = std::min(anchor_mix_cap, 0.14 + 0.06 * (1.0 - long_loop_stress));
    }
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
    double bi_raw = std::clamp((sp.brightness - 50) / 50.0, -1.0, 1.0);
    double bi_abs = std::fabs(bi_raw);
    double bi_knee = std::clamp((bi_abs - 0.40) / 0.60, 0.0, 1.0);
    double bi_eff = (bi_raw >= 0.0) ? std::pow(bi_abs, 0.82) : -std::pow(bi_abs, 0.82);
    bi_eff *= (1.0 + 0.55 * bi_knee * bi_knee);
    double brightness_tilt_db = bi_eff * 22.0;
    // Hu: +값 = husky(저중역 질감↑), -값 = brighter(상부 명료도↑) — synth_params 스펙과 일치
    // [버그 수정] 기존에 -sp.husky_tone으로 부호를 반전시켜 방향이 완전히 반대였음. 수정.
    double hu = std::clamp(sp.husky_tone / 100.0, -1.0, 1.0);
    double hu_eff = (hu >= 0.0) ? std::pow(hu, 0.78) : -std::pow(-hu, 0.78);
    double mo = std::clamp(sp.mouth_open / 100.0, -1.0, 1.0);
    double mo_eff = (mo >= 0.0) ? std::pow(mo, 0.78) : -std::pow(-mo, 0.78);
    double mo_pos_ctrl = std::max(0.0, mo_eff);
    double mo_neg_ctrl = std::max(0.0, -mo_eff);
    // Mo 전용 포먼트 이동:
    // +Mo는 F1/F2/F3를 위로, -Mo는 아래로 미는 방향.
    // blend는 극단값에서도 과변형을 막기 위해 상한을 둔다.
    // -Mo는 과도한 전대역 하향(먹먹함)보다 발음 중심 붕괴(웅얼)로 들리도록
    // 주파수축 워프는 완화하고, 대역별 필터에서 articulation 억제를 강화한다.
    double mo_formant_shift = +0.11 * mo_pos_ctrl - 0.14 * mo_neg_ctrl;
    double mo_formant_ratio = std::clamp(std::exp(mo_formant_shift), 0.78, 1.18);
    double mo_formant_blend = (std::fabs(mo_eff) > 0.01)
        ? std::clamp(0.08 + 0.32 * mo_pos_ctrl + 0.50 * mo_neg_ctrl, 0.0, 0.62)
        : 0.0;
    double growl_amt = std::pow(std::clamp(sp.growl / 100.0, 0.0, 1.0), 0.60);
    double voiced_growl_amt = std::pow(std::clamp(sp.voiced_growl / 100.0, 0.0, 1.0), 0.68);
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
    double global_hi_ap_trim = 0.090;

    // ── Tension: 발성 강도/이완 근사 (체감 강화) ───────────────────────
    // +값: 더 pressed/firm (주기성↑, 존재감↑, breathiness↓)
    // -값: 더 relaxed/breathy (주기성↓, 거칠기/숨소리↑, 존재감↓)
    double tension = std::clamp(sp.tension / 100.0, -1.0, 1.0);
    double tension_pos_raw = std::max(0.0, tension);
    double tension_neg_raw = std::max(0.0, -tension);
    double tension_pos = std::pow(tension_pos_raw, 0.70);
    // Tn+는 60 이상에서 고강도 구간으로 가속 (high-knee).
    double tension_hi_knee = std::clamp((tension_pos_raw - 0.60) / 0.40, 0.0, 1.0);
    tension_pos += (1.0 - tension_pos) * (0.28 * std::pow(tension_hi_knee, 1.12));
    // Tn-는 전 구간 감도를 상향.
    double tension_neg = std::clamp(1.02 * std::pow(tension_neg_raw, 0.64), 0.0, 1.0);
    double tension_eff = tension_pos - tension_neg;
    // Tn-에서 연결부 click/pop을 줄이기 위한 가드 계수.
    double tension_relax_click_guard = std::pow(tension_neg, 0.80);
    double tension_tract_strength = 0.80 * std::pow(std::max(tension_pos, tension_neg), 0.84);
    bool tension_can_drive_tract = tension_tract_strength > 0.05;
    // 플래그 렌더 경량 모드:
    // 외부 리샘플러 호출 비용을 고려해 기본값 ON, 필요 시 RESAMP_FAST_FLAGS=0으로 해제.

    // ── Breathiness(Bh): airy 질감 제어 (+추가 / -억제) ───────────────
    double bh = std::clamp(sp.breathiness / 100.0, -1.0, 1.0);
    double breathiness_eff = (bh >= 0.0) ? std::pow(bh, 0.72) : -std::pow(-bh, 0.72);

    // ── 추가 플래그 제어량 ───────────────────────────────────────────
    // Cs: 자음/연결 안정화 강도
    double cs = std::clamp((sp.consonant_stability - 50) / 50.0, -1.0, 1.0);
    double cs_pos = std::max(0.0, cs);
    double cs_neg = std::max(0.0, -cs);
    // At: 어택 선명도/완화
    double at = std::clamp(sp.attack / 100.0, -1.0, 1.0);
    // Ns: 노이즈 톤 컬러
    double ns = std::clamp(sp.noise_color / 100.0, -1.0, 1.0);
    double consonant_power = std::clamp(sp.consonant_power / 100.0, -1.0, 1.0);
    double vowel_power = std::clamp(sp.vowel_power / 100.0, -1.0, 1.0);
    double voicing_ctrl = std::clamp(sp.voicing / 100.0, -1.0, 1.0);
    double tremolo_amt = std::pow(std::clamp(sp.tremolo / 100.0, 0.0, 1.0), 0.80);
    double end_breath_amt = std::pow(std::clamp(sp.end_breath / 100.0, 0.0, 1.0), 0.58);
    double fry_head_amt = std::pow(std::clamp(sp.fry_head / 100.0, 0.0, 1.0), 0.72);
    double fry_tail_amt = std::pow(std::clamp(sp.fry_tail / 100.0, 0.0, 1.0), 0.72);
    int vocalizer_mode = std::clamp(sp.vocalizer, 0, 7);
    double vocalizer_strength = std::pow(std::clamp(sp.vocalizer_strength / 100.0, 0.0, 1.0), 0.72);
    auto smoothstep01_global = [](double x) {
        x = std::clamp(x, 0.0, 1.0);
        return x * x * (3.0 - 2.0 * x);
    };
    double short_note_amt = timing_short_note_amt;
    double ultra_short_amt = timing_ultra_short_amt;

    double fry_load = std::max(fry_head_amt, fry_tail_amt);
    double tract_load = std::max({std::fabs(vtl_eff), std::fabs(vtr_eff), std::fabs(vtw_eff),
                                  vc_amt, nn_amt, std::fabs(mo_eff)});
    double vocalizer_load = (vocalizer_mode > 0) ? vocalizer_strength : 0.0;
    double post_fx_load = std::pow(std::clamp(sp.distortion / 100.0, 0.0, 1.0), 0.70) +
                          0.36 * std::abs(sp.final_filter);
    double conflict_load = 0.28 * tract_load + 0.26 * vocalizer_load + 0.20 * std::fabs(voicing_ctrl) +
                           0.18 * growl_amt + 0.20 * voiced_growl_amt + 0.18 * fry_load +
                           0.15 * end_breath_amt + 0.20 * post_fx_load;
    double conflict_amt = smoothstep01_global((conflict_load - 0.64) / 0.86);
    double global_artifact_guard = std::clamp(1.0 - 0.18 * conflict_amt - 0.12 * ultra_short_amt * conflict_amt,
                                              0.70, 1.0);

    // 강한 질감 플래그가 겹칠 때는 가장 artifact가 큰 성분부터 완만하게 줄인다.
    tremolo_amt *= std::clamp(1.0 - 0.22 * short_note_amt * (fry_load + voiced_growl_amt), 0.74, 1.0);
    voiced_growl_amt *= std::clamp(1.0 - 0.24 * fry_load - 0.18 * ultra_short_amt - 0.12 * post_fx_load, 0.62, 1.0);
    growl_amt *= std::clamp(1.0 - 0.14 * fry_load - 0.10 * post_fx_load, 0.72, 1.0);
    fry_head_amt *= std::clamp(1.0 - 0.18 * vocalizer_load - 0.22 * voiced_growl_amt - 0.18 * ultra_short_amt, 0.58, 1.0);
    fry_tail_amt *= std::clamp(1.0 - 0.16 * vocalizer_load - 0.18 * voiced_growl_amt - 0.10 * short_note_amt, 0.64, 1.0);
    vocalizer_strength *= std::clamp(1.0 - 0.18 * std::max(std::fabs(vtw_eff), vc_amt) - 0.10 * fry_load, 0.72, 1.0);
    if (vocalizer_mode == 7) {
        vocalizer_strength *= std::clamp(1.0 - 0.20 * nn_amt - 0.12 * breathiness_eff, 0.70, 1.0);
    }
    bool vocalizer_enabled = vocalizer_mode > 0 && vocalizer_strength > 0.001;
    bool tract_feature_requested =
        (std::fabs(vtl_eff) > 0.01 || std::fabs(vtr_eff) > 0.01 || std::fabs(vtw_eff) > 0.01 ||
         vc_amt > 0.01 || nn_amt > 0.01 || std::fabs(mo_eff) > 0.01 || tension_tract_strength > 0.02);
    bool peak_formant_requested =
        (std::fabs(mo_eff) > 0.01 || std::fabs(vtl_eff) > 0.01 ||
         std::fabs(vtr_eff) > 0.01 || std::fabs(vtw_eff) > 0.01 ||
         vocalizer_enabled);
    bool formant_context_requested = peak_formant_requested || tract_feature_requested;
    bool pitch_fx_requested =
        tremolo_amt > 0.001 || voiced_growl_amt > 0.001 ||
        fry_head_amt > 0.001 || fry_tail_amt > 0.001;
    bool tract_lite_mode =
        fast_flags_mode && tract_feature_requested &&
        (spec_dim >= 1025 || output_samples >= static_cast<int>(fs * 0.75));

    // 출력 프레임별 F0/envelope/AP 구성
    std::vector<double> out_f0(out_n_frames, 0.0);
    FrameMatrix out_spec(out_n_frames, spec_dim);
    FrameMatrix out_ap(out_n_frames, spec_dim);
    double formant_conf_sum = 0.0;
    double formant_conf_min = 1.0;
    int formant_conf_count = 0;
    // frame별 반복 할당 방지
    std::vector<double> warp_buf;
    if (do_warp) warp_buf.assign(spec_dim, 0.0);
    std::vector<double> mo_warp_buf;
    if (mo_formant_blend > 1.0e-4) mo_warp_buf.assign(spec_dim, 0.0);
    std::vector<double> tract_warp_buf;
    if (std::fabs(vtl_eff) > 0.01 || tension_can_drive_tract) tract_warp_buf.assign(spec_dim, 0.0);
    std::vector<double> tract_formant_warp_buf;
    if (!tract_lite_mode && (std::fabs(vtr_eff) > 0.01 || tension_can_drive_tract)) {
        tract_formant_warp_buf.assign(spec_dim, 0.0);
    }
    std::vector<double> region_warp_buf;
    if (std::fabs(consonant_power) > 0.01 || std::fabs(vowel_power) > 0.01) {
        region_warp_buf.assign(spec_dim, 0.0);
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
        // [수정] src_total_ms 대신 src_voiced_end_ms 전달:
        // one-pass 경로가 데케이/릴리즈 테일까지 진행하지 않도록 한다.
        map_out_time_to_src(out_time_ms,
                            source_origin_ms,
                            consonant_scale,
                            consonant_src_ms,
                            consonant_tgt_ms,
                            transition_src_len_ms,
                            transition_tgt_len_ms,
                            loop_start_ms,
                            loop_len_ms,
                            src_voiced_end_ms,
                            effective_loop_mode,
                            src_time_ms,
                            in_vowel_loop);
        if (loop_wander_amt > 1.0e-4 && in_vowel_loop && has_vowel_loop &&
            effective_loop_mode == 3 && loop_len_ms > 20.0) {
            double loop_time = out_time_ms - consonant_tgt_ms - transition_tgt_len_ms;
            double phase_time = std::max(0.0, loop_time - loop_len_ms);
            double cycle_pos = std::fmod(phase_time, loop_len_ms);
            if (cycle_pos < 0.0) cycle_pos += loop_len_ms;
            double cycle_phase = cycle_pos / std::max(1.0e-6, loop_len_ms);
            double cycle_index = std::floor(phase_time / std::max(1.0e-6, loop_len_ms));
            double seed_raw = std::sin((cycle_index + 1.0) * 12.9898 + 0.37) * 43758.5453;
            double seed = 2.0 * (seed_raw - std::floor(seed_raw)) - 1.0;
            double slow = std::sin((cycle_index + 1.0) * 2.399963 + 0.71);
            double cycle_shape = std::sin(3.14159265358979323846 * cycle_phase);
            double amp_ms = std::clamp(loop_len_ms * 0.075, 2.5, 14.0) * loop_wander_amt;
            double offset_ms = amp_ms * cycle_shape * (0.72 * seed + 0.28 * slow);
            double guard_ms = std::max(anal_period, std::min(seam_ms * 0.35, loop_len_ms * 0.16));
            double lo_ms = loop_start_ms + guard_ms;
            double hi_ms = loop_start_ms + std::max(guard_ms, loop_len_ms - guard_ms);
            if (hi_ms > lo_ms) {
                src_time_ms = std::clamp(src_time_ms + offset_ms, lo_ms, hi_ms);
            }
        }
        src_time_ms = std::clamp(src_time_ms, 0.0, std::max(0.0, src_total_ms - 1.0e-6));
        double src_fi_d    = src_time_ms / anal_period;   // 분석 프레임 주기 사용
        int    src_fi      = static_cast<int>(src_fi_d);
        int    src_fi2     = src_fi + 1;
        if (in_vowel_loop && has_vowel_loop) {
            if (src_fi < loop_start_fi) src_fi = loop_start_fi;
            if (src_fi > loop_end_fi) src_fi = loop_end_fi;
            src_fi2 = src_fi + 1;
            if (src_fi2 > loop_end_fi) src_fi2 = (effective_loop_mode == 2 || effective_loop_mode == 3) ? loop_end_fi : loop_start_fi;
        } else {
            if (src_fi < 0)            src_fi = 0;
            if (src_fi >= src.n_frames) src_fi = src.n_frames - 1;
            src_fi2 = std::min(src_fi + 1, src.n_frames - 1);
        }
        double frac        = src_fi_d - src_fi;
        if (frac < 0.0) frac = 0.0;
        if (frac > 1.0) frac = 1.0;
        bool in_consonant = (out_time_ms <= consonant_tgt_ms);
        bool in_transition = (out_time_ms > consonant_tgt_ms) &&
                             (out_time_ms <= consonant_tgt_ms + transition_tgt_len_ms);
        const std::array<double, 4> formant_fallback = {720.0, 1580.0, 2820.0, 4100.0};
        std::array<double, 4> frame_formants = formant_fallback;
        double formant_conf = 1.0;
        double formant_effect_gate = 1.0;
        double formant_filter_gate = 1.0;
        if (formant_context_requested) {
            frame_formants = peak_formant_requested
                ? interpolate_formant_peaks(src, src_fi, src_fi2, frac)
                : formant_fallback;
            formant_conf = peak_formant_requested
                ? interpolate_formant_confidence(src, src_fi, src_fi2, frac)
                : 1.0;
            if (in_consonant) formant_conf *= 0.54;
            else if (in_transition) formant_conf *= 0.76;
            formant_conf = std::clamp(formant_conf, 0.0, 1.0);
            if (peak_formant_requested) {
                formant_conf_sum += formant_conf;
                formant_conf_min = std::min(formant_conf_min, formant_conf);
                ++formant_conf_count;
            }
            // [포먼트 필터 강도 강화]
            // formant_center_mix 기저를 올려 신뢰도가 낮아도 필터가 더 분명히 작동하도록 함.
            double formant_center_mix = 0.48 + 0.52 * formant_conf;
            for (int j = 0; j < 4; ++j) {
                frame_formants[j] = frame_formants[j] * formant_center_mix +
                                    formant_fallback[j] * (1.0 - formant_center_mix);
            }
            formant_effect_gate = std::clamp(0.42 + 0.58 * formant_conf, 0.42, 1.0);
            formant_filter_gate = std::clamp(0.64 + 0.36 * formant_conf, 0.64, 1.0);
        }
        double pf1 = frame_formants[0];
        double pf2 = frame_formants[1];
        double pf3 = frame_formants[2];
        double pf4 = frame_formants[3];

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
        double voiced_floor = 0.0;
        if (in_transition) {
            voiced_floor = (transition_voiced_ratio < 0.45) ? 0.20 : 0.08;
        }
        double voiced_eff = std::max(voiced, voiced_floor);
        // 프레임 단위 voiced/unvoiced 채터 억제
        double alpha = (in_consonant || in_transition) ? 0.18 : 0.30;
        alpha *= std::clamp(1.0 - 0.55 * cs_pos + 0.38 * cs_neg, 0.38, 1.50);
        voiced_state += alpha * (voiced_eff - voiced_state);
        voiced_eff = std::max(voiced_eff, voiced_state);

        // F0는 note contour를 최대한 유지.
        // (자음/무성 처리는 AP에서 담당해 click/랜덤 저음색을 줄임)
        double raw_target_f0 = out_f0[i];
        out_f0[i] = (raw_target_f0 >= 50.0) ? raw_target_f0 : 0.0;
        if (out_f0[i] >= 50.0 && pitch_fx_requested) {
            double cents = 0.0;
            if (tremolo_amt > 0.001) {
                double hz = 2.2 + 8.8 * tremolo_amt;
                double depth_cents = 12.0 + 46.0 * tremolo_amt;
                cents += tremolo_amt * depth_cents *
                         std::sin(2.0 * 3.14159265358979323846 * hz * out_time_ms / 1000.0);
            }
            if (voiced_growl_amt > 0.001 && voiced_eff > 0.18) {
                double vg_gate = voiced_growl_amt * std::clamp((voiced_eff - 0.18) / 0.82, 0.0, 1.0);
                double phase_slow = 2.0 * 3.14159265358979323846 * (18.0 + 18.0 * vg_gate) * out_time_ms / 1000.0;
                double phase_fast = 2.0 * 3.14159265358979323846 * (39.0 + 20.0 * vg_gate) * out_time_ms / 1000.0;
                double rough = std::sin(phase_slow) + 0.55 * std::sin(phase_fast + 0.9 * std::sin(phase_slow * 0.37));
                double pulse = (std::sin(phase_slow * 0.50) > 0.15) ? 1.0 : -0.35;
                cents += vg_gate * (38.0 * rough - 30.0 * pulse);
            }

            double tail_ms = 65.0 + 230.0 * fry_tail_amt;
            double tail_u = (tail_ms > 1.0)
                ? std::clamp((out_time_ms - (out_total_ms - tail_ms)) / tail_ms, 0.0, 1.0)
                : 0.0;
            double tail_shape = tail_u * tail_u * (3.0 - 2.0 * tail_u);
            double tail_fry_gate = std::clamp(1.12 * fry_tail_amt * tail_shape, 0.0, 1.0);

            double head_window_ms = std::clamp(consonant_tgt_ms * 0.78, 12.0, 70.0);
            double head_u = (head_window_ms > 1.0)
                ? std::clamp(out_time_ms / head_window_ms, 0.0, 1.0)
                : 1.0;
            double head_shape = in_consonant
                ? (1.0 - (head_u * head_u * (3.0 - 2.0 * head_u)))
                : 0.0;
            double head_fry_gate = std::clamp(1.13 * fry_head_amt * head_shape, 0.0, 1.0);
            double fry_gate = std::max(head_fry_gate, tail_fry_gate);
            if (fry_gate > 0.001) {
                double pulse = (std::sin(2.0 * 3.14159265358979323846 * 23.0 * out_time_ms / 1000.0) > 0.25) ? 1.0 : 0.0;
                double high_tail = std::clamp((tail_fry_gate - 0.64) / 0.34, 0.0, 1.0);
                double high_head = std::clamp((head_fry_gate - 0.42) / 0.48, 0.0, 1.0);
                double high_fry = std::max(high_tail, 0.80 * high_head);
                double comb_pulse = (std::sin(2.0 * 3.14159265358979323846 * 34.0 * out_time_ms / 1000.0) > (0.62 - 0.24 * high_fry)) ? 1.0 : 0.0;
                double short_pitch_guard = std::clamp((380.0 - out_total_ms) / 260.0, 0.0, 1.0);
                double head_drop = head_fry_gate * (132.0 + 76.0 * pulse)
                                 + high_head * (220.0 + 92.0 * comb_pulse);
                double tail_drop = tail_fry_gate * (90.0 + 48.0 * pulse)
                                 + high_tail * (145.0 + 62.0 * comb_pulse);
                head_drop = std::min(head_drop, 330.0 - 205.0 * short_pitch_guard);
                tail_drop = std::min(tail_drop, 260.0 - 165.0 * short_pitch_guard);
                double drop = head_drop + tail_drop;
                drop = std::min(drop, 390.0 - 250.0 * short_pitch_guard);
                cents -= drop;
            }

            if (std::fabs(cents) > 0.001) {
                out_f0[i] *= std::pow(2.0, cents / 1200.0);
            }
        }
        if (voicing_ctrl <= -0.995) {
            out_f0[i] = 0.0;
        }

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
        const bool forward_wrap_loop = (effective_loop_mode == 1);
        const bool midpoint_seam_loop =
            (long_loop_stress > 0.25 && loop_endpoint_match_amt > 1.0e-4);
        if (forward_wrap_loop && in_vowel_loop && has_vowel_loop &&
            !midpoint_seam_loop &&
            seam_ms > 0.5 && loop_len_ms > (2.0 * seam_ms + anal_period)) {
            double loop_time = out_time_ms - consonant_tgt_ms - transition_tgt_len_ms;
            double wrapped = std::fmod(loop_time, loop_len_ms);
            if (wrapped < 0.0) wrapped += loop_len_ms;

            double w_wrap = 0.0;
            double alt_src_time_ms = src_time_ms;
            if (wrapped < seam_ms) {
                double u = std::clamp(wrapped / seam_ms, 0.0, 1.0);
                u = u * u * (3.0 - 2.0 * u);
                w_wrap = 1.0 - u;
                alt_src_time_ms = loop_end_ms - (seam_ms - wrapped);
            } else if (wrapped > (loop_len_ms - seam_ms)) {
                double d = loop_len_ms - wrapped;
                double u = std::clamp(d / seam_ms, 0.0, 1.0);
                u = u * u * (3.0 - 2.0 * u);
                w_wrap = 1.0 - u;
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

        // 짧은 루프에서 loop start/end의 에너지 또는 원본 F0 차이가 크면
        // seam 인접 프레임을 양 끝의 중간 스펙트럼/레벨 쪽으로 약하게 보정한다.
        if (loop_endpoint_match_amt > 1.0e-4 && in_vowel_loop && has_vowel_loop &&
            loop_endpoint_width_ms > 0.5 && !loop_mid_spec.empty()) {
            double loop_time = out_time_ms - consonant_tgt_ms - transition_tgt_len_ms;
            double wrapped = std::fmod(loop_time, loop_len_ms);
            if (wrapped < 0.0) wrapped += loop_len_ms;

            double w_start = 0.0;
            double w_end = 0.0;
            if (wrapped < loop_endpoint_width_ms) {
                double u = std::clamp(wrapped / loop_endpoint_width_ms, 0.0, 1.0);
                w_start = 1.0 - (u * u * (3.0 - 2.0 * u));
            }
            if (wrapped > (loop_len_ms - loop_endpoint_width_ms)) {
                double d = loop_len_ms - wrapped;
                double u = std::clamp(d / loop_endpoint_width_ms, 0.0, 1.0);
                w_end = 1.0 - (u * u * (3.0 - 2.0 * u));
            }

            double w_sum = w_start + w_end;
            if (w_sum > 1.0e-4) {
                double edge_w = std::clamp(w_sum, 0.0, 1.0);
                double gain_db = ((w_start * loop_start_gain_db + w_end * loop_end_gain_db) /
                                  std::max(1.0e-6, w_sum)) * edge_w;
                double gain = std::pow(10.0, gain_db / 10.0);
                double blend_scale = 0.24 + 0.22 * loop_endpoint_sustain_amt;
                double blend_cap = 0.18 + 0.12 * loop_endpoint_sustain_amt;
                double mid_blend = std::clamp(edge_w * loop_endpoint_match_amt * blend_scale, 0.0, blend_cap);
                double ap_blend = mid_blend * (0.46 + 0.10 * loop_endpoint_sustain_amt);
                for (int k = 0; k < spec_dim; ++k) {
                    double p = out_spec[i][k] * gain;
                    out_spec[i][k] = p * (1.0 - mid_blend) + loop_mid_spec[k] * mid_blend;
                    out_ap[i][k] = out_ap[i][k] * (1.0 - ap_blend) + loop_mid_ap[k] * ap_blend;
                }
            }
        }

        // 연결부/자음의 무성성은 AP 상승으로 처리해 파형 불연속을 줄인다.
        {
            double uv_boost = 0.0;
            if (in_consonant || in_transition) {
                double unvoiced = 1.0 - voiced_eff;
                double base_boost = in_consonant ? 0.12 : 0.06;
                uv_boost = base_boost * (0.20 + 0.80 * unvoiced * unvoiced);
                // C+V 단음절(자음 구간 매우 짧음)에서 과도한 AP 부스트 억제
                if (transition_voiced_ratio < 0.40 && in_transition && consonant_src_ms >= 6.0)
                    uv_boost += 0.03;
                uv_boost *= std::clamp(1.0 - 0.78 * cs_pos + 0.52 * cs_neg, 0.28, 1.85);
                if (std::fabs(vtw_eff) > 0.01 && (in_consonant || in_transition)) {
                    double vtw_air_guard = std::pow(std::clamp(std::fabs(vtw_eff), 0.0, 1.0), 0.55);
                    uv_boost *= std::clamp(1.0 - 0.48 * vtw_air_guard, 0.46, 1.0);
                }
                // Tn-일 때 연결부 과도한 무성 부스트를 줄여 "툭툭" 임펄스 억제.
                uv_boost *= (1.0 - 0.38 * tension_relax_click_guard);
            }
            uv_boost = std::clamp(uv_boost, 0.0, 0.11);
            if (uv_boost > 0.01) {
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = fn_lut[k];
                    // 연결부 click 억제를 위해 고역 과증가를 완화.
                    double shaped = uv_boost * (0.74 + 0.26 * fn);
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
            double stable_elapsed_ms = out_time_ms - consonant_tgt_ms - transition_tgt_len_ms;
            double anchor_ramp_ms = std::max(seam_ms, 0.45 * transition_tgt_len_ms + 8.0);
            if (!flat_target_f0 && stable_elapsed_ms < anchor_ramp_ms) {
                double u = std::clamp(stable_elapsed_ms / std::max(0.1, anchor_ramp_ms), 0.0, 1.0);
                u = u * u * (3.0 - 2.0 * u);
                w_anchor = u * u * (3.0 - 2.0 * u);
            }
            w_anchor *= anchor_mix_cap;
            for (int k = 0; k < spec_dim; ++k) {
                out_spec[i][k] = out_spec[i][k] * (1.0 - w_anchor) + vowel_spec_anchor[k] * w_anchor;
                out_ap[i][k]   = out_ap[i][k]   * (1.0 - w_anchor) + vowel_ap_anchor[k]   * w_anchor;
            }
        }

        // 5.6. Vocalizer(Vz): 선택한 발음의 포먼트 필터를 envelope에 혼합.
        // 값: 0=off, 1=아, 2=에, 3=이, 4=오, 5=우, 6=어, 7=N.
        // 원본 발음/캐릭터를 완전히 지우는 변환기는 아니고, 목표 포먼트 쪽으로 밀어주는 필터다.
        if (in_vowel_loop && long_loop_stress > 0.18 &&
            (auto_stretch_mirror_loop || auto_short_stretch_first || auto_natural_mirror_loop)) {
            double room_guard = std::clamp((long_loop_stress - 0.18) / 0.82, 0.0, 1.0);
            room_guard *= std::clamp(0.45 + 0.55 * loop_endpoint_sustain_amt, 0.45, 1.0);
            for (int k = 0; k < spec_dim; ++k) {
                double hz = hz_lut[k];
                double low_room = std::exp(-0.5 * std::pow(std::log2((hz + 80.0) / 520.0) / 0.82, 2.0));
                double air_room = std::clamp((fn_lut[k] - 0.42) / 0.58, 0.0, 1.0);
                double db = -room_guard * (0.85 * low_room + 0.38 * air_room);
                out_spec[i][k] *= std::pow(10.0, db / 10.0);
                out_ap[i][k] = std::clamp(out_ap[i][k] + room_guard * (0.010 * low_room + 0.006 * air_room),
                                          0.0, 1.0);
            }
        }

        if (vocalizer_enabled) {
            VocalizerTarget vt = vocalizer_target(vocalizer_mode);
            double region_gate = in_consonant ? 0.16 : (in_transition ? 0.64 : 1.0);
            double voiced_gate = std::clamp(0.18 + 0.82 * voiced_eff, 0.0, 1.0);
            // 보컬라이저는 목표 포먼트 형태를 직접 부여하므로 formant_conf 의존을 낮게 설정
            double vocalizer_gate = std::clamp(0.72 + 0.28 * formant_conf, 0.72, 1.0);
            double mix = std::clamp((vt.nasal ? 0.96 : 0.92) * vocalizer_strength *
                                    region_gate * voiced_gate * global_artifact_guard *
                                    vocalizer_gate, 0.0, 0.96);
            if (mix > 0.01) {
                double e0 = 0.0;
                double e1 = 0.0;
                double src_f1 = std::clamp(pf1, 180.0, 1300.0);
                double src_f2 = std::clamp(pf2, src_f1 + 260.0, 3400.0);
                double src_f3 = std::clamp(pf3, src_f2 + 420.0, 5200.0);
                double src_f4 = std::clamp(pf4, src_f3 + 520.0, std::max(src_f3 + 520.0, fs * 0.46));
                double t1 = std::clamp(vt.f1, 160.0, fs * 0.46);
                double t2 = std::clamp(vt.f2, t1 + 230.0, fs * 0.46);
                double t3 = std::clamp(vt.f3, t2 + 360.0, fs * 0.46);
                double t4 = std::clamp(vt.f4, t3 + 440.0, fs * 0.46);
                double high_roll = (vocalizer_mode == 3) ? 1.15 :
                                   ((vocalizer_mode == 2) ? 0.45 :
                                   ((vocalizer_mode == 4 || vocalizer_mode == 5) ? -1.45 : -0.12));
                for (int k = 0; k < spec_dim; ++k) {
                    double p0 = std::max(0.0, out_spec[i][k]);
                    e0 += p0;
                    double fn = fn_lut[k];
                    double hz = hz_lut[k];
                    double st1 = log_freq_bell(hz, src_f1, 0.42);
                    double st2 = log_freq_bell(hz, src_f2, 0.46);
                    double st3 = log_freq_bell(hz, src_f3, 0.52);
                    double st4 = log_freq_bell(hz, src_f4, 0.62);
                    double tg1 = log_freq_bell(hz, t1, 0.34);
                    double tg2 = log_freq_bell(hz, t2, 0.36);
                    double tg3 = log_freq_bell(hz, t3, 0.42);
                    double tg4 = log_freq_bell(hz, t4, 0.56);
                    double valley12 = log_freq_bell(hz, std::sqrt(t1 * t2), 0.28);
                    double valley23 = log_freq_bell(hz, std::sqrt(t2 * t3), 0.32);
                    double low_band = hz_bell(hz, 430.0, 310.0);
                    double low_mid = log_freq_bell(hz, 980.0, 0.42);
                    double mid_front = log_freq_bell(hz, 2050.0, 0.42);
                    double front_edge = log_freq_bell(hz, 3050.0, 0.44);
                    double round_notch = log_freq_bell(hz, 1650.0, 0.46);
                    double round_low = log_freq_bell(hz, 700.0, 0.34);
                    double high_air = std::max(0.0, fn - 0.48);

                    double db = 0.0;
                    double ap_delta = 0.0;
                    if (vt.nasal) {
                        double nasal_low = hz_bell(hz, 250.0, 125.0);
                        double nasal_mid = log_freq_bell(hz, 980.0, 0.34);
                        double nasal_hi = log_freq_bell(hz, 2350.0, 0.38);
                        double anti1 = log_freq_bell(hz, 650.0, 0.26);
                        double anti2 = log_freq_bell(hz, 1650.0, 0.30);
                        double anti3 = log_freq_bell(hz, 3450.0, 0.42);
                        db = 18.5 * nasal_low + 11.2 * nasal_mid + 6.4 * nasal_hi
                           - 17.0 * anti1 - 15.0 * anti2 - 8.5 * anti3
                           - 5.4 * st2 - 4.2 * st3 - 2.0 * high_air;
                        ap_delta = 0.030 + 0.082 * nasal_hi + 0.040 * high_air
                                 - 0.042 * nasal_low;
                    } else {
                        db = 11.8 * tg1 + 14.0 * tg2 + 9.2 * tg3 + 3.0 * tg4
                           - 6.8 * st1 - 8.4 * st2 - 5.1 * st3 - 1.8 * st4
                           - 4.8 * valley12 - 3.5 * valley23
                           + high_roll * std::max(0.0, fn - 0.36) * 4.8;
                        if (vocalizer_mode == 1) {          // 아: 열린 F1과 낮은 F2를 더 분명히
                            db += 8.0 * tg1 + 3.4 * low_mid
                                - 5.6 * mid_front - 4.2 * front_edge - 2.6 * high_air;
                        } else if (vocalizer_mode == 2) {   // 에: 중고역 전방 모음
                            db += 6.8 * tg2 + 4.0 * tg3 + 2.2 * front_edge
                                - 4.4 * round_low - 2.6 * low_band;
                        } else if (vocalizer_mode == 3) {   // 이: F2/F3 전방성을 강조
                            db += 9.6 * tg2 + 6.2 * tg3 + 3.0 * front_edge
                                - 8.0 * low_band - 5.8 * round_low - 2.2 * low_mid;
                        } else if (vocalizer_mode == 4) {   // 오: 둥글고 뒤쪽, 우보다 조금 열림
                            db += 7.0 * tg1 + 5.0 * round_low
                                - 8.0 * round_notch - 5.6 * mid_front - 3.8 * high_air;
                        } else if (vocalizer_mode == 5) {   // 우: 가장 좁고 어두운 rounded 모음
                            db += 5.6 * tg1 + 8.2 * round_low
                                - 10.0 * round_notch - 7.4 * mid_front - 7.0 * front_edge - 6.0 * high_air;
                        } else if (vocalizer_mode == 6) {   // 어: 중립/중앙 모음 성향
                            db += 5.8 * tg1 + 5.0 * tg2 + 1.8 * low_mid
                                - 3.8 * front_edge - 3.2 * round_low - 2.4 * high_air;
                        }
                        ap_delta = -0.030 * (tg1 + tg2 + 0.55 * tg3) + 0.006 * std::max(0.0, fn - 0.55);
                    }
                    db = std::clamp(db * mix, -22.0, 22.0);
                    out_spec[i][k] = p0 * std::pow(10.0, db / 10.0);
                    out_ap[i][k] = std::clamp(out_ap[i][k] + mix * ap_delta, 0.0, 1.0);
                    e1 += out_spec[i][k];
                }
                if (e0 > 1.0e-12 && e1 > 1.0e-12) {
                    double norm = std::clamp(e0 / e1, 0.52, 1.95);
                    norm = 1.0 + 0.76 * mix * (norm - 1.0);
                    norm = std::clamp(norm, 0.62, 1.55);
                    for (int k = 0; k < spec_dim; ++k) out_spec[i][k] *= norm;
                }
            }
        }

        // 자음/모음 강화: 볼륨성 대역 에너지와 약한 포먼트 이동을 함께 적용.
        {
            double region_ctrl = 0.0;
            double gate = 0.0;
            if (in_consonant || in_transition) {
                gate = in_consonant ? 1.0 : 0.62;
                region_ctrl = consonant_power * gate;
            } else {
                gate = 1.0;
                region_ctrl = vowel_power * gate;
            }

            if (std::fabs(region_ctrl) > 0.01) {
                double pos = std::max(0.0, region_ctrl);
                double neg = std::max(0.0, -region_ctrl);
                double ratio = std::clamp(std::exp(0.085 * region_ctrl), 0.84, 1.20);
                double blend = std::clamp(0.14 + 0.34 * std::fabs(region_ctrl), 0.0, 0.50);
                if (!region_warp_buf.empty()) {
                    double inv = 1.0 / ratio;
                    for (int k = 0; k < spec_dim; ++k) {
                        double src_k = std::clamp(k * inv, 0.0, static_cast<double>(spec_dim - 1));
                        int sk = static_cast<int>(src_k);
                        int sk2 = std::min(sk + 1, spec_dim - 1);
                        double f = src_k - sk;
                        region_warp_buf[k] = out_spec[i][sk] * (1.0 - f) + out_spec[i][sk2] * f;
                    }
                    for (int k = 0; k < spec_dim; ++k) {
                        out_spec[i][k] = out_spec[i][k] * (1.0 - blend) + region_warp_buf[k] * blend;
                    }
                }
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = fn_lut[k];
                    double body = std::exp(-0.5 * std::pow((fn - 0.24) / 0.20, 2.0));
                    double pres = std::exp(-0.5 * std::pow((fn - 0.48) / 0.18, 2.0));
                    double hi = std::clamp((fn - 0.48) / 0.52, 0.0, 1.0);
                    double db = pos * (2.2 + 4.6 * body + 5.2 * pres + 1.8 * hi)
                              - neg * (1.8 + 3.6 * body + 3.8 * pres + 1.2 * hi);
                    out_spec[i][k] *= std::pow(10.0, db / 10.0);
                    double ap_delta = -pos * (0.052 + 0.085 * pres)
                                    + neg * (0.060 + 0.105 * hi);
                    out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
                }
            }
        }

        // 강제 유성화/무성화: F0는 극단 무성화에서만 끊고, 중간값은 AP 중심으로 처리.
        if (std::fabs(voicing_ctrl) > 0.01) {
            double v_pos = std::max(0.0, voicing_ctrl) * global_artifact_guard;
            double v_neg = std::max(0.0, -voicing_ctrl);
            double f0_for_harmonics = out_f0[i];
            for (int k = 0; k < spec_dim; ++k) {
                double fn = fn_lut[k];
                double hz = hz_lut[k];
                double hi = std::clamp((fn - 0.42) / 0.58, 0.0, 1.0);
                double low = std::exp(-0.5 * std::pow((fn - 0.18) / 0.18, 2.0));
                double harmonic = 0.0;
                if (v_pos > 0.001 && f0_for_harmonics >= 50.0 && hz >= f0_for_harmonics * 0.75) {
                    double h = hz / f0_for_harmonics;
                    double nearest = std::round(h);
                    if (nearest >= 1.0 && nearest <= 80.0) {
                        double dist = std::fabs(h - nearest);
                        harmonic = std::exp(-0.5 * std::pow(dist / 0.105, 2.0));
                    }
                }
                out_ap[i][k] = std::clamp(out_ap[i][k]
                                        - v_pos * (0.18 + 0.40 * hi + 0.24 * harmonic)
                                        + v_neg * (0.18 + 0.48 * hi), 0.0, 1.0);
                double db = v_pos * (1.0 * low - 0.8 * hi + 4.2 * harmonic * (0.55 + 0.45 * low))
                          + v_neg * (-1.1 * low + 1.0 * hi);
                out_spec[i][k] *= std::pow(10.0, db / 10.0);
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
            double bi_pos = std::max(0.0, bi_eff);
            double bi_neg = std::max(0.0, -bi_eff);
            for (int k = 0; k < spec_dim; ++k) {
                double freq_norm = static_cast<double>(k) / std::max(1, spec_dim - 1);
                double hz = hz_lut[k];
                double x1 = std::log2((hz + 120.0) / 2200.0);
                double bell1 = std::exp(-0.5 * (x1 * x1) / (0.85 * 0.85));
                double x2 = std::log2((hz + 120.0) / 4200.0);
                double bell2 = std::exp(-0.5 * (x2 * x2) / (0.92 * 0.92));
                double tilt_db = brightness_tilt_db * (freq_norm - 0.5);
                tilt_db += bi_pos * (2.8 * bell1 + 1.7 * bell2 + 1.3 * bi_knee * bell2)
                         - bi_neg * (2.2 * bell1 + 1.4 * bell2 + 1.0 * bi_knee * std::max(0.0, freq_norm - 0.35));
                double gain_pow  = std::pow(10.0, tilt_db / 10.0); // power
                out_spec[i][k] *= gain_pow;
                double ap_delta = -bi_pos * (0.030 * bell1 + 0.020 * bell2)
                                + bi_neg * (0.025 * bell1 + 0.030 * std::max(0.0, freq_norm - 0.40));
                out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
            }
        }

        // 5.3. Mouth Open(Mo): 입 열림(+)/입 닫힘(-) 톤 이동
        // AP는 유지하고 스펙트럼만 이동한 뒤 에너지 정규화.
        if (std::fabs(mo_eff) > 0.01) {
            double mo_frame_blend = mo_formant_blend * formant_filter_gate;
            double mo_frame_gate = formant_effect_gate;
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
                    out_spec[i][k] = out_spec[i][k] * (1.0 - mo_frame_blend)
                                   + mo_warp_buf[k] * mo_frame_blend;
                }
            }

            // (B) 밴드별 기울기: +Mo(개방) / -Mo(닫힘) 캐릭터 분리
            double e0 = 0.0;
            double e1 = 0.0;
            double mo_pos = std::max(0.0, mo_eff) * mo_frame_gate;
            double mo_neg = std::max(0.0, -mo_eff) * mo_frame_gate;
            for (int k = 0; k < spec_dim; ++k) {
                double p0 = std::max(0.0, out_spec[i][k]);
                e0 += p0;

                double fn = fn_lut[k];
                double hz = hz_lut[k];
                double mo_f1_c = std::clamp(pf1 * (1.0 + 0.12 * mo_pos - 0.16 * mo_neg), 180.0, 1450.0);
                double mo_f2_c = std::clamp(pf2 * (1.0 + 0.10 * mo_pos - 0.20 * mo_neg), mo_f1_c + 300.0, 3400.0);
                double mo_f3_c = std::clamp(pf3 * (1.0 + 0.08 * mo_pos - 0.14 * mo_neg), mo_f2_c + 420.0, 5200.0);
                double x_f1 = std::log2((hz + 120.0) / mo_f1_c);
                double f1 = std::exp(-0.5 * (x_f1 * x_f1) / (0.72 * 0.72));
                double x_f2 = std::log2((hz + 120.0) / mo_f2_c);
                double f2 = std::exp(-0.5 * (x_f2 * x_f2) / (0.82 * 0.82));
                double x_f3 = std::log2((hz + 120.0) / mo_f3_c);
                double f3 = std::exp(-0.5 * (x_f3 * x_f3) / (0.88 * 0.88));
                double x_lip = std::log2((hz + 120.0) / std::clamp(pf4 * (1.0 + 0.06 * mo_pos), 3000.0, 7600.0));
                double lip = std::exp(-0.5 * (x_lip * x_lip) / (0.92 * 0.92));
                double x_mud = std::log2((hz + 120.0) / std::clamp(0.48 * pf1, 260.0, 620.0));
                double mud = std::exp(-0.5 * (x_mud * x_mud) / (0.95 * 0.95));
                double x_box = std::log2((hz + 120.0) / std::clamp(0.72 * pf2, 760.0, 1700.0));
                double box = std::exp(-0.5 * (x_box * x_box) / (0.85 * 0.85));
                double x_mumble = std::log2((hz + 120.0) / std::clamp(0.88 * pf3, 1800.0, 3600.0));
                double mumble_notch = std::exp(-0.5 * (x_mumble * x_mumble) / (0.60 * 0.60));
                double x_muffle_hi = std::log2((hz + 120.0) / std::clamp(pf4, 3200.0, 6800.0));
                double muffle_hi = std::exp(-0.5 * (x_muffle_hi * x_muffle_hi) / (0.78 * 0.78));
                double x_mumble_core = std::log2((hz + 120.0) / std::clamp(pf2, 1050.0, 2600.0));
                double mumble_core = std::exp(-0.5 * (x_mumble_core * x_mumble_core) / (0.52 * 0.52));
                double x_throat = std::log2((hz + 120.0) / std::clamp(pf1, 360.0, 1100.0));
                double throat = std::exp(-0.5 * (x_throat * x_throat) / (0.74 * 0.74));
                // +Mo: 개방감/선명도는 유지하되 이전보다 약하게
                // -Mo: 단순 감쇠보다 "딕션 대역(1.5~3k) 붕괴 + 구강 공명 저역화"로
                // 웅얼거리는 캐릭터를 만든다.
                double shape_open = (0.72 * f1 + 0.98 * f2 + 1.06 * f3 + 0.80 * lip
                                   - 0.30 * mud - 0.12 * box);
                double shape_close = (0.72 * mud + 0.96 * box + 0.86 * f1 + 0.36 * throat
                                    - 0.92 * f2 - 1.04 * f3 - 0.68 * lip
                                    - 1.18 * mumble_notch - 0.86 * mumble_core
                                    - 0.34 * muffle_hi);
                double db = mo_pos * 8.2 * shape_open + mo_neg * 10.6 * shape_close;
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
            vc_amt > 0.01 || nn_amt > 0.01 || std::fabs(mo_eff) > 0.01 ||
            tension_tract_strength > 0.02) {
            auto smoothstep01 = [](double x) {
                x = std::clamp(x, 0.0, 1.0);
                return x * x * (3.0 - 2.0 * x);
            };

            // Tn을 성도 레이어에 직접 결합:
            // +Tn: 후두 상승/epilaryngeal narrowing 성향(짧고 집중된 tract)
            // -Tn: 이완/후두 하강 성향(길고 퍼진 tract)
            double tn_region = in_consonant ? 0.50 : (in_transition ? 0.78 : 1.0);
            double tn_gate = (0.24 + 0.76 * voiced_eff) * tn_region;
            double tn_drive = tension_tract_strength * tn_gate;

            double vtl_eff_local = std::clamp(vtl_eff + (-0.14 * tension_pos + 0.07 * tension_neg) * tn_gate,
                                              -1.0, 1.0);
            double vtr_eff_local = std::clamp(vtr_eff + (0.19 * tension_pos - 0.07 * tension_neg) * tn_gate,
                                              -1.0, 1.0);
            double vtw_eff_local = std::clamp(vtw_eff + (0.19 * tension_pos - 0.07 * tension_neg) * tn_gate,
                                              -1.0, 1.0);
            double vc_amt_local = std::clamp(vc_amt + (0.20 * tension_pos - 0.06 * tension_neg) * tn_gate,
                                             0.0, 1.0);
            double nn_local = std::clamp(nn + (-0.04 * tension_pos + 0.04 * tension_neg) * tn_gate,
                                         -1.0, 1.0);
            double nn_pos_eff_local = std::pow(std::max(0.0, nn_local), 0.40);
            double nn_neg_eff_local = std::pow(std::max(0.0, -nn_local), 0.48);
            double nn_amt_local = std::max(nn_pos_eff_local, nn_neg_eff_local);

            // 아래 tract 모듈 계산은 텐션이 반영된 local 파라미터를 사용한다.
            double vtl_eff = vtl_eff_local;
            double vtr_eff = vtr_eff_local;
            double vtw_eff = vtw_eff_local;
            double vc_amt = vc_amt_local;
            double nn_pos_eff = nn_pos_eff_local;
            double nn_neg_eff = nn_neg_eff_local;
            double nn_amt = nn_amt_local;
            vtl_eff *= formant_filter_gate;
            vtr_eff *= formant_effect_gate;
            vtw_eff *= formant_effect_gate;
            vc_amt *= std::clamp(0.62 + 0.38 * formant_conf, 0.62, 1.0);

            double voiced_gate = 0.18 + 0.82 * voiced_eff;
            double region_gate = in_consonant ? 0.30 : (in_transition ? 0.58 : 1.00);
            double vowel_enter_ms = consonant_tgt_ms + transition_tgt_len_ms;
            double ramp_in = smoothstep01((out_time_ms - vowel_enter_ms + 4.0) / 24.0);
            double ramp_out = smoothstep01((out_total_ms - out_time_ms) / 26.0);
            double time_gate = std::clamp(ramp_in * ramp_out, 0.18, 1.0);

            double vtl_drive_base = std::pow(std::clamp(std::fabs(vtl_eff), 0.0, 1.0), 0.58);
            double vtl_drive = std::clamp(1.24 * vtl_drive_base * voiced_gate * region_gate * time_gate, 0.0, 1.0);
            double vtr_drive = std::pow(std::clamp(std::fabs(vtr_eff), 0.0, 1.0), 0.42);
            double vtw_drive = std::pow(std::clamp(std::fabs(vtw_eff), 0.0, 1.0), 0.34);
            double vc_drive = std::pow(std::clamp(vc_amt, 0.0, 1.0), 0.50);
            double nn_drive = std::pow(std::clamp(nn_amt, 0.0, 1.0), 0.50);
            double mo_drive = std::pow(std::clamp(std::fabs(mo_eff), 0.0, 1.0), 0.92);
            // 텐션이 성도 레이어를 끌어올리되, 플래그 원래 캐릭터를 덮지 않게 상한 제한.
            vtr_drive = std::clamp(vtr_drive + 0.18 * tn_drive, 0.0, 1.0);
            vtw_drive = std::clamp(vtw_drive + 0.20 * tn_drive, 0.0, 1.0);
            vc_drive  = std::clamp(vc_drive  + 0.18 * tn_drive, 0.0, 1.0);
            nn_drive  = std::clamp(nn_drive  + 0.08 * tension_neg * tn_gate, 0.0, 1.0);
            vtw_drive *= std::clamp(global_artifact_guard + 0.05, 0.74, 1.0);
            vc_drive  *= std::clamp(global_artifact_guard + 0.08, 0.78, 1.0);
            double vtw_connect_gate = std::clamp((0.30 + 0.70 * voiced_eff) *
                                                 (in_consonant ? 0.18 : (in_transition ? 0.42 : 1.0)) *
                                                 (0.52 + 0.48 * time_gate),
                                                 0.10, 1.0);
            vtw_drive *= vtw_connect_gate;
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
            // [포먼트 필터 강도 강화] 각 모듈 스케일 ~20% 상향.
            constexpr double k_vtr = 7.60;
            constexpr double k_vtw = 6.90;
            constexpr double k_vc  = 6.10;
            constexpr double k_nn  = 4.75;
            constexpr double k_mo  = 1.64;

            double mix_vtr = ((std::fabs(vtr_eff) > 0.01) ? 0.18 : 0.0) + 0.92 * vtr_drive;
            double mix_vtw = ((std::fabs(vtw_eff) > 0.01) ? 0.17 : 0.0) + 0.96 * vtw_drive;
            double mix_vc  = (vc_drive > 0.01 ? 0.14 : 0.0) + 0.86 * vc_drive;
            double mix_nn  = (nn_drive > 0.01 ? 0.12 : 0.0) + 0.72 * nn_drive;
            double mix_mo  = (mo_drive > 0.01 ? 0.05 : 0.0) + 0.34 * mo_drive;
            double tract_mix_boost = 1.14;
            mix_vtr *= (1.07 + 0.28 * vtr_drive);
            mix_vtw *= (1.13 + 0.46 * vtw_drive);
            mix_vc  *= (1.03 + 0.24 * vc_drive);
            mix_nn  *= (1.04 + 0.32 * nn_drive);
            mix_vtr *= tract_mix_boost;
            mix_vtw *= tract_mix_boost;
            mix_vtw *= vtw_connect_gate;
            mix_vc  *= tract_mix_boost;
            mix_nn  *= tract_mix_boost;
            mix_vtr = std::clamp(mix_vtr, 0.0, 1.00);
            mix_vtw = std::clamp(mix_vtw, 0.0, 1.00);
            mix_vc  = std::clamp(mix_vc,  0.0, 0.97);
            mix_nn  = std::clamp(mix_nn,  0.0, 0.95);
            mix_mo  = std::clamp(mix_mo,  0.0, 0.48);

            double tract_db_boost = 1.18;
            double boost_vtr = tract_db_boost * (1.38 + 0.66 * vtr_drive);
            double boost_vtw = tract_db_boost * (1.44 + 0.82 * vtw_drive);
            double boost_vc  = tract_db_boost * (1.20 + 0.42 * vc_drive);
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
                // - g와 달리 F1은 거의 고정하고 F2/F3 간격과 고역 roll-off를 비선형으로 이동
                // - +값: 더 긴 front/oral tract 성향(F2/F3 하향, 고역 완화)
                // - -값: 더 짧고 앞쪽으로 당겨진 tract 성향(F2/F3 상향, edge 증가)
                double vtl_pos = std::max(0.0, vtl_eff);
                double vtl_neg = std::max(0.0, -vtl_eff);
                double vtl_strength = vtl_eff * (0.58 + 1.72 * vtl_drive);
                double vtl_blend = std::clamp(0.30 + 1.06 * vtl_drive, 0.16, 0.98);
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = fn_lut[k];
                    double hz = hz_lut[k];
                    double f1_bell = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / std::max(180.0, pf1)) / 0.44, 2.0));
                    double f2_bell = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / std::max(320.0, pf2)) / 0.50, 2.0));
                    double f3_bell = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / std::max(520.0, pf3)) / 0.54, 2.0));
                    double above_f1 = smoothstep01((hz - std::max(220.0, pf1 * 1.18)) / std::max(320.0, pf2 - pf1));
                    double high_band = smoothstep01((fn - 0.34) / 0.58);
                    double spacing_weight = above_f1 * (0.24 + 1.05 * f2_bell + 1.32 * f3_bell + 0.72 * high_band);
                    spacing_weight *= (1.0 - 0.58 * f1_bell);
                    double local_ratio = std::clamp(std::exp(-vtl_strength * spacing_weight * 0.42), 0.66, 1.52);
                    double src_k = k * (1.0 / local_ratio);
                    src_k = std::clamp(src_k, 0.0, static_cast<double>(spec_dim - 1));
                    int sk = static_cast<int>(src_k);
                    int sk2 = std::min(sk + 1, spec_dim - 1);
                    double f = src_k - sk;
                    tract_warp_buf[k] = out_spec[i][sk] * (1.0 - f) + out_spec[i][sk2] * f;
                }
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = fn_lut[k];
                    double hz = hz_lut[k];
                    double high_band = smoothstep01((fn - 0.38) / 0.56);
                    double f1_guard = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / std::max(180.0, pf1)) / 0.46, 2.0));
                    double local_blend = vtl_blend * std::clamp(0.38 + 0.78 * high_band + 0.24 * vtl_neg, 0.22, 1.0);
                    local_blend *= (1.0 - 0.42 * f1_guard);
                    double p = out_spec[i][k] * (1.0 - local_blend) + tract_warp_buf[k] * local_blend;
                    double color_db = vtl_pos * (-2.5 * high_band) + vtl_neg * (2.2 * high_band);
                    out_spec[i][k] = p * std::pow(10.0, color_db / 10.0);
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
                double vtr_shift_low = ((0.18 * vtr_pos) - (0.34 * vtr_neg)) * (0.32 + 0.68 * vtr_drive);
                double vtr_shift_mid = ((1.18 * vtr_pos) - (1.02 * vtr_neg)) * (0.38 + 0.62 * vtr_drive);
                double vtr_shift_high = ((1.34 * vtr_pos) - (0.88 * vtr_neg)) * (0.42 + 0.58 * vtr_drive);

                double shift_low  = (0.70 * vtr_shift_low) * tract_formant_gate;
                double shift_mid  = (1.04 * vtr_shift_mid) * tract_formant_gate;
                double shift_high = (1.12 * vtr_shift_high) * tract_formant_gate;
                double ratio_low  = std::clamp(std::exp(shift_low * 0.13), 0.88, 1.16);
                double ratio_mid  = std::clamp(std::exp(shift_mid * 0.28), 0.70, 1.56);
                double ratio_high = std::clamp(std::exp(shift_high * 0.32), 0.64, 1.70);
                double tract_formant_blend = std::clamp(
                    (0.18 + 0.82 * std::pow(std::fabs(vtr_eff), 0.72)) * tract_formant_gate,
                    0.0, 0.96);
                if ((std::fabs(ratio_low - 1.0) > 0.003 || std::fabs(ratio_mid - 1.0) > 0.003 ||
                     std::fabs(ratio_high - 1.0) > 0.003) &&
                    tract_formant_blend > 0.01) {
                    for (int k = 0; k < spec_dim; ++k) {
                        double fn = fn_lut[k];
                        double hz = hz_lut[k];
                        double f1_bell = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / std::max(180.0, pf1)) / 0.58, 2.0));
                        double f2_bell = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / std::max(320.0, pf2)) / 0.52, 2.0));
                        double f3_bell = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / std::max(520.0, pf3)) / 0.50, 2.0));
                        double mid_u = smoothstep01((hz - std::max(260.0, pf1 * 1.25)) / std::max(260.0, pf2 - pf1));
                        double high_u = smoothstep01((hz - std::max(520.0, pf2 * 1.18)) / std::max(360.0, pf3 - pf2));
                        double local_ratio = ratio_low * (1.0 - mid_u) + ratio_mid * mid_u;
                        local_ratio = local_ratio * (1.0 - high_u) + ratio_high * high_u;
                        double formant_pull = std::clamp(0.10 * f1_bell + 0.54 * f2_bell + 0.62 * f3_bell, 0.0, 1.0);
                        local_ratio = local_ratio * (1.0 + formant_pull * (local_ratio - 1.0) * 0.66);
                        double src_k = k * (1.0 / local_ratio);
                        src_k = std::clamp(src_k, 0.0, static_cast<double>(spec_dim - 1));
                        int sk = static_cast<int>(src_k);
                        int sk2 = std::min(sk + 1, spec_dim - 1);
                        double f = src_k - sk;
                        tract_formant_warp_buf[k] = out_spec[i][sk] * (1.0 - f) + out_spec[i][sk2] * f;
                    }
                    for (int k = 0; k < spec_dim; ++k) {
                        double hz = hz_lut[k];
                        double f1_guard = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / std::max(180.0, pf1)) / 0.55, 2.0));
                        double local_blend = tract_formant_blend * (1.0 - 0.44 * f1_guard);
                        out_spec[i][k] = out_spec[i][k] * (1.0 - local_blend)
                                       + tract_formant_warp_buf[k] * local_blend;
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
                std::clamp(0.92 - 1.62 * (vtw_eff * (0.20 + 0.80 * vtw_drive)), 0.13, 2.45);
            if (tract_q_state < 0.0) tract_q_state = focus_sigma_target;
            double focus_tau_ms = in_consonant ? 12.0 : (in_transition ? 18.0 : 24.0);
            focus_tau_ms *= std::clamp(1.10 - 0.35 * voiced_eff, 0.80, 1.30);
            double a_q = smooth_alpha_ms(frame_period, focus_tau_ms);
            tract_q_state += a_q * (focus_sigma_target - tract_q_state);
            double focus_sigma = std::clamp(tract_q_state, 0.16, 2.20);

            double vtr_form = (1.10 * vtr_pos - 0.82 * vtr_neg) * (0.92 + 0.64 * vtr_drive);
            double c1_target = std::clamp(
                pf1 + 0.50 * (720.0 - pf1) + 2320.0 * vtr_form + 220.0 * mo_open - 480.0 * mo_close,
                150.0, 3000.0);
            double c2_target = std::clamp(
                pf2 + 0.42 * (1580.0 - pf2) + 3480.0 * vtr_form + 200.0 * mo_open - 940.0 * mo_close,
                380.0, 7600.0);
            double c3_target = std::clamp(
                pf3 + 0.36 * (2820.0 - pf3) + 4420.0 * vtr_form - 460.0 * mo_close,
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

                double fn = fn_lut[k];
                double hz = hz_lut[k];
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
                // Vtw: 공명 폭/포커스 (위치 이동 없이 Q/대역폭 중심)
                double vtw_pos = std::max(0.0, vtw_eff);
                double vtw_neg = std::max(0.0, -vtw_eff);
                double vtw_hi_term = high * (in_transition ? 0.24 : (in_consonant ? 0.10 : 1.0));
                double valley12 = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / std::sqrt(c1 * c2)) / 0.50, 2.0));
                double valley23 = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / std::sqrt(c2 * c3)) / 0.54, 2.0));
                double formant_sum = std::clamp(f1 + f2 + f3, 0.0, 1.35);
                double valley_sum = std::clamp(valley12 + valley23, 0.0, 1.35);
                double db_vtw = k_vtw * boost_vtw * vtw_drive * (
                    vtw_pos * (16.8 * (1.26 * formant_sum - 1.44 * valley_sum - 0.38 * vtw_spread + 0.22 * vtw_hi_term)) +
                    vtw_neg * (13.8 * (1.05 * valley_sum + 0.72 * vtw_spread - 0.92 * formant_sum - 0.18 * vtw_hi_term))
                );
                db_vtw = sat_db(db_vtw, 20.0);
                // Vc: 협착 (epilaryngeal twang + antiresonance)
                double vc_center = std::clamp(0.52 * c2 + 0.36 * c3 + 260.0, 1500.0, 4700.0);
                double vc_anti_center = std::clamp(0.48 * c2 + 120.0, 560.0, 2200.0);
                double vc_core_bw = std::clamp(0.66 - 0.30 * vc_drive, 0.30, 0.66);
                double vc_anti_bw = std::clamp(0.82 - 0.22 * vc_drive, 0.44, 0.82);
                double vc_core = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / vc_center) / vc_core_bw, 2.0));
                double vc_anti = std::exp(-0.5 * std::pow(std::log2((hz + 120.0) / vc_anti_center) / vc_anti_bw, 2.0));
                double vc_edge_c = std::clamp(0.78 * c3 + 900.0, 2600.0, 7600.0);
                double vc_box_c = std::clamp(0.72 * c2, 900.0, 2400.0);
                double vc_sizzle_c = std::clamp(1.28 * c3, 5200.0, 11000.0);
                double vc_edge = std::exp(-0.5 * std::pow((hz - vc_edge_c) / std::clamp(0.24 * vc_edge_c, 700.0, 1500.0), 2.0));
                double vc_box = std::exp(-0.5 * std::pow((hz - vc_box_c) / std::clamp(0.40 * vc_box_c, 460.0, 920.0), 2.0));
                double vc_sizzle_guard = std::exp(-0.5 * std::pow((hz - vc_sizzle_c) / std::clamp(0.20 * vc_sizzle_c, 980.0, 1900.0), 2.0));
                double vc_twang = std::exp(-0.5 * std::pow((hz - std::clamp(0.62 * c3 + 420.0, 2200.0, 5200.0)) /
                                                  std::clamp(0.20 * c3, 520.0, 1200.0), 2.0));
                double vc_shape = 2.05 * vc_core + 1.28 * vc_twang + 0.92 * constr_hi + 0.72 * vc_edge + 0.36 * constr_top
                                - 1.56 * vc_anti - 0.78 * constr_mid - 0.58 * constr_low - 0.58 * vc_box - 0.80 * vc_sizzle_guard;
                double harmonic_gate = std::clamp(0.52 + 0.48 * (1.0 - out_ap[i][k]), 0.40, 1.0);
                double db_vc = k_vc * boost_vc * vc_drive * harmonic_gate * (15.8 * vc_shape);
                db_vc = sat_db(db_vc, 25.0);
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

                // Source(성대원) + tube-response 레이어:
                // tract_lite_mode에서는 본 레이어를 줄여 플래그 렌더 속도를 높인다.
                double harm_hi = 0.0;
                double harm_mid = 0.0;
                double src_voiced = std::clamp(0.24 + 0.76 * voiced_eff, 0.0, 1.0);
                double db_src_vtr = 0.0;
                double db_src_vtw = 0.0;
                double db_src_vc = 0.0;
                double db_src_nn = 0.0;
                double db_tube = 0.0;
                double tube_low_puff = 0.0;
                double mix_tube = 0.0;
                double mix_src_vtr = 0.0;
                double mix_src_vtw = 0.0;
                double mix_src_vc = 0.0;
                double mix_src_nn = 0.0;
                if (!tract_lite_mode) {
                    double f0_ref = std::max(80.0, out_f0[i]);
                    double harm_idx = hz / f0_ref;
                    harm_hi = std::clamp((harm_idx - 3.0) / 16.0, 0.0, 1.0);
                    harm_mid = std::exp(-0.5 * std::pow((harm_idx - 8.0) / 4.2, 2.0));

                    db_src_vtr =
                        vtr_pos * vtr_drive * (6.2 * (1.40 * harm_hi + 0.78 * f3 - 0.62 * f1))
                      - vtr_neg * vtr_drive * (6.8 * (1.12 * harm_hi + 0.72 * f2 + 0.52 * f3));
                    db_src_vtr = sat_db(db_src_vtr, 12.0);

                    db_src_vtw = vtw_drive * (
                        vtw_pos * (4.8 * harm_mid - 5.4 * vtw_spread) +
                        vtw_neg * (-4.2 * harm_mid + 5.2 * vtw_spread)
                    );
                    db_src_vtw = sat_db(db_src_vtw, 9.5);

                    db_src_vc = vc_drive * src_voiced *
                        (6.2 * vc_core + 4.6 * vc_twang + 3.6 * constr_hi - 3.2 * constr_low - 2.8 * vc_anti);
                    db_src_vc = sat_db(db_src_vc, 13.5);

                    db_src_nn = (0.30 + 0.70 * voiced_eff) *
                        (nn_pos_eff * (3.5 * nasal_form + 2.4 * nasal_low + 1.5 * nasal_bridge
                                      - 3.2 * nasal_notch1 - 2.6 * nasal_notch2)
                       + nn_neg_eff * (-2.2 * nasal_form - 1.6 * nasal_low - 1.0 * nasal_bridge
                                      + 1.8 * nasal_notch1 + 1.4 * nasal_notch2 + 1.2 * f2));
                    db_src_nn = sat_db(db_src_nn, 10.8);

                    db_tube = tract_tube_response_db(
                        hz, fs, vtl_eff, vtr_eff, vtw_eff, vc_drive,
                        nn_pos_eff, nn_neg_eff, mo_eff, voiced_eff,
                        in_consonant, in_transition);
                    tube_low_puff = hz_bell(hz, 120.0, 110.0);
                    double tube_ctrl = std::max({
                        std::fabs(vtl_eff), std::fabs(vtr_eff), std::fabs(vtw_eff),
                        vc_drive, nn_amt, std::fabs(mo_eff), tn_drive
                    });
                    mix_tube = std::clamp((0.08 + 0.42 * std::pow(tube_ctrl, 0.72)) *
                                          (0.34 + 0.66 * voiced_eff) *
                                          (in_consonant ? 0.42 : (in_transition ? 0.72 : 1.0)),
                                          0.0, 0.56);
                    mix_tube = std::clamp(mix_tube + 0.10 * tn_drive, 0.0, 0.62);

                    mix_src_vtr = std::clamp((0.16 + 0.66 * vtr_drive) * src_voiced, 0.0, 0.92);
                    mix_src_vtw = std::clamp((0.10 + 0.52 * vtw_drive) * src_voiced * vtw_connect_gate, 0.0, 0.80);
                    mix_src_vc  = std::clamp((0.18 + 0.74 * vc_drive) * src_voiced, 0.0, 0.94);
                    mix_src_nn  = std::clamp((0.11 + 0.60 * nn_drive) * (0.30 + 0.70 * voiced_eff), 0.0, 0.84);
                    double tract_src_boost = 1.16;
                    mix_src_vtr = std::clamp(mix_src_vtr * tract_src_boost, 0.0, 0.96);
                    mix_src_vtw = std::clamp(mix_src_vtw * tract_src_boost, 0.0, 0.84);
                    mix_src_vc  = std::clamp(mix_src_vc  * tract_src_boost, 0.0, 0.96);
                    mix_src_nn  = std::clamp(mix_src_nn  * tract_src_boost, 0.0, 0.90);
                }

                // 모듈별 독립 적용:
                // 공통 dB 합산 대신 각 모듈을 순차 blend 적용해 캐릭터 섞임을 줄인다.
                double p = p0;
                p = apply_module(p, db_mo,  mix_mo, 0.50, 0.58, 1.98);
                p = apply_module(p, db_vtr, mix_vtr, 1.36 + 0.36 * vtr_drive, 0.22, 5.20);
                p = apply_module(p, db_vtw, mix_vtw, 1.22 + 0.66 * vtw_drive, 0.24, 4.50);
                p = apply_module(p, db_vc,  mix_vc,  1.40 + 0.34 * vc_drive, 0.22, 5.40);
                p = apply_module(p, db_nn,  mix_nn,  1.10 + 0.42 * nn_drive, 0.24, 4.30);
                p = apply_module(p, db_src_vtr, mix_src_vtr, 0.88, 0.46, 3.05);
                p = apply_module(p, db_src_vtw, mix_src_vtw, 1.04, 0.40, 3.30);
                p = apply_module(p, db_src_vc,  mix_src_vc,  0.96, 0.44, 3.20);
                p = apply_module(p, db_src_nn,  mix_src_nn,  0.80, 0.50, 2.66);
                p = apply_module(p, db_tube, mix_tube, 0.78, 0.46, 2.95);
                out_spec[i][k] = std::max(0.0, p);
                e1 += out_spec[i][k];

                double ap_delta = 0.0;
                // 모듈별 AP 연동 (source character 반영)
                double ap_vc = -vc_drive * (0.010 + 0.035 * vc_core + 0.030 * vc_twang + 0.018 * constr_hi)
                             + vc_drive * (0.008 * vc_anti + 0.006 * constr_low);
                double ap_nn = nn_pos_eff * (0.004 + 0.018 * nasal_form + 0.008 * nasal_low
                                            - 0.017 * nasal_notch1 - 0.012 * nasal_notch2)
                              - nn_neg_eff * (0.004 + 0.016 * nasal_form + 0.008 * nasal_low);
                double ap_src_vtr = (vtr_pos * 0.010 - vtr_neg * 0.014) * (0.30 + 0.70 * harm_hi);
                double ap_src_vtw = (-vtw_pos * (0.010 + 0.026 * harm_mid) + vtw_neg * (0.004 + 0.010 * harm_mid)) * src_voiced * vtw_connect_gate;
                double ap_src_vc = -vc_drive * src_voiced * (0.018 + 0.045 * vc_core + 0.036 * vc_twang + 0.020 * constr_hi);
                double ap_src_nn = nn_pos_eff * (0.006 + 0.020 * nasal_form + 0.010 * nasal_low
                                                - 0.012 * nasal_notch1)
                                  - nn_neg_eff * (0.006 + 0.018 * nasal_form + 0.008 * nasal_low);
                ap_delta += ap_vc * (0.08 + 0.18 * mix_vc);
                ap_delta += ap_nn * (0.08 + 0.18 * mix_nn);
                ap_delta += ap_src_vtr;
                ap_delta += ap_src_vtw;
                ap_delta += ap_src_vc;
                ap_delta += ap_src_nn;
                ap_delta -= mix_tube * (0.004 + 0.018 * high + 0.010 * tube_low_puff);
                if (std::fabs(vtw_eff) > 0.01 && (in_consonant || in_transition)) {
                    double vtw_air_guard = std::pow(std::clamp(std::fabs(vtw_eff), 0.0, 1.0), 0.62);
                    double hi_air = std::pow(std::clamp((fn - 0.46) / 0.54, 0.0, 1.0), 1.20);
                    ap_delta -= vtw_air_guard * (in_transition ? 0.040 : 0.026) * hi_air;
                }
                ap_delta = std::clamp(ap_delta, -0.055, 0.046);
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
            constexpr double attack_win_ms = 28.0;
            if (out_time_ms <= attack_win_ms) {
                double u = 1.0 - std::clamp(out_time_ms / attack_win_ms, 0.0, 1.0);
                double a_pos = std::max(0.0, at);
                double a_neg = std::max(0.0, -at);
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = fn_lut[k];
                    double hz = hz_lut[k];
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
        // T+/- 극단에서 먹먹함/샤함이 나지 않도록
        // 존재감 대역 중심으로 움직이고 초고역/저중역을 과하게 밀지 않는다.
        if (std::fabs(tension_eff) > 0.01) {
            // 성도 레이어에서 주요 캐릭터를 만들고, 여기서는 glottal source 보조만 수행.
            double src_mix = std::clamp(0.42 + 0.20 * (1.0 - tension_tract_strength), 0.38, 0.62);
            double t_pos = tension_pos * src_mix * (1.0 + 0.08 * tension_hi_knee);
            // Tn-는 질감은 유지하되 프레임 간 급변을 줄이기 위해 소스 강도를 완화.
            double t_neg = tension_neg * src_mix * (0.82 + 0.06 * tension_neg_raw);
            t_pos = std::clamp(t_pos, 0.0, 0.92);
            t_neg = std::clamp(t_neg, 0.0, 0.82);
            double e0 = 0.0;
            double e1 = 0.0;
            for (int k = 0; k < spec_dim; ++k) {
                double p0 = std::max(0.0, out_spec[i][k]);
                e0 += p0;

                // spectral effort:
                // - pressed(T+): 2~5k 존재감 + 상부 선명도 보강, 저중역 과중 억제
                // - relaxed(T-): 기존과 유사하게 존재감/긴장도 완화
                double presence = curve_lut.presence_2800[k];
                double mid = curve_lut.mid_1300[k];
                double body = curve_lut.body_900[k];
                double low = curve_lut.low_650[k];
                double hi = curve_lut.hi_42[k];
                double air = curve_lut.air_55[k];
                double top = curve_lut.top_68[k];

                // Tn-: "힘 빠짐"은 유지하되 과도한 먹먹함을 막기 위해
                // 존재감 감쇠를 완화하고, 바디를 일부 되돌려 발음 중심을 보존한다.
                double relax_cut = t_neg * (3.2 * presence
                                          + 1.0 * mid
                                          + 0.35 * hi
                                          + 0.20 * top);
                double relax_body = t_neg * (0.75 * body + 0.25 * low);
                double relax_clarity = t_neg * (1.9 * presence + 1.35 * hi + 0.35 * air - 0.18 * top);
                double press_body = t_pos * (1.0 * body + 0.65 * mid);
                double press_focus = t_pos * (7.0 * presence + 2.1 * mid + 1.8 * hi + 0.35 * air
                                            - 2.0 * top - 0.5 * low);
                double db = press_focus + press_body - relax_cut + relax_body + relax_clarity;
                db = std::clamp(db, -8.5, 8.5);
                out_spec[i][k] = p0 * std::pow(10.0, db / 10.0); // power
                e1 += out_spec[i][k];

                // aperiodicity effort:
                // - pressed(T+): 저/중역 AP 감소는 유지하되, 상부 AP를 약간 살려 먹먹함 방지
                // - relaxed(T-): 전대역 AP 증가
                double low_mid = curve_lut.low_mid_018[k];
                double ap_delta = 0.0;
                ap_delta -= t_pos * (0.105 * low_mid + 0.030 * (1.0 - hi));
                ap_delta += t_pos * (0.010 * hi - 0.040 * top);
                // T-는 숨 섞임 체감은 남기되, 초고역 hiss가 튀지 않게 top을 별도 억제.
                ap_delta += t_neg * (0.016 + 0.014 * body + 0.026 * hi);
                ap_delta -= t_neg * (0.055 * top + 0.018 * low_mid);
                out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
            }

            // T+에서 과도한 피크 상승을 막기 위해
            // 프레임 에너지를 맞추되 crest를 약하게 가드.
            if (e0 > 1.0e-12 && e1 > 1.0e-12) {
                double norm_raw = std::clamp(e0 / e1, 0.88, 1.16);
                // 텐션은 음색 변화가 주 목적이므로 레벨 보정은 부분적으로만 적용한다.
                // 프레임별 강한 sag/crest 보정은 작은 음량 펌핑으로 들릴 수 있다.
                double norm = 1.0 + 0.45 * (norm_raw - 1.0);
                double tone_guard = std::pow(10.0, (-0.10 * t_pos - 0.06 * t_neg) / 10.0);
                norm = std::clamp(norm * tone_guard, 0.92, 1.08);
                for (int k = 0; k < spec_dim; ++k) out_spec[i][k] *= norm;
            }
        }

        // 6.2. Growl(Gr): 저중역 rasp + 비주기성 강화
        if (growl_amt > 0.01) {
            double growl_voicing = 0.30 + 0.70 * voiced_eff;
            double gr_lfo = 0.82 + 0.18 * std::sin(2.0 * math::PI * (0.0072 * i));
            double g = growl_amt * growl_voicing * gr_lfo * global_artifact_guard;
            for (int k = 0; k < spec_dim; ++k) {
                double fn = fn_lut[k];
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

        // 6.3. Voiced Growl(Vg): 유성 구간 전용 저역 맥동 + throat rasp
        if (voiced_growl_amt > 0.01 && voiced_eff > 0.18) {
            double vg_gate = voiced_growl_amt * std::clamp((voiced_eff - 0.18) / 0.82, 0.0, 1.0) *
                             global_artifact_guard;
            double pulse = 0.62 + 0.38 * std::sin(2.0 * math::PI * (0.0180 * i + 0.26 * std::sin(0.0027 * i)));
            double vg = vg_gate * pulse;
            for (int k = 0; k < spec_dim; ++k) {
                double hz = hz_lut[k];
                double fn = fn_lut[k];
                double sub = std::exp(-0.5 * std::pow((hz - 120.0) / 95.0, 2.0));
                double throat = std::exp(-0.5 * std::pow((hz - 620.0) / 360.0, 2.0));
                double rasp_low = std::exp(-0.5 * std::pow((hz - 1150.0) / 520.0, 2.0));
                double rasp_mid = std::exp(-0.5 * std::pow((hz - 2100.0) / 760.0, 2.0));
                double hiss_guard = std::clamp((fn - 0.48) / 0.52, 0.0, 1.0);

                double db = vg * (6.4 * sub + 7.2 * throat + 5.8 * rasp_low + 3.2 * rasp_mid - 3.2 * hiss_guard);
                out_spec[i][k] *= std::pow(10.0, db / 10.0);

                double ap_delta = vg * (0.055 + 0.105 * throat + 0.190 * rasp_low + 0.145 * rasp_mid - 0.045 * hiss_guard);
                out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
            }
        }

        // 6.7. Breathiness(Bh): airy 질감 강화(+) / 억제(-)
        if (std::fabs(breathiness_eff) > 0.01) {
            double bh_pos = std::max(0.0, breathiness_eff);
            double bh_neg = std::max(0.0, -breathiness_eff);
            for (int k = 0; k < spec_dim; ++k) {
                double fn = fn_lut[k];
                double hz = hz_lut[k];
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

        // 6.8. 의사 어미숨/보컬프라이: 노트 끝부분 전용 질감.
        {
            double breath_start_ms = out_total_ms * (2.0 / 3.0);
            double breath_ms = std::max(1.0, out_total_ms - breath_start_ms);
            double breath_u = (breath_ms > 1.0)
                ? std::clamp((out_time_ms - breath_start_ms) / breath_ms, 0.0, 1.0)
                : 0.0;
            double short_note_boost = std::clamp((420.0 - out_total_ms) / 260.0, 0.0, 1.0);
            double breath_curve = std::pow(breath_u, 1.45 - 0.70 * short_note_boost);
            double breath_gate = std::clamp((1.45 + 0.70 * short_note_boost) * end_breath_amt, 0.0, 1.0) *
                                 breath_curve;
            breath_gate *= std::clamp(global_artifact_guard + 0.08, 0.78, 1.0);

            double fry_tail_ms = 65.0 + 230.0 * fry_tail_amt;
            double fry_tail_u = (fry_tail_ms > 1.0)
                ? std::clamp((out_time_ms - (out_total_ms - fry_tail_ms)) / fry_tail_ms, 0.0, 1.0)
                : 0.0;
            double tail_shape = fry_tail_u * fry_tail_u * (3.0 - 2.0 * fry_tail_u);
            double tail_fry_gate = std::clamp(1.12 * fry_tail_amt * tail_shape, 0.0, 1.0);
            double head_window_ms = std::clamp(consonant_tgt_ms * 0.78, 12.0, 70.0);
            double head_u = (head_window_ms > 1.0)
                ? std::clamp(out_time_ms / head_window_ms, 0.0, 1.0)
                : 1.0;
            double head_shape = in_consonant
                ? (1.0 - (head_u * head_u * (3.0 - 2.0 * head_u)))
                : 0.0;
            double head_fry_gate = std::clamp(1.07 * fry_head_amt * head_shape, 0.0, 1.0);
            tail_fry_gate *= global_artifact_guard;
            head_fry_gate *= std::clamp(global_artifact_guard + 0.04, 0.74, 1.0);
            double fry_gate = std::max(head_fry_gate, tail_fry_gate);
            double high_tail = std::clamp((tail_fry_gate - 0.64) / 0.34, 0.0, 1.0);
            double high_head = std::clamp((head_fry_gate - 0.40) / 0.48, 0.0, 1.0);
            double high_fry = std::max(high_tail, 0.85 * high_head);
            double comb_phase = 2.0 * 3.14159265358979323846 * (30.0 + 18.0 * high_fry) * out_time_ms / 1000.0;
            double comb_wave = 0.5 + 0.5 * std::cos(comb_phase);
            double comb_gate = high_fry * std::pow(comb_wave, 8.0);

            if (breath_gate > 0.001 || fry_gate > 0.001) {
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = fn_lut[k];
                    double hz = hz_lut[k];
                    double air = std::exp(-0.5 * std::pow((hz - 5200.0) / 1900.0, 2.0));
                    double body = std::exp(-0.5 * std::pow((hz - 850.0) / 520.0, 2.0));
                    double creak = std::exp(-0.5 * std::pow((hz - 180.0) / 130.0, 2.0));
                    double hum = std::exp(-0.5 * std::pow((hz - 130.0) / 95.0, 2.0));
                    double mud = std::exp(-0.5 * std::pow((hz - 330.0) / 180.0, 2.0));
                    double crack = std::exp(-0.5 * std::pow((hz - 1350.0) / 680.0, 2.0));
                    double crack_hi = std::exp(-0.5 * std::pow((hz - 2450.0) / 920.0, 2.0));
                    if (breath_gate > 0.001) {
                        double fade_db = -(16.0 - 6.0 * short_note_boost) * breath_gate;
                        double air_db = breath_gate * ((12.5 + 3.5 * short_note_boost) * air
                                      + 4.0 * std::max(0.0, fn - 0.45) - 4.2 * body);
                        out_spec[i][k] *= std::pow(10.0, (fade_db + air_db) / 10.0);
                        out_ap[i][k] = std::clamp(out_ap[i][k] + breath_gate * (0.34 + 0.88 * air + 0.32 * fn), 0.0, 1.0);
                    }
                    if (fry_gate > 0.001) {
                        double comb_hz = 95.0 + 45.0 * high_fry;
                        double teeth = 0.5 + 0.5 * std::cos(2.0 * 3.14159265358979323846 * hz / comb_hz);
                        teeth = std::pow(teeth, 10.0);
                        double notch = 1.0 - teeth;
                        double crack_mix = crack + 0.72 * crack_hi;
                        double db = head_fry_gate * (1.3 * creak + 0.9 * body + 6.6 * crack + 3.9 * crack_hi + 4.2 * teeth
                                                   - 3.0 * hum - 1.5 * mud - 1.8 * std::max(0.0, fn - 0.62))
                                  + tail_fry_gate * (1.7 * creak + 0.3 * body + 4.3 * crack + 2.9 * crack_hi + 2.5 * teeth
                                                   - 7.0 * hum - 4.2 * mud - 1.4 * std::max(0.0, fn - 0.56))
                                  + high_head * (13.8 * teeth + 6.6 * crack_mix - 6.6 * notch - 2.2 * hum)
                                  + high_tail * (7.2 * teeth + 4.8 * crack_mix - 7.4 * notch - 7.0 * hum - 3.8 * mud)
                                  + comb_gate * (5.4 * teeth + 5.0 * crack_mix - 4.6 * hum - 2.0 * mud);
                        out_spec[i][k] *= std::pow(10.0, db / 10.0);
                        double ap_delta = head_fry_gate * (0.112 + 0.135 * crack_mix + 0.102 * teeth - 0.050 * air)
                                        + tail_fry_gate * (0.082 + 0.110 * crack_mix + 0.075 * teeth - 0.045 * air)
                                        + high_head * (0.21 * teeth + 0.205 * crack_mix - 0.14 * notch - 0.04 * hum)
                                        + high_tail * (0.12 * teeth + 0.145 * crack_mix - 0.15 * notch - 0.115 * hum - 0.075 * mud);
                        out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);
                    }
                }
            }
        }

        // 7. Noise (N): +추가 / -억제
        {
            double nlev = std::clamp(sp.noise_level / 100.0, -1.0, 1.0);
            if (std::fabs(nlev) > 0.01) {
                double n_pos = std::max(0.0, nlev);
                double n_neg = std::max(0.0, -nlev);
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = fn_lut[k];
                    double ap_delta = 0.0;
                    ap_delta += n_pos * (0.10 + 0.62 * std::pow(fn, 0.70));
                    ap_delta -= n_neg * (0.13 + 0.74 * std::pow(fn, 0.82));
                    out_ap[i][k] = std::clamp(out_ap[i][k] + ap_delta, 0.0, 1.0);

                    // N<0일 때 hiss가 과하게 남지 않도록 스펙트럼도 약하게 정돈.
                    if (n_neg > 0.01) {
                        double hi = std::clamp((fn - 0.45) / 0.55, 0.0, 1.0);
                        double body = std::exp(-0.5 * std::pow((fn - 0.20) / 0.20, 2.0));
                        double hiss = std::clamp((fn - 0.62) / 0.38, 0.0, 1.0);
                        double db = n_neg * (1.0 * body - 2.1 * hi - 1.0 * hiss);
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
                        double fn = fn_lut[k];
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
                            double fn = fn_lut[k];
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
            double noise_presence = std::max(base_presence, std::max(n_pos * 1.10, bh_pos * 0.95));
            if (noise_presence > 0.01) {
                double e0 = 0.0;
                double e1 = 0.0;
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = fn_lut[k];
                    double hz = hz_lut[k];
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

        // 8.8. Airy-weak voice 보정:
        // 숨소리가 많은 약한 음성에서 과한 거칠기(상부 hiss)를 줄이고 바디를 보강.
        {
            double airy_mix = std::clamp(0.65 * std::max(0.0, breathiness_eff) +
                                         0.45 * std::max(0.0, sp.noise_level / 100.0), 0.0, 1.0);
            double weak_harm = std::clamp((75.0 - sp.harmonics) / 75.0, 0.0, 1.0);
            double ctrl = airy_mix * (0.35 + 0.65 * weak_harm);
            if (ctrl > 0.02) {
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = fn_lut[k];
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
            double hu_husky = std::max(0.0, hu_eff);   // Hu > 0: husky
            double hu_bright = std::max(0.0, -hu_eff); // Hu < 0: brighter
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

                double fn = fn_lut[k];
                double hz = hz_lut[k];
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
                double hi = curve_lut.global_hi_34[k];
                double db = global_hi_tilt_db * hi;
                double puff = curve_lut.puff_90[k];
                double puff_gate = (in_consonant ? 1.0 : (in_transition ? 0.65 : 0.28));
                db -= puff_gate * 1.65 * puff;
                double room_gate = in_consonant ? 0.14 : (in_transition ? 0.38 : 1.0);
                double hz = hz_lut[k];
                double room_lowmid = std::exp(-0.5 * std::pow(std::log2((hz + 90.0) / 560.0) / 0.78, 2.0));
                double room_air = std::clamp((fn_lut[k] - 0.46) / 0.54, 0.0, 1.0);
                db -= room_gate * (0.42 * room_lowmid + 0.18 * room_air);
                out_spec[i][k] *= std::pow(10.0, db / 10.0);

                double hiss_guard = curve_lut.global_hiss_48[k];
                double ap_cut = global_hi_ap_trim * hi * hi
                              + 0.018 * hiss_guard
                              + (0.010 + 0.024 * puff_gate) * puff
                              + room_gate * (0.012 * room_lowmid + 0.010 * room_air);
                out_ap[i][k] = std::clamp(out_ap[i][k] - ap_cut, 0.0, 1.0);
            }
        }

        // 9.2. broadband noise/pop guard:
        // AP와 초저역이 프레임 단위로 튀는 경우만 약하게 눌러 바람/팝 노이즈를 줄인다.
        if (i > 0) {
            double guard_base = in_consonant ? 0.042 : (in_transition ? 0.022 : 0.006);
            guard_base += (in_consonant || in_transition ? 0.020 : 0.008) * tension_relax_click_guard;
            for (int k = 0; k < spec_dim; ++k) {
                double hi = curve_lut.guard_hi_52[k];
                double low_puff = curve_lut.low_puff_110[k];
                double w_ap = std::clamp(guard_base + 0.030 * hi
                                       + (0.024 + 0.018 * tension_relax_click_guard) * low_puff, 0.0, 0.115);
                out_ap[i][k] = out_ap[i][k] * (1.0 - w_ap) + out_ap[i - 1][k] * w_ap;
                if (low_puff > 0.04) {
                    double w_spec = std::clamp((in_consonant ? 0.050 : (in_transition ? 0.020 : 0.009)) * low_puff,
                                               0.0, 0.070);
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
                join_smooth = 0.16 + 0.24 * cs_pos + 0.09 * cs_neg;
                // C+V 단음절 무성→유성 전환: AP 클릭 억제를 위해 스무딩 강화
                if (in_transition && consonant_src_ms < 6.0 && transition_voiced_ratio < 0.40)
                    join_smooth = std::max(join_smooth, 0.22);
            } else if (in_vowel_loop && long_loop_stress > 0.12) {
                // 극단 길이 루프에서는 반복부 프레임 차이가 누적되어 거칠게 들리기 쉬우므로
                // 안정 모음 구간만 아주 약하게 시간 방향으로 붙인다.
                join_smooth = 0.014 + 0.030 * long_loop_stress + 0.014 * cs_pos;
            } else if (!has_consonant_head && out_time_ms <= 8.0) {
                // 어두 무자음 진입의 미세 경계 완화
                join_smooth = 0.08;
            }
            join_smooth *= std::clamp(1.0 - 0.52 * dense_articulation_amt, 0.36, 1.0);
            join_smooth = std::clamp(join_smooth, 0.0, 0.42);
            if (join_smooth > 1.0e-4) {
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = fn_lut[k];
                    double loop_guard = in_vowel_loop ? (1.0 - 0.45 * long_loop_stress) : 1.0;
                    double w_spec = std::clamp(join_smooth * loop_guard * (0.92 - 0.32 * fn), 0.0, 0.35);
                    double w_ap   = std::clamp(join_smooth * loop_guard * (0.70 + 0.25 * fn), 0.0, 0.35);
                    out_spec[i][k] = out_spec[i][k] * (1.0 - w_spec) + out_spec[i - 1][k] * w_spec;
                    out_ap[i][k]   = out_ap[i][k]   * (1.0 - w_ap)   + out_ap[i - 1][k]   * w_ap;
                }
            }
        }

        // 9.6. join de-click guard:
        // 연결부에서 frame 단위 스펙트럼/AP 급변이 발생하면
        // 해당 프레임만 선택적으로 눌러 "툭툭" 임펄스성 잡음을 줄인다.
        if (i > 0) {
            int jump_stride = std::max(1, spec_dim / 96);
            double jump_score = 0.0;
            int jump_count = 0;
            for (int k = 0; k < spec_dim; k += jump_stride) {
                double cur = std::max(1.0e-12, out_spec[i][k]);
                double prv = std::max(1.0e-12, out_spec[i - 1][k]);
                double dj_spec = std::fabs(std::log(cur) - std::log(prv));
                double dj_ap = std::fabs(out_ap[i][k] - out_ap[i - 1][k]);
                jump_score += dj_spec + 0.70 * dj_ap;
                ++jump_count;
            }
            if (jump_count > 0) jump_score /= static_cast<double>(jump_count);

            double join_gate = (in_consonant || in_transition) ? 1.0
                            : (out_time_ms <= 10.0 ? 0.70 : 0.22);
            double jump_thr = 0.24 - 0.06 * cs_neg + 0.06 * cs_pos;
            jump_thr -= 0.05 * tension_relax_click_guard;
            jump_thr = std::clamp(jump_thr, 0.22, 0.36);
            double declick = std::clamp((jump_score - jump_thr)
                                        * (0.44 + 0.56 * join_gate + 0.22 * tension_relax_click_guard),
                                        0.0, 0.30);
            declick *= std::clamp(1.0 - 0.42 * dense_articulation_amt, 0.44, 1.0);
            if (in_vowel_loop && long_loop_stress > 0.12) {
                declick *= std::clamp(1.0 - 0.62 * long_loop_stress, 0.30, 1.0);
            }
            if (declick > 1.0e-4) {
                for (int k = 0; k < spec_dim; ++k) {
                    double fn = fn_lut[k];
                    double w_spec = std::clamp(declick * (0.44 + 0.44 * fn), 0.0, 0.38);
                    double w_ap   = std::clamp(declick * (0.52 + 0.36 * fn), 0.0, 0.42);
                    out_spec[i][k] = out_spec[i][k] * (1.0 - w_spec) + out_spec[i - 1][k] * w_spec;
                    out_ap[i][k]   = out_ap[i][k]   * (1.0 - w_ap)   + out_ap[i - 1][k]   * w_ap;
                }
            }
        }

        // 고음 AP 프레임 간 안정화 (F0 > 500Hz 비자음 구간)
        // 배음 밀도 저하로 인한 AP per-frame 분산을 시간 방향 블렌드로 완화.
        if (i > 0 && flat_f0_hz > 500.0 && !in_consonant) {
            double hp_blend = std::clamp((flat_f0_hz - 500.0) / 400.0, 0.0, 1.0) * 0.12;
            for (int k = 0; k < spec_dim; ++k) {
                out_ap[i][k] = out_ap[i][k] * (1.0 - hp_blend) + out_ap[i - 1][k] * hp_blend;
            }
        }
    }

    // 최종 F0 프레임 jitter 미세 억제 (zero-phase FIR).
    // flat note에서는 더 강하게, 변조 노트에서는 약하게.
    // 고음(F0>500Hz)에서는 배음 밀도가 낮아 per-frame 분산이 청취됨 → radius +1.
    {
        int hi_bonus = (flat_f0_hz > 500.0) ? 1 : 0;
        if (flat_target_f0) {
            smooth_out_f0_log_zero_phase(out_f0, 4 + hi_bonus);
        } else if (has_f0_mod) {
            smooth_out_f0_log_zero_phase(out_f0, 4 + hi_bonus);
        } else if (hi_bonus > 0) {
            smooth_out_f0_log_zero_phase(out_f0, 1);
        }
    }
    // slew limiter 강도:
    // - flat note: 강하게(잔떨림 억제)
    // - bend/mod note: 약하게(음정 추종성 확보)
    double slew_cents_per_ms = flat_target_f0 ? 2.0 : (has_f0_mod ? 34.0 : 6.0);
    stabilize_out_f0(out_f0, frame_period, slew_cents_per_ms);

    // ── WORLD Synthesis ───────────────────────────────────────────────
    std::vector<double> y(output_samples, 0.0);
    std::vector<double*> spec_ptrs(out_n_frames);
    std::vector<double*> ap_ptrs  (out_n_frames);
    for (int i = 0; i < out_n_frames; ++i) {
        spec_ptrs[i] = out_spec[i];
        ap_ptrs[i]   = out_ap[i];
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

    if (verbose_log_enabled()) {
        double formant_conf_avg = (formant_conf_count > 0)
            ? (formant_conf_sum / static_cast<double>(formant_conf_count))
            : 1.0;
        double formant_conf_lo = (formant_conf_count > 0) ? formant_conf_min : 1.0;
        std::cerr << "[Resamp] WORLD synthesized: out_frames=" << out_n_frames
                  << " frame_period=" << frame_period
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
                  << " t_trk=" << tension_tract_strength
                  << " Hu=" << sp.husky_tone
                  << " hu_eff=" << hu_eff
                  << " Ns=" << sp.noise_color
                  << " ns_eff=" << ns
                  << " con_tgt=" << consonant_tgt_ms
                  << " short=" << timing_short_note_amt
                  << " short_pitch_move=" << short_pitch_move_amt
                  << " loop_mode_eff=" << effective_loop_mode
                  << " auto_short_stretch=" << (auto_short_stretch_first ? 1 : 0)
                  << " auto_mirror=" << (auto_natural_mirror_loop ? 1 : 0)
                  << " loop_wander=" << loop_wander_amt
                  << " loop_len=" << loop_len_ms
                  << " micro_stretch=" << (micro_alias_stretch ? 1 : 0)
                  << " loop_stress=" << long_loop_stress
                  << " seam_ms=" << seam_ms
                  << " endpoint_match=" << loop_endpoint_match_amt
                  << " endpoint_sustain=" << loop_endpoint_sustain_amt
                  << " anchor_cap=" << anchor_mix_cap
                  << " formant_conf_avg=" << formant_conf_avg
                  << " formant_conf_min=" << formant_conf_lo
                  << '\n';
    }
    return output;
}

} // namespace resamp::synth
