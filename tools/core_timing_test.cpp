#include "synth/pitch_mapper.hpp"
#include "synth/world_synth.hpp"
#include "post/volume.hpp"
#include "util/math_util.hpp"
#include "util/base64.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

static int checks = 0;
static void check(bool ok, const char* message) {
    ++checks;
    if (!ok) throw std::runtime_error(message);
}
static double cents(double hz) { return 1200.0 * std::log2(hz / 440.0); }

int main() {
    try {
        resamp::RenderParams p;
        resamp::SynthParams sp;
        for (int v : {0, 50, 100, 150, 200})
            check(std::abs(resamp::math::velocity_duration_scale(v) -
                std::exp2(1.0 - v / 100.0)) < 1e-12, "velocity ratio");
        p.pitch_bend.resize(49);
        for (int i = 0; i < 49; ++i) p.pitch_bend[i] = 25 * i;
        for (int bpm : {60, 120, 240}) {
            p.tempo = bpm;
            auto a = resamp::synth::make_f0_contour(p, sp, 48000, 48000);
            auto b = resamp::synth::make_f0_contour(p, sp, 60000, 48000);
            check(std::abs(cents(a[12000]) - std::min(1200.0, bpm * 10.0)) < 1e-8,
                  "tempo / 5 tick timing at 250ms");
            check(std::equal(a.begin(), a.end(), b.begin()), "padding changed pitch timing");
            check(std::abs(cents(b.back()) - 1200.0) < 1e-8, "last bend not held");
        }
        p.pitch_bend = resamp::base64::decode_pitch_bend_cents("Bk#48#");
        check(p.pitch_bend.size() == 49, "RLE size");
        auto constant = resamp::synth::make_f0_contour(p, sp, 48000, 48000);
        check(std::abs(cents(constant.front()) - 100.0) < 1e-8 &&
              std::abs(cents(constant.back()) - 100.0) < 1e-8, "constant RLE pitch");
        p.tempo = 120;
        p.pitch_bend.resize(193);
        for (int i = 0; i < 193; ++i)
            p.pitch_bend[i] = static_cast<int>(std::round(80 * std::sin(i * 5.0 / 960.0 * 2 * resamp::math::PI * 6)));
        auto vibrato = resamp::synth::make_f0_contour(p, sp, 48001, 48000);
        for (int i = 0; i < 193; ++i)
            check(std::abs(cents(vibrato[i * 250]) - p.pitch_bend[i]) < 1e-8,
                  "vibrato value was flattened or delayed");
        for (double tempo : {0.0, -120.0, std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()}) {
            p.tempo = tempo;
            bool threw = false;
            try { resamp::synth::make_f0_contour(p, sp, 10, 48000); }
            catch (const std::invalid_argument&) { threw = true; }
            check(threw, "invalid tempo accepted");
        }

        std::vector<float> edge(44100, 0.20f);
        std::fill(edge.begin(), edge.begin() + 1500, 0.08f);
        std::fill(edge.end() - 1500, edge.end(), 0.08f);
        auto original = edge;
        resamp::post::apply_boundary_level_guard(edge, 44100);
        bool no_boost = true;
        for (size_t i = 0; i < edge.size(); ++i)
            no_boost &= std::abs(edge[i]) <= std::abs(original[i]) + 1e-7f;
        check(no_boost, "boundary boosted decay");

        // Two voiced syllables separated by silence, plus analysis-only postroll.
        resamp::synth::WorldAnalysis w;
        w.fs = 16000; w.fft_size = 512; w.frame_period = 5; w.n_frames = 105;
        w.f0.assign(105, 220);
        w.temporal_positions.resize(105);
        w.spectrogram.assign(105, std::vector<double>(257, 0.001));
        w.aperiodicity.assign(105, std::vector<double>(257, 0.1));
        w.formant_peaks.assign(105, {700, 1500, 2600, 3800});
        w.formant_confidence.assign(105, 0);
        for (int i = 0; i < 105; ++i) {
            w.temporal_positions[i] = i * 0.005;
            if (i >= 35 && i < 50) w.f0[i] = 0;
            for (int k = 0; k < 257; ++k)
                w.spectrogram[i][k] *= 1 + 10 * std::exp(-std::pow((k - 30.0) / 15, 2));
        }
        p = resamp::RenderParams{};
        p.consonant_ms = 20; p.source_origin_ms = 5; p.source_end_ms = 500;
        std::vector<double> target(16000, 440);
        std::ostringstream log;
        auto* old = std::cerr.rdbuf(log.rdbuf());
        auto a = resamp::synth::world_render(w, target, p, sp, 16000);
        std::cerr.rdbuf(old);
        const std::string output = log.str();
        const auto at = output.find("loop_end_ms=");
        check(at != std::string::npos, "timing diagnostic missing");
        check(std::stod(output.substr(at + 12)) < 175, "loop includes second syllable");
        for (int i = 101; i < 105; ++i) {
            w.f0[i] = 1000;
            w.spectrogram[i].assign(257, 1000);
            w.aperiodicity[i].assign(257, 0.95);
        }
        auto b = resamp::synth::world_render(w, target, p, sp, 16000);
        check(a == b, "analysis postroll entered playback");
        check(std::all_of(a.begin(), a.end(), [](float v) { return std::isfinite(v); }),
              "non-finite rendered samples");
        for (int v : {0, 50, 100, 150, 200}) {
            p.velocity = v; p.consonant_ms = 100;
            log.str(""); log.clear(); old = std::cerr.rdbuf(log.rdbuf());
            resamp::synth::world_render(w, std::vector<double>(2400, 440), p, sp, 2400);
            std::cerr.rdbuf(old);
            const auto pos = log.str().find("consonant_tgt_ms=");
            check(pos != std::string::npos && std::abs(std::stod(log.str().substr(pos + 17)) -
                100 * resamp::math::velocity_duration_scale(v)) < 0.001,
                "short request compressed fixed consonant");
        }
        std::cout << "PASS " << checks << " checks\n";
        return 0;
    } catch (const std::exception& e) {
        std::cout << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
