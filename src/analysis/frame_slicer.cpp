#include "frame_slicer.hpp"
#include "residual.hpp"
#include "pitch_detector.hpp"
#include "util/window.hpp"
#include <algorithm>
#include <cmath>

namespace resamp::analysis {

std::vector<AnalysisFrame> slice_and_analyze(
    const std::vector<float>& signal,
    const io::FrqData&        frq,
    int sample_rate, int frame_size, int hop_size, int lpc_order)
{
    int N = static_cast<int>(signal.size());
    if (N == 0) return {};

    auto win = window::hann(frame_size);

    // frq 없을 때 F0 추정 한 번에 수행
    std::vector<double> f0_fallback;
    if (!frq.valid)
        f0_fallback = estimate_f0_contour(signal, sample_rate, frame_size, hop_size);

    std::vector<AnalysisFrame> frames;
    std::vector<float> buf(frame_size, 0.0f);

    int n_frames = (N + hop_size - 1) / hop_size;

    for (int fi = 0; fi < n_frames; ++fi) {
        int center = fi * hop_size + frame_size / 2;
        int start  = fi * hop_size;

        // 윈도우 적용 복사
        for (int i = 0; i < frame_size; ++i) {
            int idx = start + i;
            buf[i] = (idx < N) ? signal[idx] * win[i] : 0.0f;
        }

        // F0 결정
        double f0 = 0.0;
        if (frq.valid) {
            f0 = io::frq_get_f0(frq, start + frame_size / 2);
        } else if (!f0_fallback.empty()) {
            int idx = std::min(start + frame_size / 2, N - 1);
            f0 = f0_fallback[idx];
        }

        // LPC 분석
        LpcCoeffs lpc = analyze_lpc(buf.data(), frame_size, lpc_order, sample_rate);

        // 잔차 추출 (윈도우 없는 원본 신호 구간 사용)
        std::vector<float> raw(frame_size);
        for (int i = 0; i < frame_size; ++i) {
            int idx = start + i;
            raw[i] = (idx < N) ? signal[idx] : 0.0f;
        }
        auto res = extract_residual(raw, lpc);

        AnalysisFrame af;
        af.center_sample = center;
        af.f0      = f0;
        af.voiced  = (f0 > 50.0);
        af.lpc     = std::move(lpc);
        af.residual = std::move(res);
        frames.push_back(std::move(af));
    }

    return frames;
}

} // namespace resamp::analysis
