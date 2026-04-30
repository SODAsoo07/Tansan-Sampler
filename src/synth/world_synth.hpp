#pragma once
#include "args/arg_parser.hpp"
#include "flags/synth_params.hpp"
#include <array>
#include <string>
#include <vector>

namespace resamp::synth {

// WORLD 분석 결과 (Harvest + CheapTrick + D4C)
struct WorldAnalysis {
    int    fs           = 0;       // 샘플링 주파수
    int    fft_size     = 0;       // CheapTrick FFT 크기
    double frame_period = 5.0;     // 프레임 주기 (ms)
    int    n_frames     = 0;
    std::vector<double> f0;                            // [n_frames]
    std::vector<double> temporal_positions;            // [n_frames] (s 단위)
    std::vector<std::vector<double>> spectrogram;      // [n_frames][fft_size/2+1]
    std::vector<std::vector<double>> aperiodicity;     // [n_frames][fft_size/2+1]
    std::vector<std::array<double, 4>> formant_peaks;  // [n_frames] F1..F4 peak Hz
    std::vector<double> formant_confidence;            // [n_frames] 0..1 peak tracking reliability
};

// 입력 신호를 WORLD로 분석 (F0 + spectral envelope + aperiodicity).
// 분석 F0는 Harvest raw per-frame F0를 그대로 사용.
WorldAnalysis world_analyze(
    const std::vector<float>& signal,
    int                       sample_rate);

// 디스크 캐시를 사용한 WORLD 분석.
// cache key: source_wav_path + source file mtime/size + trim range + sample_rate
WorldAnalysis world_analyze_cached(
    const std::vector<float>& signal,
    int                       sample_rate,
    const std::string&        source_wav_path,
    int                       src_start_sample,
    int                       src_end_sample);

// 시간 매핑 + 피치/포먼트 변환 + WORLD 합성
//   src: 분석 결과 (트리밍된 소스 신호 기준)
//   target_f0_per_sample: 샘플 단위 타겟 F0 (output_samples 길이)
// 반환: float 출력 신호 (output_samples 길이)
std::vector<float> world_render(
    const WorldAnalysis&        src,
    const std::vector<double>&  target_f0_per_sample,
    const RenderParams&         params,
    const SynthParams&          sp,
    int                         output_samples);

} // namespace resamp::synth
