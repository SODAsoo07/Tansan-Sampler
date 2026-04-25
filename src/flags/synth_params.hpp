#pragma once

namespace resamp {

// 플래그 파싱 후 합성 파라미터 (물리 성도 모델 제어)
struct SynthParams {
    // ── 성도 물리 파라미터 ─────────────────────────────
    int gender      = 0;    // g: -100~+100  성도 길이 (포먼트 워핑)
    int brightness  = 50;   // B: 0~100      스펙트럼 기울기 (HF/LF 에너지)
    int tension     = 0;    // t: -100~+100  성대 긴장도
    int harmonics   = 100;  // H: 0~100      배음 수 (소스 LPF cutoff)
    int merge       = 100;  // M: 0~100      유/무성 블렌딩
    int noise_level = 0;    // N: 0~100      배경 노이즈 혼합
    int peak_comp   = 86;   // P: 0~100      피크 제한 강도
    int voice_color = 0;    // c: -100~+100  음색 (고차 반사 계수 조정)

    static SynthParams defaults() { return {}; }
};

} // namespace resamp
