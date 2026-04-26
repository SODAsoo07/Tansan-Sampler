#pragma once

namespace resamp {

// 플래그 파싱 후 합성 파라미터 (물리 성도 모델 제어)
struct SynthParams {
    // ── 성도 물리 파라미터 ─────────────────────────────
    int gender      = 0;    // g: -100~+100  성도 길이 (포먼트 워핑)
    int brightness  = 50;   // Bi: 0~100     스펙트럼 기울기 (HF/LF 에너지)
    int husky_tone  = 0;    // Hu: -100~+100 허스키(+) ↔ 밝음(-) 톤
    int mouth_open  = 0;    // Mo: -100~+100 입 열림(+)/입 닫힘(-)
    int tension     = 0;    // Tn: -100~+100 성대 긴장도
    int pitch_cents = 0;    // t: -1200~+1200 노트 전체 cents 오프셋
    int growl       = 0;    // Gr: 0~100     growl/rasp 질감
    // Tract simulator layer (AVOX Throat 계열의 세분화 제어)
    int tract_length = 0;        // Vtl: -100~+100 성도 길이 이동
    int tract_resonance = 0;     // Vtr: -100~+100 공명 중심 이동
    int tract_focus = 0;         // Vtw: -100~+100 공명 폭/집중도
    int tract_constriction = 0;  // Vc: 0~100 협착 강도
    int nasal_coupling = 0;      // Nn: 0~100 비성 결합 강도
    int harmonics   = 70;   // H/Hr: 0~100   70 중립, 그 이상은 배음 강조
    int noise_level = 0;    // N: -100~+100  노이즈 추가(+)/억제(-)
    int breathiness = 0;    // Bh: -100~+100 숨소리 추가(+)/억제(-)
    int transition_length   = 100; // Tr: 0~200     VC 연결 길이 스케일
    int consonant_stability = 50;  // Cs: 0~100     자음/연결 안정화 강도
    int attack              = 0;   // At: -100~+100 어택 선명도/완화
    int release_air         = 0;   // Rl: 0~100     릴리즈 airy 강도
    int noise_color         = 0;   // Ns: -100~+100 노이즈 톤 (밝기/어둠)
    int peak_comp   = 86;   // P: 0~100      피크 제한 강도
    int voice_color = 0;    // c: -100~+100  음색 (고차 반사 계수 조정)

    static SynthParams defaults() { return {}; }
};

} // namespace resamp
