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
    int voiced_growl = 0;   // Vg: 0~100     유성 구간 전용 growl
    // Tract simulator layer (AVOX Throat 계열의 세분화 제어)
    int tract_length = 0;        // Vtl: -100~+100 성도 길이 이동
    int tract_resonance = 0;     // Vtr: -100~+100 공명 중심 이동
    int tract_focus = 0;         // Vtw: -100~+100 공명 폭/집중도
    int tract_constriction = 0;  // Vc: 0~100 협착 강도
    int nasal_coupling = 0;      // Nn: -100~+100 비성 결합(+)/비성 억제(-)
    int harmonics   = 70;   // Hr: 0~100     70 중립, 그 이상은 배음 강조
    int noise_level = 0;    // N: -100~+100  노이즈 추가(+)/억제(-)
    int breathiness = 0;    // Bh: -100~+100 숨소리 추가(+)/억제(-)
    int consonant_stability = 50;  // Cs: 0~100     자음/연결 안정화 강도
    int attack              = 0;   // At: -100~+100 어택 선명도/완화
    int noise_color         = 0;   // Ns: -100~+100 노이즈 톤 (밝기/어둠)
    int peak_comp   = 86;   // P: 0~100      피크 제한 강도
    int loud_norm   = 15;   // Ln: -100~+100 음량 정규화 강도 (-완화 / +강화), 기본 15

    // ── 확장 플래그 ───────────────────────────────────
    int loop_mode = 1;          // Lp: 0=one-pass stretch, 1=forward loop, 2=ping-pong loop
    int consonant_power = 0;    // Cw: -100~+100 자음 강화/약화
    int vowel_power = 0;        // Vw: -100~+100 모음 강화/약화
    int reverse_mode = 0;       // Rv: 0=off, 1=full reverse
    int voicing = 0;            // Vo: -100~+100 강제 무성화/유성화
    int final_filter = 0;       // Fc: -1=low-cut, 0=off, 1=high-cut
    int end_breath = 0;         // Eb: 0~100 의사 어미숨
    int fry_head = 0;           // Fh: 0~100 선행자음부 보컬프라이
    int fry_tail = 0;           // Ft: 0~100 어미 보컬프라이
    int tremolo = 0;            // Tm: 0~100 주기적 피치 떨림
    int distortion = 0;         // Ds: 0~100 디스토션
    int bitcrusher = 0;         // Bc: 0~100 비트크러셔
    int vocalizer = 0;          // Vz: 0=off, 1=아, 2=에, 3=이, 4=오, 5=우, 6=어, 7=N

    static SynthParams defaults() { return {}; }
};

} // namespace resamp
