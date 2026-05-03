#pragma once
#include <string>
#include <vector>
#include <cstdint>

namespace resamp {

// UTAU 리샘플러 CLI 인수 전체를 담는 구조체
struct RenderParams {
    // ── 필수 인수 ─────────────────────────────────────
    std::string input_wav;     // arg[1]: 원본 wav 경로
    std::string output_wav;    // arg[2]: 출력 wav 경로
    std::string pitch_str;     // arg[3]: 음정 문자열 (예: "A4", "C#5")
    int         velocity = 100; // arg[4]: 자음 속도 (0~200, 100=100%)

    // ── 선택 인수 ─────────────────────────────────────
    std::string flags      = "";    // arg[5]: 플래그 문자열 ("_"=없음)
    double offset_ms       = 0.0;   // arg[6]: wav 시작 오프셋 (ms)
    double length_ms       = 500.0; // arg[7]: 출력 길이 (ms)
    double consonant_ms    = 0.0;   // arg[8]: 자음 영역 길이 (ms)
    double cutoff_ms       = 0.0;   // arg[9]: 끝 커팅 (ms, 음수=끝에서)
    int    volume          = 100;   // arg[10]: 볼륨 (0~200)
    int    modulation      = 0;     // arg[11]: 피치 모듈레이션 (0~200)
    double tempo           = 120.0; // arg[12]: 템포 (BPM, "!120" 형식)
    std::vector<int> pitch_bend;    // arg[13]: 디코딩된 pitch bend (cent 단위)

    // ── 파생 값 ───────────────────────────────────────
    double target_hz = 440.0; // pitch_str에서 변환된 Hz
    double source_origin_ms = 0.0; // 분석용 trim 시작점 기준 실제 offset 위치
};

// argc/argv에서 파싱
RenderParams parse_args(int argc, char** argv);

// "A4", "C#5", "Bb3" 등 → Hz
double pitch_str_to_hz(const std::string& s);

} // namespace resamp
