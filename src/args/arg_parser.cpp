#include "arg_parser.hpp"
#include "util/base64.hpp"
#include <stdexcept>
#include <cmath>
#include <cstdlib>
#include <cctype>
#include <algorithm>
#include <cstring>

namespace resamp {

// ── 음정 문자열 → Hz ─────────────────────────────────────────────────────
double pitch_str_to_hz(const std::string& s) {
    if (s.empty() || s == "R") return 0.0; // rest

    // 음정 클래스 인덱스 (C=0, C#=1, D=2, ..., B=11)
    static const struct { const char* name; int idx; } notes[] = {
        {"C#", 1}, {"Db", 1}, {"D#", 3}, {"Eb", 3},
        {"F#", 6}, {"Gb", 6}, {"G#", 8}, {"Ab", 8},
        {"A#", 10},{"Bb", 10},
        {"C",  0}, {"D",  2}, {"E",  4}, {"F",  5},
        {"G",  7}, {"A",  9}, {"B",  11},
    };

    int note_idx = -1;
    size_t pos   = 0;
    for (auto& n : notes) {
        size_t len = std::strlen(n.name);
        if (s.size() > len && s.substr(0, len) == n.name) {
            note_idx = n.idx;
            pos = len;
            break;
        }
    }
    if (note_idx < 0)
        throw std::invalid_argument("Unknown note: " + s);

    // 옥타브 파싱 (음수도 허용: C-1)
    int octave = std::stoi(s.substr(pos));

    // MIDI 노트 번호: C4=60, A4=69
    // midi = (octave+1)*12 + note_idx
    int midi = (octave + 1) * 12 + note_idx;

    return 440.0 * std::pow(2.0, (midi - 69) / 12.0);
}

// ── CLI 파싱 ──────────────────────────────────────────────────────────────
RenderParams parse_args(int argc, char** argv) {
    if (argc < 5)
        throw std::runtime_error(
            "Usage: resamp <in.wav> <out.wav> <pitch> <velocity> "
            "[flags] [offset] [length] [consonant] [cutoff] "
            "[volume] [modulation] [tempo] [pitch_bend]");

    RenderParams p;
    p.input_wav  = argv[1];
    p.output_wav = argv[2];
    p.pitch_str  = argv[3];
    p.velocity   = std::atoi(argv[4]);
    p.target_hz  = pitch_str_to_hz(p.pitch_str);

    auto get_str = [&](int i) -> std::string {
        return (i < argc) ? argv[i] : "";
    };
    auto get_dbl = [&](int i, double def) -> double {
        if (i >= argc) return def;
        std::string v = argv[i];
        return v.empty() ? def : std::stod(v);
    };
    auto get_int = [&](int i, int def) -> int {
        if (i >= argc) return def;
        std::string v = argv[i];
        return v.empty() ? def : std::stoi(v);
    };
    auto has_alpha = [](const std::string& s) -> bool {
        for (unsigned char ch : s) {
            if (std::isalpha(ch)) return true;
        }
        return false;
    };
    auto trim_copy = [](std::string s) -> std::string {
        auto is_space = [](unsigned char ch) { return std::isspace(ch) != 0; };
        while (!s.empty() && is_space(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
        while (!s.empty() && is_space(static_cast<unsigned char>(s.back()))) s.pop_back();
        return s;
    };
    auto looks_number = [](const std::string& s) -> bool {
        if (s.empty()) return false;
        char* end = nullptr;
        std::strtod(s.c_str(), &end);
        return end != s.c_str() && *end == '\0';
    };

    // OpenUtau/UTAU 구현에 따라 flags 인자가 생략되어
    // optional 인자가 한 칸 당겨지는 경우가 있다.
    // arg[5]에 알파벳이 없고 숫자 형태면 "flags 생략"으로 간주한다.
    int opt_base = 5;
    if (argc > 5) {
        std::string a5 = trim_copy(get_str(5));
        bool flags_present = false;
        if (a5 == "_") {
            flags_present = true; // placeholder로 전달된 경우
        } else if (has_alpha(a5)) {
            flags_present = true; // 정상 플래그 문자열
        } else if (looks_number(a5)) {
            flags_present = false; // flags 생략 + optional 당겨짐
        } else {
            // 공백/기타 특수 문자열은 flags로 간주하지 않고
            // optional 인자 시작점으로 처리해 쉬프트 오판을 줄인다.
            flags_present = false;
        }

        if (flags_present) {
            p.flags = a5;
            if (p.flags == "_") p.flags.clear();
            opt_base = 6;
        } else {
            p.flags.clear();
            opt_base = 5;
        }
    }

    p.offset_ms    = get_dbl(opt_base + 0, 0.0);
    p.length_ms    = get_dbl(opt_base + 1, 500.0);
    p.consonant_ms = get_dbl(opt_base + 2, 0.0);
    p.cutoff_ms    = get_dbl(opt_base + 3, 0.0);
    p.volume       = get_int(opt_base + 4, 100);
    p.modulation   = get_int(opt_base + 5, 0);

    // tempo: "!120" 또는 숫자
    if (argc > (opt_base + 6)) {
        std::string t = argv[opt_base + 6];
        if (!t.empty() && t[0] == '!') t = t.substr(1);
        if (!t.empty()) p.tempo = std::stod(t);
    }

    // pitch_bend (base64)
    if (argc > (opt_base + 7)) {
        std::string pb = argv[opt_base + 7];
        if (!pb.empty() && pb != "AA==" && pb != "AA")
            p.pitch_bend = base64::decode_pitch_bend_cents(pb);
    }

    return p;
}

} // namespace resamp
