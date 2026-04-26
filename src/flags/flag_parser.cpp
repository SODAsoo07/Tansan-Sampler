#include "flag_parser.hpp"
#include "util/math_util.hpp"
#include <cctype>
#include <string>
#include <algorithm>

namespace resamp {

SynthParams parse_flags(const std::string& s) {
    SynthParams p = SynthParams::defaults();
    if (s.empty()) return p;

    size_t i = 0;
    while (i < s.size()) {
        // 플래그 이름 (영문자 1자 이상)
        if (!std::isalpha(static_cast<unsigned char>(s[i]))) { ++i; continue; }

        std::string name;
        while (i < s.size() && std::isalpha(static_cast<unsigned char>(s[i])))
            name += s[i++];
        std::string lname = name;
        std::transform(lname.begin(), lname.end(), lname.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

        // 값 파싱 (부호 + 숫자, 없으면 0)
        std::string num_s;
        if (i < s.size() && (s[i] == '+' || s[i] == '-'))
            num_s += s[i++];
        while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i])))
            num_s += s[i++];

        int val = num_s.empty() ? 0 : std::stoi(num_s);

        // 이름 → 파라미터 매핑
        if      (lname == "g") p.gender      = math::clamp(val, -100, 100);
        else if (lname == "bi")
                            p.brightness     = math::clamp(val, 0, 100);
        else if (lname == "hu")
                            p.husky_tone     = math::clamp(val, -100, 100);
        else if (lname == "mo")
                            p.mouth_open     = math::clamp(val, 0, 100);
        else if (lname == "tn")
                            p.tension        = math::clamp(val, -100, 100);
        else if (lname == "t")
                            p.pitch_cents    = math::clamp(val, -1200, 1200);
        else if (lname == "gr")
                            p.growl          = math::clamp(val, 0, 100);
        else if (lname == "vtl")
                            p.tract_length   = math::clamp(val, -100, 100);
        else if (lname == "vtr")
                            p.tract_resonance = math::clamp(val, -100, 100);
        else if (lname == "vtw")
                            p.tract_focus    = math::clamp(val, -100, 100);
        else if (lname == "vc")
                            p.tract_constriction = math::clamp(val, 0, 100);
        else if (lname == "nn")
                            p.nasal_coupling = math::clamp(val, 0, 100);
        else if (lname == "h" || lname == "hr")
                            p.harmonics      = math::clamp(val, 0, 100);
        else if (lname == "n") p.noise_level = math::clamp(val, 0, 100);
        else if (lname == "bh" || lname == "brh")
                            p.breathiness    = math::clamp(val, 0, 100);
        else if (lname == "tr") p.transition_length = math::clamp(val, 0, 200);
        else if (lname == "cs") p.consonant_stability = math::clamp(val, 0, 100);
        else if (lname == "at") p.attack = math::clamp(val, -100, 100);
        else if (lname == "rl") p.release_air = math::clamp(val, 0, 100);
        else if (lname == "ns") p.noise_color = math::clamp(val, -100, 100);
        else if (lname == "p") p.peak_comp   = math::clamp(val, 0, 100);
        else if (lname == "c") p.voice_color = math::clamp(val, -100, 100);
        // 알 수 없는 플래그는 무시
    }
    return p;
}

} // namespace resamp
