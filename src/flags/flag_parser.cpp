#include "flag_parser.hpp"
#include "util/math_util.hpp"
#include <cctype>
#include <string>

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

        // 값 파싱 (부호 + 숫자, 없으면 0)
        std::string num_s;
        if (i < s.size() && (s[i] == '+' || s[i] == '-'))
            num_s += s[i++];
        while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i])))
            num_s += s[i++];

        int val = num_s.empty() ? 0 : std::stoi(num_s);

        // 이름 → 파라미터 매핑
        if      (name == "g") p.gender      = math::clamp(val, -100, 100);
        else if (name == "B") p.brightness  = math::clamp(val, 0, 100);
        else if (name == "t") p.tension     = math::clamp(val, -100, 100);
        else if (name == "H") p.harmonics   = math::clamp(val, 0, 100);
        else if (name == "M") p.merge       = math::clamp(val, 0, 100);
        else if (name == "N") p.noise_level = math::clamp(val, 0, 100);
        else if (name == "P") p.peak_comp   = math::clamp(val, 0, 100);
        else if (name == "c") p.voice_color = math::clamp(val, -100, 100);
        // 알 수 없는 플래그는 무시
    }
    return p;
}

} // namespace resamp
