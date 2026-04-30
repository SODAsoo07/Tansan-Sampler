#include "flag_parser.hpp"
#include "util/math_util.hpp"
#include <cctype>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace resamp {

namespace {

constexpr const char* kPresetFileName = "tansanSampler.txt";

std::string trim_copy(const std::string& s) {
    size_t first = 0;
    while (first < s.size() && std::isspace(static_cast<unsigned char>(s[first]))) ++first;

    size_t last = s.size();
    while (last > first && std::isspace(static_cast<unsigned char>(s[last - 1]))) --last;
    return s.substr(first, last - first);
}

std::string lowercase_copy(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return s;
}

bool is_preset_name_char(char ch) {
    const unsigned char u = static_cast<unsigned char>(ch);
    return std::isalnum(u) || ch == '_' || ch == '-';
}

std::vector<std::filesystem::path> preset_candidate_paths(const std::string& executable_path) {
    std::vector<std::filesystem::path> paths;

    const char* override_path = std::getenv("RESAMP_PRESET_FILE");
    if (override_path != nullptr && *override_path != '\0') {
        paths.emplace_back(override_path);
    }

    if (!executable_path.empty()) {
        std::error_code ec;
        std::filesystem::path exe_path(executable_path);
        if (exe_path.has_parent_path()) {
            paths.push_back(std::filesystem::absolute(exe_path.parent_path(), ec) / kPresetFileName);
        }
    }

    std::error_code ec;
    paths.push_back(std::filesystem::current_path(ec) / kPresetFileName);
    return paths;
}

std::unordered_map<std::string, std::string> load_presets(const std::string& executable_path) {
    std::unordered_map<std::string, std::string> presets;

    for (const auto& path : preset_candidate_paths(executable_path)) {
        std::ifstream ifs(path);
        if (!ifs) continue;

        std::string line;
        while (std::getline(ifs, line)) {
            std::string work = trim_copy(line);
            if (work.empty() || work[0] == '#' || work[0] == ';') continue;

            size_t comment = work.find('#');
            if (comment != std::string::npos) work = trim_copy(work.substr(0, comment));
            if (work.empty()) continue;

            size_t sep = work.find('=');
            if (sep == std::string::npos) sep = work.find(':');
            if (sep == std::string::npos) continue;

            std::string name = lowercase_copy(trim_copy(work.substr(0, sep)));
            std::string flags = trim_copy(work.substr(sep + 1));
            if (!name.empty() && !flags.empty()) {
                presets[name] = flags;
            }
        }

        if (!presets.empty()) break;
    }

    return presets;
}

std::string expand_fragment(const std::string& fragment,
                            const std::unordered_map<std::string, std::string>& presets,
                            std::unordered_set<std::string>& stack);

std::string expand_preset(const std::string& name,
                          const std::unordered_map<std::string, std::string>& presets,
                          std::unordered_set<std::string>& stack) {
    const std::string lname = lowercase_copy(name);
    auto it = presets.find(lname);
    if (it == presets.end() || stack.count(lname) > 0) return std::string();

    stack.insert(lname);
    std::string expanded = expand_fragment(it->second, presets, stack);
    stack.erase(lname);
    return expanded;
}

std::string expand_fragment(const std::string& fragment,
                            const std::unordered_map<std::string, std::string>& presets,
                            std::unordered_set<std::string>& stack) {
    std::string preset_flags;
    std::string literal_flags;

    size_t i = 0;
    while (i < fragment.size()) {
        if (fragment[i] != '@') {
            literal_flags += fragment[i++];
            continue;
        }

        size_t name_begin = i + 1;
        size_t name_end = name_begin;
        while (name_end < fragment.size() && is_preset_name_char(fragment[name_end])) {
            ++name_end;
        }

        if (name_end == name_begin) {
            ++i;
            continue;
        }

        size_t match_len = 0;
        for (size_t len = name_end - name_begin; len > 0; --len) {
            const std::string candidate = lowercase_copy(fragment.substr(name_begin, len));
            if (presets.count(candidate) > 0) {
                match_len = len;
                break;
            }
        }

        if (match_len == 0) {
            ++i;
            continue;
        }

        const std::string name = fragment.substr(name_begin, match_len);
        const std::string expanded = expand_preset(name, presets, stack);
        if (!expanded.empty()) {
            preset_flags += expanded;
            preset_flags += ' ';
        }
        i = name_begin + match_len;
    }

    return preset_flags + literal_flags;
}

} // namespace

std::string expand_flag_presets(const std::string& flag_str,
                                const std::string& executable_path) {
    if (flag_str.find('@') == std::string::npos) return flag_str;

    const auto presets = load_presets(executable_path);
    if (presets.empty()) return flag_str;

    std::unordered_set<std::string> stack;
    return expand_fragment(flag_str, presets, stack);
}

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
                            p.mouth_open     = math::clamp(val, -100, 100);
        else if (lname == "tn")
                            p.tension        = math::clamp(val, -100, 100);
        else if (lname == "t")
                            p.pitch_cents    = math::clamp(val, -1200, 1200);
        else if (lname == "gr")
                            p.growl          = math::clamp(val, 0, 100);
        else if (lname == "vg" || lname == "vgrl" || lname == "vgrowl")
                            p.voiced_growl   = math::clamp(val, 0, 100);
        else if (lname == "vtl")
                            p.tract_length   = math::clamp(val, -100, 100);
        else if (lname == "vtr")
                            p.tract_resonance = math::clamp(val, -100, 100);
        else if (lname == "vtw")
                            p.tract_focus    = math::clamp(val, -100, 100);
        else if (lname == "vc" || lname == "vcs" || lname == "vcons")
                            p.tract_constriction = math::clamp(val, 0, 100);
        else if (lname == "nn" || lname == "nas" || lname == "nasal")
                            p.nasal_coupling = math::clamp(val, -100, 100);
        else if (lname == "hr")
                            p.harmonics      = math::clamp(val, 0, 100);
        else if (lname == "n") p.noise_level = math::clamp(val, -100, 100);
        else if (lname == "bh" || lname == "brh")
                            p.breathiness    = math::clamp(val, -100, 100);
        else if (lname == "cs") p.consonant_stability = math::clamp(val, 0, 100);
        else if (lname == "at") p.attack = math::clamp(val, -100, 100);
        else if (lname == "ns") p.noise_color = math::clamp(val, -100, 100);
        else if (lname == "p") p.peak_comp   = math::clamp(val, 0, 100);
        else if (lname == "ln" || lname == "lnrm")
                            p.loud_norm     = math::clamp(val, -100, 100);
        else if (lname == "lp" || lname == "loop")
                            p.loop_mode = math::clamp(val, 0, 2);
        else if (lname == "cw" || lname == "cp")
                            p.consonant_power = math::clamp(val, -100, 100);
        else if (lname == "vw" || lname == "vp")
                            p.vowel_power = math::clamp(val, -100, 100);
        else if (lname == "rv" || lname == "rev")
                            p.reverse_mode = (val > 0) ? 1 : 0;
        else if (lname == "vo" || lname == "voi")
                            p.voicing = math::clamp(val, -100, 100);
        else if (lname == "fc" || lname == "flt")
                            p.final_filter = math::clamp(val, -1, 1);
        else if (lname == "eb" || lname == "endbr")
                            p.end_breath = math::clamp(val, 0, 100);
        else if (lname == "fh")
                            p.fry_head = math::clamp(val, 0, 100);
        else if (lname == "ft")
                            p.fry_tail = math::clamp(val, 0, 100);
        else if (lname == "tm" || lname == "trem")
                            p.tremolo = math::clamp(val, 0, 100);
        else if (lname == "ds" || lname == "dist")
                            p.distortion = math::clamp(val, 0, 100);
        else if (lname == "bc" || lname == "bit")
                            p.bitcrusher = math::clamp(val, 0, 100);
        else if (lname == "vz" || lname == "voc" || lname == "vocalizer")
                            p.vocalizer = math::clamp(val, 0, 7);
        else if (lname == "vzs" || lname == "vocstr" || lname == "vocalizerstrength")
                            p.vocalizer_strength = math::clamp(val, 0, 100);
        // 알 수 없는 플래그는 무시
    }
    return p;
}

} // namespace resamp
