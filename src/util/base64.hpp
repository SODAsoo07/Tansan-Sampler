#pragma once
#include <string>
#include <vector>
#include <cstdint>

namespace resamp::base64 {

// Base64 디코딩 → bytes
// UTAU pitch_bend 인수: base64 인코딩된 int8_t 배열
inline std::vector<int8_t> decode(const std::string& s) {
    static const char tbl[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    auto val = [&](char c) -> int {
        for (int i = 0; i < 64; ++i)
            if (tbl[i] == c) return i;
        return -1; // padding or invalid
    };

    std::vector<int8_t> out;
    out.reserve(s.size() * 3 / 4);

    for (size_t i = 0; i + 3 < s.size(); i += 4) {
        int v0 = val(s[i]);
        int v1 = val(s[i + 1]);
        int v2 = val(s[i + 2]);
        int v3 = val(s[i + 3]);

        if (v0 < 0 || v1 < 0) break;
        out.push_back(static_cast<int8_t>((v0 << 2) | (v1 >> 4)));
        if (v2 >= 0) out.push_back(static_cast<int8_t>(((v1 & 0xF) << 4) | (v2 >> 2)));
        if (v3 >= 0) out.push_back(static_cast<int8_t>(((v2 & 0x3) << 6) | v3));
    }
    return out;
}

} // namespace resamp::base64
