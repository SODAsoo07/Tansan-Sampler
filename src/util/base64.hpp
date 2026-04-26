#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <cctype>

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

inline int uint6(char c) {
    if (c >= 'A' && c <= 'Z') return static_cast<int>(c - 'A');
    if (c >= 'a' && c <= 'z') return static_cast<int>(c - 'a') + 26;
    if (c >= '0' && c <= '9') return static_cast<int>(c - '0') + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

// 2글자 Base64를 signed int12(-2048~2047)로 변환
inline bool to_int12(char c0, char c1, int& out) {
    int v0 = uint6(c0);
    int v1 = uint6(c1);
    if (v0 < 0 || v1 < 0) return false;
    int u12 = (v0 << 6) | v1;
    if (u12 & 0x800) u12 -= 4096;
    out = u12;
    return true;
}

// UTAU/OpenUtau pitch string(arg[13]) 디코드:
// - 우선 12bit + RLE(#) 포맷을 시도 (단위: cent)
// - 실패 시 legacy int8 base64로 폴백 (단위: 10cent)
inline std::vector<int> decode_pitch_bend_cents(const std::string& s) {
    std::vector<int> out;
    if (s.empty() || s == "AA==" || s == "AA") return out;

    auto parse_rle = [&](const std::string& src) -> bool {
        std::vector<int> tmp;
        size_t pos = 0;

        while (pos < src.size()) {
            size_t hash = src.find('#', pos);
            std::string chunk = (hash == std::string::npos)
                              ? src.substr(pos)
                              : src.substr(pos, hash - pos);

            if (!chunk.empty()) {
                // int12 stream: 2글자 단위
                if (chunk.size() % 2 != 0) return false;
                for (size_t i = 0; i + 1 < chunk.size(); i += 2) {
                    int v = 0;
                    if (!to_int12(chunk[i], chunk[i + 1], v)) return false;
                    tmp.push_back(v); // already cents
                }
            }

            if (hash == std::string::npos) break;
            pos = hash + 1;

            // '#' 뒤 숫자는 직전 값 반복 횟수
            size_t next_hash = src.find('#', pos);
            std::string rep_s = (next_hash == std::string::npos)
                              ? src.substr(pos)
                              : src.substr(pos, next_hash - pos);
            if (rep_s.empty()) return false;
            if (!std::all_of(rep_s.begin(), rep_s.end(),
                             [](unsigned char ch) { return std::isdigit(ch) != 0; })) {
                return false;
            }
            if (tmp.empty()) return false;

            int rep = std::stoi(rep_s);
            rep = std::clamp(rep, 0, 200000);
            tmp.insert(tmp.end(), rep, tmp.back());

            if (next_hash == std::string::npos) {
                pos = src.size();
            } else {
                pos = next_hash + 1;
            }
        }

        if (tmp.empty()) return false;
        out = std::move(tmp);
        return true;
    };

    if (s.find('#') != std::string::npos && parse_rle(s)) {
        return out;
    }
    // '#'가 없어도 12bit 스트림일 수 있다.
    if ((s.size() % 2 == 0) && parse_rle(s)) {
        return out;
    }

    // legacy: int8 base64(10 cents 단위)
    std::vector<int8_t> bytes = decode(s);
    out.reserve(bytes.size());
    for (int8_t b : bytes) out.push_back(static_cast<int>(b) * 10);
    return out;
}

} // namespace resamp::base64
