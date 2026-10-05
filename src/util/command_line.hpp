#pragma once
#include <string>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#endif

namespace resamp {
inline std::vector<std::string> utf8_command_line(int argc, char** argv) {
    std::vector<std::string> args;
#ifdef _WIN32
    int count = 0;
    auto wide = CommandLineToArgvW(GetCommandLineW(), &count);
    if (wide) {
        for (int i = 0; i < count; ++i) {
            int size = WideCharToMultiByte(CP_UTF8, 0, wide[i], -1, nullptr, 0, nullptr, nullptr);
            std::string value(size, '\0');
            WideCharToMultiByte(CP_UTF8, 0, wide[i], -1, value.data(), size, nullptr, nullptr);
            if (size > 0) value.resize(size - 1);
            args.push_back(std::move(value));
        }
        LocalFree(wide);
        return args;
    }
#endif
    for (int i = 0; i < argc; ++i) args.emplace_back(argv[i] ? argv[i] : "");
    return args;
}
} // namespace resamp
