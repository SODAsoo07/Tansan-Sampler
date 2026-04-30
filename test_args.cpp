#include <iostream>
#include <string>
#include <vector>
#include "src/args/arg_parser.hpp"

int main() {
    const char* argv[] = {"resamp.exe", "in.wav", "out.wav", "C4", "100", "", "100", "500", "50", "-200", "100", "0"};
    auto p = resamp::parse_args(12, const_cast<char**>(argv));
    std::cout << "off=" << p.offset_ms << " len=" << p.length_ms << " con=" << p.consonant_ms << " cut=" << p.cutoff_ms << "\n";
    return 0;
}
