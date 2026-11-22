#include "benchmark.h"
#include <exception>
#include <iostream>

int main() {
    try { pz3::runBenchmark(pz3::Config{}, std::cout); }
    catch (const std::exception& error) {
        std::cerr << "Benchmark failed: " << error.what() << '\n';
        return 1;
    }
}
