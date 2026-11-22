#pragma once
#include "physics.h"
#include <iosfwd>
#include <vector>
namespace pz3 {
struct BenchmarkRow {
    Mode mode = Mode::Discrete;
    bool stable = true, deterministic = true, tunneled = false, bounced = false;
    double millisecondsPerFrame = 0.0, millisecondsPerScheduledStep = 0.0;
    double normalStepsPerFrame = 0.0, fastStepsPerFrame = 0.0, highStepsPerFrame = 0.0;
    double stackPeakSpeed = 0.0, stackMaxDrift = 0.0, stackMaxPenetration = 0.0, stackJitter = 0.0;
};
std::vector<BenchmarkRow> measureBenchmark(const Config& config);
void printBenchmark(const Config& config, const std::vector<BenchmarkRow>& rows, std::ostream& output);
void runBenchmark(const Config& config, std::ostream& output);
}
