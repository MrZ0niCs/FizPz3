#include "demo_state.h"
#include "benchmark.h"
#include "scenes.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
void check(bool ok, const char* message) {
    if (!ok) {
        throw std::runtime_error(message);
    }
}
void controls() {
    pz3::DemoState state;
    check(state.paused() && state.frames() == 0, "Start paused at frame zero");
    state.update(1);
    check(state.frames() == 0, "Pause freezes time");
    state.singleStep();
    check(state.frames() == 1 && state.world().bodies()[0].velocity.x > 0, "Step while paused tunnels in discrete");
    for (auto mode : {pz3::Mode::Discrete, pz3::Mode::Ccd, pz3::Mode::MultiRate, pz3::Mode::Pure240}) {
        state.selectMode(mode);
        check(state.frames() == 0 && state.world().mode() == mode, "Mode resets frame counter");
        check(state.world().bodies()[0].bounds.center.x == state.config().wallProjectilePosition.x,
              "Mode restores identical scene");
        state.singleStep();
        if (mode != pz3::Mode::Discrete)
            check(state.world().bodies()[0].velocity.x < 0, "Mode switch and one step bounce");
    }
    state.toggleScene();
    check(state.scene() == pz3::SceneId::Combined && state.frames() == 0, "Scene switch resets");
    check(state.world().bodies().size() == state.config().staticCount + state.config().normalCount + state.config().fastCount,
          "Combined scene loaded");
    state.setPaused(false);
    state.update(state.config().normalDt() * 2);
    check(state.frames() == 2, "Playback uses fixed steps");
    state.singleStep();
    check(state.frames() == 2, "Single step only while paused");
    state.setPaused(true);
    state.update(1);
    check(state.frames() == 2, "Pause after playback");
    state.reset();
    check(state.frames() == 0 && state.stats().contacts == 0, "Reset clears metrics");
    state.toggleSlowMotion();
    state.setPaused(false);
    state.update(state.config().normalDt() * 10);
    check(state.frames() == 1, "Slow motion changes elapsed time, not dt");
    state.toggleScene();
    check(state.scene() == pz3::SceneId::Wall && state.frames() == 0, "Return to wall resets");
}
void benchmark() {
    pz3::Config c;
    const auto a = pz3::measureBenchmark(c), b = pz3::measureBenchmark(c);
    check(a.size() == 4 && b.size() == 4, "Four benchmark rows");
    for (std::size_t i = 0; i < a.size(); ++i) {
        const auto& r = a[i];
        const auto& s = b[i];
        check(r.stable && r.deterministic, "Measured stack stability and repeatability");
        check(r.tunneled == (r.mode == pz3::Mode::Discrete), "Measured wall tunnelling");
        check(r.bounced == (r.mode != pz3::Mode::Discrete), "Measured wall bounce");
        check(r.highStepsPerFrame == (i < 2 ? 0.0 : 4.0), "High-frequency step count");
        check(std::isfinite(r.millisecondsPerFrame) && r.millisecondsPerFrame > 0 &&
              std::isfinite(r.millisecondsPerScheduledStep) && r.millisecondsPerScheduledStep > 0, "Valid timings");
        check(r.stable == s.stable && r.deterministic == s.deterministic && r.tunneled == s.tunneled &&
              r.bounced == s.bounced && r.stackPeakSpeed == s.stackPeakSpeed &&
              r.stackJitter == s.stackJitter && r.stackMaxDrift == s.stackMaxDrift &&
              r.stackMaxPenetration == s.stackMaxPenetration, "Repeatable diagnostics exclude timings");
    }
    std::ostringstream text;
    pz3::printBenchmark(c, a, text);
    for (const char* field : {"Stable", "Determ.", "Tunnel", "Bounce", "ms/frame", "ms/step(avg)", "high/frame"})
        check(text.str().find(field) != std::string::npos, "Report has every required column");
    c.benchmarkMaxStackSpeed = 0;
    const auto strict = pz3::measureBenchmark(c);
    bool detected = false;
    for (const auto& row : strict) detected = detected || !row.stable;
    check(detected, "Stability is measured rather than hardcoded");
}
}
int main() {
    try {
        controls();
        benchmark();
        std::cout << "PASS controls and benchmark diagnostics\n";
    }
    catch (const std::exception& e) { std::cerr << "FAIL stage2: " << e.what() << '\n'; return 1; }
}
