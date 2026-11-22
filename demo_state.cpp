#include "demo_state.h"
#include "scenes.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace pz3 {
DemoState::DemoState(Config config)
    : config_(std::move(config)), world_(config_, makeWallScene(config_), mode_) {}
void DemoState::reset() {
    world_ = World(config_, scene_ == SceneId::Wall ? makeWallScene(config_) : makeCombinedScene(config_), mode_);
    accumulator_ = 0.0;
    frames_ = 0;
    lastPhysicsMs_ = 0.0;
    stats_ = {};
    error_.clear();
}
void DemoState::selectMode(Mode mode) {
    mode_ = mode;
    reset();
}
void DemoState::toggleScene() {
    scene_ = scene_ == SceneId::Wall ? SceneId::Combined : SceneId::Wall;
    reset();
}
void DemoState::setPaused(bool paused) {
    paused_ = paused;
    accumulator_ = 0.0;
}
void DemoState::toggleSlowMotion() {
    slow_ = !slow_;
    accumulator_ = 0.0;
}
void DemoState::step() {
    if (!error_.empty()) return;
    try {
        const auto start = std::chrono::steady_clock::now();
        stats_ = world_.step();
        lastPhysicsMs_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        ++frames_;
    } catch (const std::exception& e) {
        error_ = e.what();
        setPaused(true);
    }
}
void DemoState::singleStep() { 
    if (paused_) step(); 
}
void DemoState::update(double elapsedSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0)
        throw std::invalid_argument("Invalid presentation elapsed time");
    if (paused_) return;
    accumulator_ += elapsedSeconds * playbackSpeed();
    while (accumulator_ + config_.timeEpsilon >= config_.normalDt() && !paused_) {
        accumulator_ = std::max(0.0, accumulator_ - config_.normalDt());
        step();
    }
}
}
