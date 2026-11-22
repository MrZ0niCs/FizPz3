#pragma once
#include "physics.h"
#include <string>

namespace pz3 {
enum class SceneId { Wall, Combined };
// управление сценой, режимом, паузой и скоростью воспроизведения
class DemoState {
public:
    explicit DemoState(Config config = {});
    void selectMode(Mode mode);
    void toggleScene();
    void reset();
    void setPaused(bool paused);
    void togglePause() { setPaused(!paused_); }
    void singleStep();
    void update(double elapsedSeconds);
    void toggleSlowMotion();
    const World& world() const { return world_; }
    const Config& config() const { return config_; }
    SceneId scene() const { return scene_; }
    bool paused() const { return paused_; }
    double playbackSpeed() const { return slow_ ? 0.1 : 1.0; }
    std::size_t frames() const { return frames_; }
    double simulationSeconds() const { return static_cast<double>(frames_) * config_.normalDt(); }
    double lastPhysicsMs() const { return lastPhysicsMs_; }
    const StepStats& stats() const { return stats_; }
    const std::string& error() const { return error_; }
private:
    void step();
    Config config_;
    SceneId scene_ = SceneId::Wall;
    Mode mode_ = Mode::Discrete;
    World world_;
    bool paused_ = true, slow_ = false;
    double accumulator_ = 0.0, lastPhysicsMs_ = 0.0;
    std::size_t frames_ = 0;
    StepStats stats_;
    std::string error_;
};
}
