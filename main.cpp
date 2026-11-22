#include <cstddef>
#include "demo_state.h"
#include <SFML/Graphics.hpp>
#include <deque>
#include <string>
#include <vector>
#include "config.h"


namespace pz3::render {
struct ViewConfig {
    unsigned width = 1280, height = 800, fps = 60;
    float canvasWidth = 1280.f, canvasHeight = 800.f;
    float worldLeft = 0.f, worldTop = 128.f, worldWidth = 1280.f, worldHeight = 672.f;
    float wallViewHeight = 240.f, combinedViewHeight = 460.f;
    std::size_t trailSamples = 48;
};

class Display {
public:
    Display(const ViewConfig& config, const std::string& fontPath);
    void draw(sf::RenderWindow& window, const DemoState& state, const std::string& benchmarkStatus);
private:
    void text(sf::RenderTarget& target, const std::string& value, float x, float y,
              unsigned size = 16, sf::Color color = sf::Color(220, 229, 241)) const;
    void updateTrails(const DemoState& state);
    struct Trail {
        std::size_t bodyIndex;
        std::deque<sf::Vector2f> points;
    };
    ViewConfig config_;
    sf::Font font_;
    std::vector<Trail> trails_;
    std::size_t lastFrame_ = 0;
    Mode lastMode_ = Mode::Discrete;
    SceneId lastScene_ = SceneId::Wall;
    bool trailsInitialized_ = false;
};


int runApplication(const Config& config, const std::string& fontPath);
}



#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace pz3::render {
namespace {
const sf::Color projectileColor{255, 175, 65};
std::string number(double value, int precision = 2) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}
sf::Color bodyColor(BodyType type) {
    if (type == BodyType::Static) {
        return sf::Color(170, 180, 195);
    }
    if (type == BodyType::Normal) {
        return sf::Color(65, 165, 255);
    }
    return projectileColor;
}
}
Display::Display(const ViewConfig& config, const std::string& fontPath) : config_(config) {
    std::vector<std::string> candidates;
    if (!fontPath.empty()) candidates.push_back(fontPath);
    else {
#ifdef _MSC_VER
        char* directory = nullptr;
        std::size_t length = 0;
        const auto result = _dupenv_s(&directory, &length, "WINDIR");
        const std::unique_ptr<char, decltype(&std::free)> windir(directory, &std::free);
        if (result == 0 && windir) candidates.push_back(std::string(windir.get()) + "/Fonts/segoeui.ttf");
#else
        if (const char* windir = std::getenv("WINDIR")) candidates.push_back(std::string(windir) + "/Fonts/segoeui.ttf");
#endif
        candidates.push_back("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");
        candidates.push_back("/System/Library/Fonts/Supplemental/Arial.ttf");
    }
    for (const auto& candidate : candidates)
        if (std::filesystem::exists(candidate) && font_.loadFromFile(candidate)) return;
    throw std::runtime_error("No readable UI font. Start with --font path/to/font.ttf");
}


void Display::text(sf::RenderTarget& target, const std::string& value, float x, float y,
                   unsigned size, sf::Color color) const {
    sf::Text label(value, font_, size);
    label.setPosition(x, y);
    label.setFillColor(color);
    target.draw(label);
}


// Запоминаем показанные позиции быстрых тел для отрисовки следа.
void Display::updateTrails(const DemoState& state) {
    const auto& bodies = state.world().bodies();
    const bool reset = !trailsInitialized_ || state.frames() < lastFrame_ ||
                       state.world().mode() != lastMode_ || state.scene() != lastScene_;
    if (reset) {
        trails_.clear();
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (bodies[i].type == BodyType::Fast) trails_.push_back({i, {}});
    }
    if (reset || state.frames() != lastFrame_) {
        for (auto& trail : trails_) {
            const auto center = bodies[trail.bodyIndex].bounds.center;
            trail.points.push_back({static_cast<float>(center.x), static_cast<float>(center.y)});
            while (trail.points.size() > config_.trailSamples) trail.points.pop_front();
        }
    }
    lastFrame_ = state.frames();
    lastMode_ = state.world().mode();
    lastScene_ = state.scene();
    trailsInitialized_ = true;
}
void Display::draw(sf::RenderWindow& window, const DemoState& state, const std::string& benchmarkStatus) {
    const auto& c = state.config();
    const auto& v = config_;
    const bool wall = state.scene() == SceneId::Wall;
    window.clear(sf::Color(16, 19, 25));
    updateTrails(state);

    const float height = wall ? v.wallViewHeight : v.combinedViewHeight;
    const sf::Vector2f center = wall ?
        sf::Vector2f(static_cast<float>(c.wallLeft), static_cast<float>(c.wallCenterY)) :
        sf::Vector2f(static_cast<float>(c.stackBase.x + 140.0), static_cast<float>(c.stackBase.y - 50.0));
    sf::View worldView(center, {height * v.worldWidth / v.worldHeight, height});
    worldView.setViewport({v.worldLeft / v.canvasWidth, v.worldTop / v.canvasHeight,
                           v.worldWidth / v.canvasWidth, v.worldHeight / v.canvasHeight});
    window.setView(worldView);
    for (const auto& trail : trails_) {
        sf::VertexArray line(sf::LineStrip);
        for (std::size_t i = 0; i < trail.points.size(); ++i) {
            auto color = projectileColor;
            color.a = static_cast<sf::Uint8>(25 + 145 * (i + 1) / trail.points.size());
            line.append({trail.points[i], color});
        }
        window.draw(line);
        const auto& bounds = state.world().bodies()[trail.bodyIndex].bounds;
        sf::RectangleShape mark({static_cast<float>(bounds.half.x * 2), static_cast<float>(bounds.half.y * 2)});
        mark.setOrigin(static_cast<float>(bounds.half.x), static_cast<float>(bounds.half.y));
        for (std::size_t i = 0; i + 1 < trail.points.size(); ++i) {
            auto color = projectileColor;
            color.a = static_cast<sf::Uint8>(15 + 85 * (i + 1) / trail.points.size());
            mark.setPosition(trail.points[i]);
            mark.setFillColor(color);
            window.draw(mark);
        }
    }
    for (const auto& body : state.world().bodies()) {
        const auto min = body.bounds.min();
        sf::RectangleShape box({static_cast<float>(body.bounds.half.x * 2),
                                static_cast<float>(body.bounds.half.y * 2)});
        box.setPosition(static_cast<float>(min.x), static_cast<float>(min.y));
        box.setFillColor(bodyColor(body.type));
        box.setOutlineThickness(-0.4f);
        box.setOutlineColor(sf::Color(16, 19, 25));
        window.draw(box);
    }

    window.setView(sf::View(sf::FloatRect(0, 0, v.canvasWidth, v.canvasHeight)));
    const auto mode = state.world().mode();
    const auto highSteps = mode == Mode::Pure240 ? state.stats().normalSubsteps : state.stats().fastSubsteps;
    const double highHz = mode == Mode::Pure240 ? c.purePhysicsHz : c.fastPhysicsHz;
    text(window, std::string("Mode: ") + modeName(mode) + " | Scene: " +
         (wall ? "Wall" : "Combined / stack"), 16, 10, 18);
    text(window, "Physics ms/frame (last): " + number(state.lastPhysicsMs(), 4) +
         " | Steps at " + number(highHz, 0) + " Hz: " + std::to_string(highSteps), 16, 36, 18);
    text(window, std::string(state.paused() ? "PAUSED" : "RUNNING") +
         " | Time: " + number(state.simulationSeconds(), 3) + " s | Frames: " +
         std::to_string(state.frames()) + " | Playback: " + number(state.playbackSpeed(), 1) + "x", 16, 62, 18);
    const auto status = state.error().empty() ?
        "Keys: 1-4  Tab  Space  N  R  B | " + benchmarkStatus :
        "Physics error: " + state.error() + " | R to reset";
    text(window, status, 16, 88, 16);
    window.display();
}
}


#include "benchmark.h"
#include <chrono>
#include <future>
#include <iostream>
#include <exception>

// создаём окно SFML отрисовщик и состояние демонстрации
namespace pz3::render {
int runApplication(const Config& config, const std::string& fontPath) {
    const ViewConfig viewConfig;
    Display display(viewConfig, fontPath);
    DemoState state(config);
    sf::RenderWindow window(sf::VideoMode(viewConfig.width, viewConfig.height), "PZ3 | AABB Physics Lab");
    window.setFramerateLimit(viewConfig.fps);
    window.setKeyRepeatEnabled(false);
    sf::Clock clock;
    std::future<std::string> benchmark;
    std::string status = "B: run headless comparison";
    std::cout << "AABB Physics Lab: 1-4 mode, TAB scene, R reset, SPACE pause, N step, T slow, B benchmark, ESC exit.\n"
              << "Starts paused so the wall experiment can be inspected one step at a time.\n" << std::flush;
    while (window.isOpen()) {
        bool controlChanged = false;
        sf::Event event{};
        // обрабатываем события окна и передаём команды с клавиатуры в DemoState
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
            if (event.type == sf::Event::LostFocus) {
                state.setPaused(true);
                controlChanged = true;
            }
            if (event.type != sf::Event::KeyPressed) continue;
            controlChanged = true;
            switch (event.key.code) {
            case sf::Keyboard::Num1: case sf::Keyboard::Numpad1: state.selectMode(Mode::Discrete); break;
            case sf::Keyboard::Num2: case sf::Keyboard::Numpad2: state.selectMode(Mode::Ccd); break;
            case sf::Keyboard::Num3: case sf::Keyboard::Numpad3: state.selectMode(Mode::MultiRate); break;
            case sf::Keyboard::Num4: case sf::Keyboard::Numpad4: state.selectMode(Mode::Pure240); break;
            case sf::Keyboard::Tab: state.toggleScene(); break;
            case sf::Keyboard::R: state.reset(); break;
            case sf::Keyboard::Space: state.togglePause(); break;
            case sf::Keyboard::N: state.singleStep(); break;
            case sf::Keyboard::T: state.toggleSlowMotion(); break;
            case sf::Keyboard::Escape: window.close(); break;
            case sf::Keyboard::B:
                if (!benchmark.valid()) {
                    state.setPaused(true);
                    status = "Benchmark running...";
                    std::cout << "\nStarting headless benchmark; simulation paused.\n" << std::flush;
                    benchmark = std::async(std::launch::async, [config] {
                        std::ostringstream out;
                        runBenchmark(config, out);
                        return out.str();
                    });
                }
                break;
            default: break;
            }
        }
        if (!window.isOpen()) break;
        if (benchmark.valid() && benchmark.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            try {
                std::cout << benchmark.get() << std::flush;
                status = "Done. Table in console. B: rerun";
            } catch (const std::exception& e) {
                std::cerr << "Benchmark failed: " << e.what() << '\n';
                status = "Benchmark failed. See console.";
            }
        }
        // измеряем время между кадрами и передаём его в DemoState
        // после команды управления пропускаем автоматическое обновление на этом кадре
        const double elapsed = clock.restart().asSeconds();
        if (!controlChanged) state.update(elapsed);
        display.draw(window, state, status);
    }
    if (benchmark.valid()) {
        try { std::cout << benchmark.get() << std::flush; }
        catch (const std::exception& e) { std::cerr << "Benchmark failed: " << e.what() << '\n'; }
    }
    return 0;
}
}



// читаем параметры запуска и выбираем открыть окно или запустить бенчмарк
int main(int argc, char** argv) {
    try {
        bool benchmark = false;
        std::string font;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--benchmark") benchmark = true;
            else if (arg == "--font" && i + 1 < argc) font = argv[++i];
            else if (arg == "--help" || arg == "-h") {
                std::cout << "Usage: aabb_demo [--benchmark] [--font font.ttf]\n"
                          << "--benchmark: print headless comparison without creating a window or loading a font.\n"
                          << "Controls: 1-4 modes, TAB scene, R reset, SPACE pause, N step, T slow, B benchmark, ESC exit.\n";
                return 0;
            } else { std::cerr << "Unknown or incomplete option: " << arg << '\n'; return 2; }
        }
        const pz3::Config config;

        if (benchmark) {
            pz3::runBenchmark(config, std::cout);
            return 0;
        }
        return pz3::render::runApplication(config, font);
    } catch (const std::exception& e) {
        std::cerr << "Application failed: " << e.what() << '\n';
        return 1;
    }
}
