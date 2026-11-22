#include "benchmark.h"
#include "scenes.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <ostream>
#include <stdexcept>

namespace pz3 {
namespace {
// переводим заданное врем€ симул€ции в число базовых шагов
std::size_t frameCount(const Config& c) {
    return static_cast<std::size_t>(std::ceil(c.benchmarkSeconds * c.normalPhysicsHz));
}

// провер€ем точное совпадение координат и скоростей всех тел
bool sameState(const std::vector<Body>& a, const std::vector<Body>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i].bounds.center.x != b[i].bounds.center.x || a[i].bounds.center.y != b[i].bounds.center.y ||
            a[i].velocity.x != b[i].velocity.x || a[i].velocity.y != b[i].velocity.y) return false;
    return true;
}

void diagnose(const Config& c, BenchmarkRow& r, const std::vector<Body>& reference) {
    const auto initial = makeCombinedScene(c);
    World world(c, initial, r.mode);
    const auto frames = frameCount(c);
    const auto settle = static_cast<std::size_t>(static_cast<double>(frames) * c.benchmarkSettleFraction);
    std::vector<double> minY(c.stackCount, std::numeric_limits<double>::infinity());
    std::vector<double> maxY(c.stackCount, -std::numeric_limits<double>::infinity());
    // отдельно прогон€ем сцену B и провер€ем устойчивость блоков в стеке
    for (std::size_t frame = 0; frame < frames; ++frame) {
        world.step();
        for (std::size_t i = 0; i < c.stackCount; ++i) {
            const auto index = c.staticCount + i;
            const auto& b = world.bodies()[index];
            const auto& base = initial.bodies[index];
            const auto& lower = world.bodies()[i == 0 ? 0 : index - 1];
            const double speed = std::hypot(b.velocity.x, b.velocity.y);
            const double drift = std::hypot(b.bounds.center.x - base.bounds.center.x,
                                            b.bounds.center.y - base.bounds.center.y);
            const double penetration = std::max(0.0, b.bounds.max().y - lower.bounds.min().y);
            const bool finite = std::isfinite(speed) && std::isfinite(drift) && std::isfinite(penetration);
            r.stable = r.stable && finite;
            if (!finite) continue;
            r.stackPeakSpeed = std::max(r.stackPeakSpeed, speed);
            r.stackMaxDrift = std::max(r.stackMaxDrift, drift);
            r.stackMaxPenetration = std::max(r.stackMaxPenetration, penetration);
            if (frame >= settle) {
                minY[i] = std::min(minY[i], b.bounds.center.y);
                maxY[i] = std::max(maxY[i], b.bounds.center.y);
            }
        }
    }
    for (std::size_t i = 0; i < c.stackCount; ++i)
        if (std::isfinite(minY[i]) && std::isfinite(maxY[i]))
            r.stackJitter = std::max(r.stackJitter, maxY[i] - minY[i]);
    r.stable = r.stable && r.stackPeakSpeed <= c.benchmarkMaxStackSpeed &&
        r.stackMaxDrift <= c.benchmarkMaxStackDrift && r.stackMaxPenetration <= c.benchmarkMaxPenetration &&
        r.stackJitter <= c.benchmarkMaxJitter;
    r.deterministic = r.deterministic && sameState(reference, world.bodies());
    // дважды запускаем сцену A на один базовый шаг сравниваем результаты и провер€ем пролЄт сквозь стену и отскок
    World wall(c, makeWallScene(c), r.mode), repeat(c, makeWallScene(c), r.mode);
    const auto stats = wall.step();
    repeat.step();
    r.deterministic = r.deterministic && sameState(wall.bodies(), repeat.bodies());
    r.tunneled = wall.bodies()[0].bounds.min().x > c.wallLeft + c.wallThickness && stats.contacts == 0;
    r.bounced = stats.contacts > 0 && wall.bodies()[0].velocity.x < 0.0;
}
}
std::vector<BenchmarkRow> measureBenchmark(const Config& c) {
    c.validate();
    if (!std::isfinite(c.benchmarkSettleFraction) || c.benchmarkSettleFraction < 0.0 ||
        c.benchmarkSettleFraction >= 1.0 || !std::isfinite(c.benchmarkMaxStackSpeed) ||
        c.benchmarkMaxStackSpeed < 0.0 || !std::isfinite(c.benchmarkMaxStackDrift) ||
        c.benchmarkMaxStackDrift < 0.0 || !std::isfinite(c.benchmarkMaxPenetration) ||
        c.benchmarkMaxPenetration < 0.0 || !std::isfinite(c.benchmarkMaxJitter) || c.benchmarkMaxJitter < 0.0)
        throw std::invalid_argument("Invalid benchmark diagnostic limits");
    std::vector<BenchmarkRow> rows;
    const auto frames = frameCount(c);
    // дл€ каждого повтора создаЄм сцену B заново с теми же начальными услови€ми
    for (auto mode : {Mode::Discrete, Mode::Ccd, Mode::MultiRate, Mode::Pure240}) {
        BenchmarkRow r;
        r.mode = mode;
        double milliseconds = 0.0;
        std::size_t normalSteps = 0, fastSteps = 0;
        std::vector<Body> reference;
        for (std::size_t repeat = 0; repeat < c.benchmarkRepeats; ++repeat) {
            World world(c, makeCombinedScene(c), mode);
            // замер€ем цикл вызовов world.step() вместе с подсчЄтом подшагов
            // мир уже создан, диагностика выполн€етс€ после остановки таймера
            const auto start = std::chrono::steady_clock::now();
            for (std::size_t frame = 0; frame < frames; ++frame) {
                const auto stats = world.step();
                normalSteps += stats.normalSubsteps;
                fastSteps += stats.fastSubsteps;
            }
            milliseconds += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            if (repeat == 0) reference = world.bodies();
            else r.deterministic = r.deterministic && sameState(reference, world.bodies());
        }
        // усредн€ем результаты по всем базовым шагам всех повторов
        const double samples = static_cast<double>(frames) * static_cast<double>(c.benchmarkRepeats);
        r.millisecondsPerFrame = milliseconds / samples;
        r.normalStepsPerFrame = static_cast<double>(normalSteps) / samples;
        r.fastStepsPerFrame = static_cast<double>(fastSteps) / samples;
        r.highStepsPerFrame = mode == Mode::Pure240 ? r.normalStepsPerFrame : r.fastStepsPerFrame;
        const auto scheduled = mode == Mode::Pure240 ? normalSteps : normalSteps + fastSteps;
        r.millisecondsPerScheduledStep = milliseconds / static_cast<double>(scheduled);
        diagnose(c, r, reference);
        rows.push_back(r);
    }
    return rows;
}
// выводит таблицу замеров и диагностику дл€ каждого режима
void printBenchmark(const Config& c, const std::vector<BenchmarkRow>& rows, std::ostream& out) {
    const auto oldFlags = out.flags();
    const auto oldPrecision = out.precision();
    out << "HEADLESS AABB BENCHMARK (physics only; no window)\nCombined scene: "
        << c.staticCount << " static / " << c.normalCount << " normal / " << c.fastCount
        << " fast; stack=" << c.stackCount << "; Hz=" << c.normalPhysicsHz << '/'
        << c.fastPhysicsHz << '/' << c.purePhysicsHz << "; seconds="
        << static_cast<double>(frameCount(c)) * c.normalDt() << "; repeats=" << c.benchmarkRepeats
        << "; projectile=" << c.projectileSpeed << " px/s; wall=" << c.wallThickness
        << " px; restitution=" << c.restitution << '\n'
        << "Identical initial states and simulated duration. Timing excludes construction, diagnostics and rendering.\n"
        << "Stability: speed<=" << c.benchmarkMaxStackSpeed << " px/s, drift<=" << c.benchmarkMaxStackDrift
        << " px, overlap<=" << c.benchmarkMaxPenetration << " px, settled jitter<=" << c.benchmarkMaxJitter
        << " px; settle fraction=" << c.benchmarkSettleFraction << ".\n"
        << "Determinism: exact positions/velocities across repeats + independent diagnostic run; wall checked twice.\n"
        << "Tunnel/bounce: wall scene after one base step.\n\n"
        << std::left << std::setw(17) << "Mode" << std::setw(9) << "Stable" << std::setw(10) << "Determ."
        << std::setw(9) << "Tunnel" << std::setw(9) << "Bounce" << std::right << std::setw(12) << "ms/frame"
        << std::setw(13) << "ms/step(avg)" << std::setw(12) << "high/frame"
        << std::setw(12) << "norm/frame" << std::setw(12) << "fast/frame" << '\n';
    for (const auto& r : rows)
        out << std::left << std::setw(17) << modeName(r.mode) << std::setw(9) << (r.stable ? "yes" : "NO")
            << std::setw(10) << (r.deterministic ? "yes" : "NO") << std::setw(9) << (r.tunneled ? "yes" : "no")
            << std::setw(9) << (r.bounced ? "yes" : "no") << std::right << std::fixed << std::setprecision(5)
            << std::setw(12) << r.millisecondsPerFrame << std::setw(13) << r.millisecondsPerScheduledStep
            << std::setw(12) << r.highStepsPerFrame << std::setw(12) << r.normalStepsPerFrame
            << std::setw(12) << r.fastStepsPerFrame << '\n';
    out << "\nStep(avg) = total physics wall time / scheduled passes (amortized, not separate normal/fast costs).\n"
        << "Pure counts its shared pass once. High/frame uses fast Hz in Multi-rate, pure Hz in Pure.\n"
        << "At defaults high/frame counts 240 Hz steps per base frame.\nSimulated dt: base="
        << c.normalDt() * 1000.0 << " ms, fast=" << c.fastDt() * 1000.0
        << " ms, pure=" << c.pureDt() * 1000.0 << " ms.\n\nMeasured stack diagnostics (px, px/s):\n";
    for (const auto& r : rows)
        out << "  " << modeName(r.mode) << ": peak-speed=" << r.stackPeakSpeed << ", drift="
            << r.stackMaxDrift << ", overlap=" << r.stackMaxPenetration << ", settled-jitter=" << r.stackJitter << '\n';
    out << "Timing varies with CPU/load/build; deterministic results exclude timings.\n";
    out.flags(oldFlags);
    out.precision(oldPrecision);
}
// запускает замер и передаЄт результаты на вывод
void runBenchmark(const Config& config, std::ostream& output) {
    printBenchmark(config, measureBenchmark(config), output);
}
}
