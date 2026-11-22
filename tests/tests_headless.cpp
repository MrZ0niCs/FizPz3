#include "physics.h"
#include "scenes.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace pz3;
constexpr Mode modes[] = {Mode::Discrete, Mode::Ccd, Mode::MultiRate, Mode::Pure240};
void require(bool result, const std::string& message) {
    if (!result) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
            message + ": actual=" + std::to_string(actual) + " expected=" + std::to_string(expected));
}
Body body(Vec2 center, Vec2 half, Vec2 velocity, BodyType type, double inverseMass = 1.0) {
    return {{center, half}, velocity, type == BodyType::Static ? 0.0 : inverseMass, 1.0, type};
}
void equalState(const World& a, const World& b) {
    require(a.bodies().size() == b.bodies().size(), "State size");
    for (std::size_t i = 0; i < a.bodies().size(); ++i) {
        const auto& x = a.bodies()[i];
        const auto& y = b.bodies()[i];
        require(x.bounds.center.x == y.bounds.center.x && x.bounds.center.y == y.bounds.center.y &&
                x.velocity.x == y.velocity.x && x.velocity.y == y.velocity.y, "Exact deterministic state");
    }
}

void wall() {
    const Config c;
    for (auto mode : modes) {
        World world(c, makeWallScene(c), mode);
        const auto stats = world.step();
        const Body& p = world.bodies()[0];
        if (mode == Mode::Discrete) {
            require(p.bounds.min().x > c.wallLeft + c.wallThickness, "60 Hz must tunnel through wall");
            near(p.velocity.x, c.projectileSpeed, 1e-9, "Tunnelling speed");
            require(stats.contacts == 0, "No discrete collision");
        } else {
            require(stats.contacts > 0 && p.velocity.x < 0.0, "Contact and bounce");
            require(p.bounds.max().x <= c.wallLeft + c.penetrationSlop * 2.0, "Projectile remains before wall");
            near(p.velocity.x, -c.projectileSpeed, 1e-9, "Elastic wall reflection");
        }
        require(stats.ccdLimitHits == 0, "No CCD limit");
    }
    Config inelastic = c;
    inelastic.restitution = 0.5;
    World world(inelastic, makeWallScene(inelastic), Mode::Ccd);
    world.step();
    near(world.bodies()[0].velocity.x, -1000.0, 1e-9, "Config restitution");
}

void ccd() {
    const Config c;
    World wallWorld(c, makeWallScene(c), Mode::Ccd);
    const auto stats = wallWorld.step();
    const double hitX = c.wallLeft - c.projectileSize.x * 0.5;
    const double hitTime = (hitX - c.wallProjectilePosition.x) / c.projectileSpeed;
    near(wallWorld.bodies()[0].bounds.center.x,
         hitX - c.projectileSpeed * (c.normalDt() - hitTime), 1e-8, "CCD residual dt");
    require(stats.ccdEvents == 1, "Exactly one wall event");


    Scene corridor{{body({5, 0}, {1, 1}, {2000, 0}, BodyType::Fast),
                    body({-1, 0}, {1, 10}, {}, BodyType::Static),
                    body({11, 0}, {1, 10}, {}, BodyType::Static)}, {}};
    World repeated(c, corridor, Mode::Ccd);
    const auto repeatedStats = repeated.step();
    require(repeatedStats.ccdEvents == 4, "Multiple bounces in residual time");
    near(repeated.bodies()[0].bounds.center.x, 6.333333333333333, 1e-8, "Repeated bounce final position");
    near(repeated.bodies()[0].velocity.x, 2000, 1e-9, "Repeated bounce final velocity");


    Scene pair{{body({0, 0}, {1, 1}, {1000, 0}, BodyType::Fast),
                body({10, 0}, {1, 1}, {-1000, 0}, BodyType::Normal)}, {}};
    World moving(c, pair, Mode::Ccd);
    moving.step();
    near(moving.bodies()[0].velocity.x, -1000, 1e-8, "Relative velocity A");
    near(moving.bodies()[1].velocity.x, 1000, 1e-8, "Relative velocity B");
    near(moving.bodies()[0].bounds.center.x, 8.0 - 1000 * c.normalDt(), 1e-8, "Moving A residual");
    near(moving.bodies()[1].bounds.center.x, 2.0 + 1000 * c.normalDt(), 1e-8, "Moving B residual");

    Body a = body({0, 0}, {1, 1}, {1000, 0}, BodyType::Fast);
    Body b = body({10, 4}, {1, 1}, {}, BodyType::Static);
    require(!sweptAabb(a, b, c.normalDt(), c).hit, "Parallel separated axis");
    b.bounds.center.y = 0;
    const auto hit = sweptAabb(a, b, c.normalDt(), c);
    require(hit.hit, "Parallel overlapping axis");
    near(hit.time, 0.008, 1e-12, "Slab time");
    a.bounds.center.x = 8;
    a.velocity.x = -1000;
    require(!sweptAabb(a, b, c.normalDt(), c).hit, "Touching but separating");
    a.velocity.x = 1000;
    require(sweptAabb(a, b, c.normalDt(), c).hit, "Touching and closing");
    a.bounds.center = {10, -10};
    a.velocity = {0, 1000};
    const auto vertical = sweptAabb(a, b, c.normalDt(), c);
    require(vertical.hit && vertical.normal.y == 1, "Vertical normal");

    Scene boundary{{body({0, 0}, {1, 1}, {600, 0}, BodyType::Fast),
                    body({12, 0}, {1, 5}, {}, BodyType::Static)}, {}};
    World atEnd(c, boundary, Mode::Ccd);
    atEnd.step();
    near(atEnd.bodies()[0].bounds.center.x, 10, 1e-9, "Contact at exact dt");
    require(atEnd.bodies()[0].velocity.x < 0, "Boundary bounce");

    Config limit = c;
    limit.maxCcdEvents = 1;
    World limited(limit, corridor, Mode::Ccd);
    bool failed = false;
    try { limited.step(); } catch (const std::runtime_error&) { failed = true; }
    require(failed && limited.lastStats().ccdLimitHits == 1, "Explicit CCD event limit failure");
}

void multirate() {
    Config c;

    Scene free{{body({0, 0}, {1, 1}, {}, BodyType::Normal),
                body({100, 0}, {1, 1}, {}, BodyType::Fast)}, c.gravity};
    World w(c, free, Mode::MultiRate);
    const auto stats = w.step();
    require(stats.normalSubsteps == 1 && stats.fastSubsteps == 4, "Separate 60/240 rates");
    near(w.bodies()[0].bounds.center.y, c.gravity.y * c.normalDt() * c.normalDt(), 1e-10,
         "Normal integrated only once");
    near(w.bodies()[1].bounds.center.y, c.gravity.y * c.fastDt() * c.fastDt() * 10, 1e-10,
         "Fast integrated four times");


    Scene crossing{{body({0, 0}, {2, 2}, {2000, 0}, BodyType::Fast),
                    body({12, 0}, {2, 2}, {-100, 0}, BodyType::Normal)}, {}};
    World mixed(c, crossing, Mode::MultiRate);
    require(mixed.step().contacts > 0, "Fast-normal contact");
    require(mixed.bodies()[0].velocity.x < 0 && mixed.bodies()[1].velocity.x > 0,
            "Fast-normal impulse affects both bodies");
    near(mixed.bodies()[0].velocity.x + mixed.bodies()[1].velocity.x, 1900, 1e-8,
         "Fast-normal momentum");

    Scene normals{{body({0, 0}, {2, 2}, {300, 0}, BodyType::Normal),
                   body({8, 0}, {2, 2}, {}, BodyType::Normal)}, {}};
    World normalPair(c, normals, Mode::MultiRate);
    require(normalPair.step().contacts > 0, "Normal-normal boundary contact");
    near(normalPair.bodies()[1].velocity.x, 300, 1e-8, "Normal-normal impulse");

    Scene fastPair{{body({0, 0}, {2, 2}, {600, 0}, BodyType::Fast),
                    body({8, 0}, {2, 2}, {}, BodyType::Fast)}, {}};
    for (auto mode : modes) {
        World pair(c, fastPair, mode);
        pair.step();
        if (mode != Mode::Discrete) {
            near(pair.bodies()[0].velocity.x, 0, 1e-8, "Fast-fast A impulse");
            near(pair.bodies()[1].velocity.x, 600, 1e-8, "Fast-fast B impulse");
        }
    }
    c.projectileSpeed = 20000;
    World tooFast(c, makeWallScene(c), Mode::MultiRate);
    tooFast.step();
    require(tooFast.bodies()[0].velocity.x > 0, "Multi-rate can tunnel at excessive speed");
}

void combinedFastNormal() {
    const Config c;
    const Scene scene = makeCombinedScene(c);
    const std::size_t firstFast = c.staticCount + c.normalCount;
    const std::size_t secondFast = firstFast + 1;
    const std::size_t target = c.staticCount + c.stackCount;
    const Body& initialProjectile = scene.bodies[secondFast];
    const Body& initialBlock = scene.bodies[target];
    near(scene.bodies[firstFast].bounds.center.x, c.fastOrigin.x, 0.0, "First projectile x preserved");
    near(scene.bodies[firstFast].bounds.center.y, c.fastOrigin.y, 0.0, "First projectile y preserved");


    Scene pair{{initialProjectile, initialBlock}, scene.gravity};
    World collision(c, pair, Mode::MultiRate);
    const auto stats = collision.step();
    const auto& projectile = collision.bodies()[0];
    const auto& block = collision.bodies()[1];
    require(stats.contacts > 0, "Scene B fast-normal pair makes contact within one base step");
    require(stats.fastSubsteps == 4 && stats.normalSubsteps == 1, "Fast-normal uses real multi-rate schedule");
    require(projectile.velocity.x > 0.0 && projectile.velocity.x < initialProjectile.velocity.x,
            "Second projectile transfers horizontal momentum to the right");
    require(block.velocity.x > initialBlock.velocity.x, "Loose normal gains horizontal velocity");
    near(projectile.velocity.x / projectile.inverseMass + block.velocity.x / block.inverseMass,
         initialProjectile.velocity.x / initialProjectile.inverseMass + initialBlock.velocity.x / initialBlock.inverseMass,
         1e-8, "Fast-normal horizontal momentum conserved");

    World combined(c, scene, Mode::MultiRate);
    combined.step();
    near(combined.bodies()[secondFast].velocity.x, projectile.velocity.x, 1e-8,
         "Projectile also changes velocity in full Scene B");
    near(combined.bodies()[target].velocity.x, block.velocity.x, 1e-8,
         "Target also changes velocity in full Scene B");


    const double stackRight = scene.bodies[c.staticCount].bounds.max().x;
    for (auto mode : modes) {
        World world(c, scene, mode);
        bool staticBounce = false;
        for (int frame = 0; frame < 600; ++frame) {
            world.step();
            staticBounce = staticBounce || world.bodies()[firstFast].velocity.x < 0.0;
            for (std::size_t i = target; i < firstFast; ++i)
                require(world.bodies()[i].bounds.min().x > stackRight, "Loose normals stay right of left stack");
            require(world.bodies()[secondFast].bounds.min().x > stackRight,
                    "Second projectile stays right of left stack");
        }
        require(staticBounce, "First projectile still demonstrates fast-static bounce");
    }
}

void scheduler() {
    Config c;
    for (double normalHz : {60.0, 120.0, 100.0}) {
        c.normalPhysicsHz = normalHz;
        for (auto mode : {Mode::MultiRate, Mode::Pure240}) {
            World world(c, makeWallScene(c), mode);
            std::size_t normals = 0, fast = 0;
            for (int frame = 0; frame < static_cast<int>(normalHz); ++frame) {
                const auto stats = world.step();
                normals += stats.normalSubsteps;
                fast += stats.fastSubsteps;
            }
            require(fast == 240, "Accumulator gives 240 fixed fast ticks per second");
            require(normals == (mode == Mode::MultiRate ? static_cast<std::size_t>(normalHz) : 240),
                    "Normal steps track selected rate");
        }
    }
    c = Config{};
    for (auto mode : modes) {
        World framed(c, makeCombinedScene(c), mode), direct(c, makeCombinedScene(c), mode);
        require(framed.advance(c.normalDt() * 0.25) == 0, "Outer accumulator holds partial step");
        require(framed.advance(c.normalDt() * 0.75) == 1, "Outer accumulator completes step");
        require(framed.advance(c.normalDt() * 2) == 2, "Outer accumulator performs catch-up");
        for (int i = 0; i < 3; ++i) direct.step();
        equalState(framed, direct);
    }
}

void determinism() {
    Config c;
    for (auto mode : modes) {
        for (bool combined : {false, true}) {
            const auto scene = combined ? makeCombinedScene(c) : makeWallScene(c);
            World first(c, scene, mode), second(c, scene, mode);
            for (int frame = 0; frame < 180; ++frame) {
                const auto a = first.step();
                const auto b = second.step();
                require(a.contacts == b.contacts && a.ccdEvents == b.ccdEvents &&
                        a.fastSubsteps == b.fastSubsteps, "Deterministic contact order and counters");
            }
            equalState(first, second);
        }
    }
}

void stack() {
    Config c;
    for (auto mode : modes) {
        World world(c, makeCombinedScene(c), mode);
        std::vector<double> minY(c.stackCount, std::numeric_limits<double>::infinity());
        std::vector<double> maxY(c.stackCount, -std::numeric_limits<double>::infinity());
        for (int frame = 0; frame < 600; ++frame) {
            const auto stats = world.step();
            require(stats.ccdLimitHits == 0, "Stack CCD event limit");
            for (std::size_t i = 0; i < c.stackCount; ++i) {
                const Body& b = world.bodies()[c.staticCount + i];
                require(std::isfinite(b.bounds.center.y) && std::isfinite(b.velocity.y), "Finite stack");
                require(std::abs(b.velocity.y) < 30, "No explosive stack speeds");
                near(b.bounds.center.x, c.stackBase.x, 1e-8, "Stack lateral stability");
                const double expected = c.stackBase.y - (static_cast<double>(i) + 0.5) * c.blockSize.y;
                near(b.bounds.center.y, expected, 1.0, "Stack position bounded");
                if (frame >= 300) {
                    minY[i] = std::min(minY[i], b.bounds.center.y);
                    maxY[i] = std::max(maxY[i], b.bounds.center.y);
                }
            }
        }
        for (std::size_t i = 0; i < c.stackCount; ++i) {
            require(maxY[i] - minY[i] < 0.1, "Settled stack jitter below 0.1 px");
            const auto& upper = world.bodies()[c.staticCount + i];
            const auto& lower = world.bodies()[i == 0 ? 0 : c.staticCount + i - 1];
            const double penetration = upper.bounds.max().y - lower.bounds.min().y;
            require(penetration < 0.2, "Stack penetration below 0.2 px per contact");
        }
        std::cout << "  stack stable: " << modeName(mode) << " (10 simulated seconds)\n";
    }
}

void config() {
    const Config c;
    const auto combined = makeCombinedScene(c);
    std::size_t statics = 0, normals = 0, fast = 0;
    for (const auto& b : combined.bodies) {
        if (b.type == BodyType::Static) {
            ++statics;
            near(b.bounds.half.x * 2, c.blockSize.x * 2, 1e-9, "Static size x2");
        } else if (b.type == BodyType::Normal) ++normals;
        else ++fast;
    }
    require(statics == c.staticCount && normals == c.normalCount && fast == c.fastCount,
            "Configured scene counts");
    for (auto mode : modes) {
        Scene scene{{body({0, 0}, {2, 2}, {100, 100}, BodyType::Static)}, c.gravity};
        World w(c, scene, mode);
        w.step();
        near(w.bodies()[0].bounds.center.x, 0, 0, "Statics do not move");
        near(w.bodies()[0].velocity.x, 0, 0, "Static velocity sanitized");
    }
    Config invalid = c; invalid.normalPhysicsHz = 0;
    bool rejected = false;
    try { invalid.validate(); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Reject invalid timestep");
    World w(c, makeWallScene(c), Mode::Discrete);
    rejected = false;
    try { w.advance(-1); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Reject negative elapsed time");
}
}

int main(int argc, char** argv) {
    const std::vector<std::pair<std::string, std::function<void()>>> cases = {
        {"wall", wall}, {"ccd", ccd}, {"multirate", multirate}, {"scheduler", scheduler},
        {"combined_fast_normal", combinedFastNormal},
        {"determinism", determinism}, {"stack", stack}, {"config", config}};
    int failed = 0, executed = 0;
    for (const auto& test : cases) {
        if (argc > 1 && test.first != argv[1]) continue;
        ++executed;
        try { test.second(); std::cout << "PASS " << test.first << '\n'; }
        catch (const std::exception& error) {
            ++failed; std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
        }
    }
    if (executed == 0) { std::cerr << "Unknown test name\n"; return 1; }
    return failed == 0 ? 0 : 1;
}
