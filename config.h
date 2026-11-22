#pragma once

#include <cstddef>
#include <vector>

namespace pz3 {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
    Vec2& operator+=(Vec2 other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    Vec2& operator-=(Vec2 other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }
};
inline Vec2 operator+(Vec2 a, Vec2 b) { return a += b; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return a -= b; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

struct Aabb {
    Vec2 center;
    Vec2 half;
    Vec2 min() const { return center - half; }
    Vec2 max() const { return center + half; }
};

enum class BodyType { Static, Normal, Fast };
enum class Mode { Discrete, Ccd, MultiRate, Pure240 };

struct Body {
    Aabb bounds;
    Vec2 velocity;
    double inverseMass = 1.0;
    double restitution = 0.0;
    BodyType type = BodyType::Normal;
    double mobility() const {
        // статические тела не получают импульсов/изменения положения
        if (type == BodyType::Static) {
            return 0.0;
        }
        return inverseMass;
    }
};

struct Scene {
    std::vector<Body> bodies;
    Vec2 gravity;
};

struct StepStats {
    std::size_t normalSubsteps = 0;
    std::size_t fastSubsteps = 0;
    std::size_t contacts = 0;
    std::size_t ccdEvents = 0;
    std::size_t ccdLimitHits = 0;
};

const char* modeName(Mode mode);



struct Config {
    double restitution = 1.0;           // отскок
    double blockRestitution = 0.0;      // отскок блоков
    double projectileSpeed = 2000.0;    // скорость снаряда
    double wallThickness = 10.0;        // толщина стены
    double normalPhysicsHz = 60.0;      // частота обновления физики для нормальных тел
    double fastPhysicsHz = 240.0;       // частота обновления физики для быстрых тел
    double purePhysicsHz = 240.0;       // частота обновления физики для чистой симуляции, высокая для всех
    Vec2 gravity{0.0, 600.0};           // гравитация для нормальных тел
    Vec2 wallGravity{0.0, 0.0};         // гравитация для стены

    Vec2 projectileSize{4.0, 4.0};
    Vec2 wallProjectilePosition{80.0, 100.0};
    double wallLeft = 100.0;
    double wallCenterY = 100.0;
    double wallHeight = 200.0;
    double projectileMass = 1.0;
    double blockMass = 1.0;
    Vec2 blockSize{20.0, 20.0};
    double staticSizeMultiplier = 2.0;
    std::size_t staticCount = 5;
    std::size_t normalCount = 12;
    std::size_t fastCount = 2;
    std::size_t stackCount = 6;
    Vec2 stackBase{200.0, 480.0};
    double supportSpacing = 100.0;
    Vec2 looseBlocksOrigin{400.0, 400.0};
    double looseBlockSpacing = 45.0;
    Vec2 loosePairOffset{6.0, -26.0};
    Vec2 fastOrigin{50.0, 490.0};
    double fastSpacing = 10.0;
    Vec2 dynamicFastOrigin{360.0, 405.0};

    int velocityIterations = 24;
    int positionIterations = 24;
    double penetrationSlop = 0.001;
    double positionCorrection = 0.8;
    double restitutionThreshold = 20.0;
    double contactTolerance = 1e-8;
    double velocityEpsilon = 1e-10;
    double timeEpsilon = 1e-12;
    std::size_t maxCcdEvents = 128;
    double benchmarkSeconds = 5.0;
    std::size_t benchmarkRepeats = 3;
    double benchmarkMaxStackSpeed = 30.0;
    double benchmarkMaxStackDrift = 1.0;
    double benchmarkMaxPenetration = 0.2;
    double benchmarkMaxJitter = 0.1;
    double benchmarkSettleFraction = 0.5;

    double normalDt() const { return 1.0 / normalPhysicsHz; }
    double fastDt() const { return 1.0 / fastPhysicsHz; }
    double pureDt() const { return 1.0 / purePhysicsHz; }
    void validate() const;
};
}
