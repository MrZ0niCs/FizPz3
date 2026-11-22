#include "config.h"
#include <cmath>
#include <stdexcept>

namespace pz3 {
namespace {
bool finite(Vec2 v) { return std::isfinite(v.x) && std::isfinite(v.y); }
bool positive(double v) { return std::isfinite(v) && v > 0.0; }
bool size(Vec2 v) { return positive(v.x) && positive(v.y); }
bool unit(double v) { return std::isfinite(v) && v >= 0.0 && v <= 1.0; }
}  

// проверяем настройки перед использованием
void Config::validate() const {
    // все частоты должны быть положительными, а fast и pure не ниже базовой
    if (!positive(normalPhysicsHz) || !positive(fastPhysicsHz) ||
        !positive(purePhysicsHz) || fastPhysicsHz < normalPhysicsHz ||
        purePhysicsHz < normalPhysicsHz) {
        throw std::invalid_argument("Invalid physics/scene configuration");
    }

    // проверяем компоненты гравитации обеих сцен
    if (!finite(gravity) || !finite(wallGravity)) {
        throw std::invalid_argument("Invalid physics/scene configuration");
    }

    // оба коэффициента отскока должны быть в диапазоне от 0 до 1
    if (!unit(restitution) || !unit(blockRestitution)) {
        throw std::invalid_argument("Invalid physics/scene configuration");
    }

    // начальная скорость снаряда, размеры и массы должны быть положительными
    if (!positive(projectileSpeed) ||
        !positive(wallThickness) || !positive(wallHeight) || !size(projectileSize) ||
        !size(blockSize) || !positive(staticSizeMultiplier) ||
        !positive(projectileMass) || !positive(blockMass)) {
        throw std::invalid_argument("Invalid physics/scene configuration");
    }

    // координаты должны быть конечными, интервалы расстановки положительные
    if (!finite(wallProjectilePosition) || !std::isfinite(wallLeft) ||
        !std::isfinite(wallCenterY) || !finite(stackBase) || !finite(looseBlocksOrigin) ||
        !finite(fastOrigin) || !finite(dynamicFastOrigin) || !finite(loosePairOffset) || !positive(supportSpacing) ||
        !positive(looseBlockSpacing) || !positive(fastSpacing)) {
        throw std::invalid_argument("Invalid physics/scene configuration");
    }

    // проверяем число тел, стек должен содержать хотя бы один блок и помещаться в normalCount
    if (staticCount < 3 || staticCount > 10 || normalCount < 10 || normalCount > 100 ||
        fastCount < 1 || fastCount > 10 || stackCount == 0 || stackCount > normalCount) {
        throw std::invalid_argument("Invalid physics/scene configuration");
    }

    // проверяем число проходов решателя и допустимые значения поправок и порогов контакта
    if (velocityIterations < 1 || positionIterations < 1 || !positive(positionCorrection) ||
        positionCorrection > 1.0 || !std::isfinite(penetrationSlop) || penetrationSlop < 0.0 ||
        !std::isfinite(restitutionThreshold) || restitutionThreshold < 0.0 ||
        !positive(contactTolerance) || !positive(velocityEpsilon)) {
        throw std::invalid_argument("Invalid physics/scene configuration");
    }

    // допуск по времени должен быть меньше любого физического шага, а лимит событий CCD больше нуля
    if (!positive(timeEpsilon) || timeEpsilon >= normalDt() ||
        timeEpsilon >= fastDt() || timeEpsilon >= pureDt() ||
        maxCcdEvents == 0) {
        throw std::invalid_argument("Invalid physics/scene configuration");
    }

    // для замера нужны положительная длительность и хотя бы один повтор
    if (!positive(benchmarkSeconds) || benchmarkRepeats == 0) {
        throw std::invalid_argument("Invalid physics/scene configuration");
    }
}

// по значению Mode возвращаем название режима для отображения
const char* modeName(Mode mode) {
    switch (mode) {
    case Mode::Discrete: return "Discrete";
    case Mode::Ccd: return "CCD";
    case Mode::MultiRate: return "Multi-rate";
    case Mode::Pure240: return "Pure high-rate";
    }
    return "Unknown";
}
}
