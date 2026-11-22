#include "scenes.h"

namespace pz3 {
namespace {

// создание прямоугольника
Body makeBody(Vec2 center, Vec2 size, BodyType type, double mass,
              double restitution, Vec2 velocity = {}) {
    return {{center, size * 0.5}, velocity,
            type == BodyType::Static ? 0.0 : 1.0 / mass, restitution, type};
}
}

// первая сцена, снаряд в стену
Scene makeWallScene(const Config& c) {
    c.validate();
    Scene scene{{}, c.wallGravity};
    scene.bodies.push_back(makeBody(c.wallProjectilePosition, c.projectileSize,
        BodyType::Fast, c.projectileMass, c.restitution, {c.projectileSpeed, 0.0}));
    scene.bodies.push_back(makeBody({c.wallLeft + c.wallThickness * 0.5, c.wallCenterY},
        {c.wallThickness, c.wallHeight}, BodyType::Static, c.blockMass, c.restitution));
    return scene;
}

// вторая сцена
Scene makeCombinedScene(const Config& c) {
    c.validate();
    Scene scene{{}, c.gravity};
    const Vec2 staticSize = c.blockSize * c.staticSizeMultiplier;
    // расставляем неподвижные опоры в ряд
    for (std::size_t i = 0; i < c.staticCount; ++i) {
        scene.bodies.push_back(makeBody(
            {c.stackBase.x + static_cast<double>(i) * c.supportSpacing,
             c.stackBase.y + staticSize.y * 0.5},
            staticSize, BodyType::Static, c.blockMass, c.restitution));
    }
    // первые блоки ставим друг на друга остальные отдельно.
    for (std::size_t i = 0; i < c.normalCount; ++i) {
        Vec2 position;
        if (i < c.stackCount) {
            position = {c.stackBase.x, c.stackBase.y - (static_cast<double>(i) + 0.5) * c.blockSize.y};
        } else {
            position = {c.looseBlocksOrigin.x + static_cast<double>(i - c.stackCount) * c.looseBlockSpacing,
                        c.looseBlocksOrigin.y};
        }
        // если отдельных блоков хотя бы два то ставим последний со смещением над предыдущим
        if (c.normalCount - c.stackCount >= 2 && i == c.normalCount - 1) {
            position = scene.bodies.back().bounds.center + c.loosePairOffset;
        }
        scene.bodies.push_back(makeBody(position, c.blockSize,
            BodyType::Normal, c.blockMass, c.blockRestitution));
    }
    // запуск снарядов
    for (std::size_t i = 0; i < c.fastCount; ++i) {
        const Vec2 position = i == 1 ? c.dynamicFastOrigin :
            Vec2{c.fastOrigin.x, c.fastOrigin.y + static_cast<double>(i) * c.fastSpacing};
        scene.bodies.push_back(makeBody(
            position,
            c.projectileSize, BodyType::Fast, c.projectileMass, c.restitution,
            {c.projectileSpeed, 0.0}));
    }
    return scene;
}
}
