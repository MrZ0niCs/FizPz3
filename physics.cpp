#include "physics.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>


namespace pz3 {

// проверка пересечения AABB
bool overlapContact(const Aabb& a, const Aabb& b, double tolerance, Contact& out) {
    const Vec2 delta = b.center - a.center;
    const double x = a.half.x + b.half.x - std::abs(delta.x);
    const double y = a.half.y + b.half.y - std::abs(delta.y);
    if (x < -tolerance || y < -tolerance) return false;
    if (x < y) out = {{delta.x >= 0.0 ? 1.0 : -1.0, 0.0}, std::max(0.0, x)};
    else out = {{0.0, delta.y >= 0.0 ? 1.0 : -1.0}, std::max(0.0, y)};
    return true;
}

// тест на столкновение AABB
SweepHit sweptAabb(const Body& a, const Body& b, double dt, const Config& config) {
    const Vec2 va = a.mobility() > 0.0 ? a.velocity : Vec2{};
    const Vec2 vb = b.mobility() > 0.0 ? b.velocity : Vec2{};
    // второе тело как неподвижное
    const Vec2 v = va - vb;
    Contact contact;
    if (overlapContact(a.bounds, b.bounds, config.contactTolerance, contact)) {
        if (dot(v, contact.normal) > config.velocityEpsilon)
            return {true, 0.0, contact.normal};
        return {};
    }

    // время по одной оси, затем по другой, затем объединяем интервалы
    const Vec2 amin = a.bounds.min(), amax = a.bounds.max();
    const Vec2 bmin = b.bounds.min(), bmax = b.bounds.max();
    const double infinity = std::numeric_limits<double>::infinity();
    auto slab = [&](double loA, double hiA, double loB, double hiB,
                    double speed, double& entry, double& exit) {
        if (std::abs(speed) <= config.velocityEpsilon) {
            entry = -infinity;
            exit = infinity;
            return hiA >= loB && hiB >= loA;
        }
        entry = (loB - hiA) / speed;
        exit = (hiB - loA) / speed;
        if (entry > exit) std::swap(entry, exit);
        return true;
    };

    // пересечение интервалов по осям
    double ex, xx, ey, xy;
    if (!slab(amin.x, amax.x, bmin.x, bmax.x, v.x, ex, xx) ||
        !slab(amin.y, amax.y, bmin.y, bmax.y, v.y, ey, xy)) return {};
    const double entry = std::max(ex, ey), exit = std::min(xx, xy);
    if (entry > exit || exit < 0.0 || entry < 0.0 || entry > dt) return {};
    const Vec2 normal = ex > ey ? Vec2{v.x > 0.0 ? 1.0 : -1.0, 0.0}
                               : Vec2{0.0, v.y > 0.0 ? 1.0 : -1.0};
    return {true, entry, normal};
}


// запуск симуляции с проверкой конфигурации и тел
World::World(Config config, Scene scene, Mode mode)
    : config_(std::move(config)), bodies_(std::move(scene.bodies)),
      gravity_(scene.gravity), mode_(mode) {
    config_.validate();
    if (!std::isfinite(gravity_.x) || !std::isfinite(gravity_.y))
        throw std::invalid_argument("Non-finite scene gravity");
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        const Body& b = bodies_[i];
        if (!std::isfinite(b.bounds.center.x) || !std::isfinite(b.bounds.center.y) ||
            !std::isfinite(b.bounds.half.x) || !std::isfinite(b.bounds.half.y) ||
            b.bounds.half.x <= 0.0 || b.bounds.half.y <= 0.0 ||
            !std::isfinite(b.velocity.x) || !std::isfinite(b.velocity.y) ||
            !std::isfinite(b.inverseMass) || b.inverseMass < 0.0 ||
            (b.type != BodyType::Static && b.inverseMass == 0.0) ||
            !std::isfinite(b.restitution) || b.restitution < 0.0 || b.restitution > 1.0)
            throw std::invalid_argument("Invalid body");
        if (b.type == BodyType::Normal) normal_.push_back(i);
        if (b.type == BodyType::Fast) fast_.push_back(i);
        if (b.type != BodyType::Static) dynamic_.push_back(i);
        else bodies_[i].velocity = {};
    }
}

// проверка разрешено ли взаимодействие между телами
bool World::accepts(std::size_t a, std::size_t b, Pairs pairs) const {
    if (bodies_[a].mobility() + bodies_[b].mobility() == 0.0) return false;
    const bool fast = bodies_[a].type == BodyType::Fast || bodies_[b].type == BodyType::Fast;
    if (pairs == Pairs::All) {
        return true;
    }
    if (pairs == Pairs::Fast) {
        return fast;
    }
    return !fast;
}

// применение гравитации к телам
void World::kick(const std::vector<std::size_t>& group, double dt) {
    for (auto i : group) bodies_[i].velocity += gravity_ * dt;
}
// перемещение тел с их скоростью
void World::drift(const std::vector<std::size_t>& group, double dt) {
    for (auto i : group) bodies_[i].bounds.center += bodies_[i].velocity * dt;
}
// применение импульса между телами
void World::impulse(Body& a, Body& b, Vec2 n, double restitution) {
    const double closing = dot(b.velocity - a.velocity, n);
    if (closing >= -config_.velocityEpsilon) return;
    const double j = -(1.0 + restitution) * closing / (a.mobility() + b.mobility());
    a.velocity -= n * (j * a.mobility());
    b.velocity += n * (j * b.mobility());
}

// решение системы контактов для заданного типа пар
void World::solve(Pairs pairs) {
    struct Pair {
        std::size_t a, b;
        Vec2 normal;
        double target;
    };
    std::vector<Pair> contacts;
    // собираем контакты и задаём скорость разлёта вдоль нормали
    for (std::size_t a = 0; a < bodies_.size(); ++a) {
        for (std::size_t b = a + 1; b < bodies_.size(); ++b) {
            if (!accepts(a, b, pairs)) continue;
            Contact c;
            if (!overlapContact(bodies_[a].bounds, bodies_[b].bounds,
                                config_.contactTolerance, c)) continue;
            const double closing = dot(bodies_[b].velocity - bodies_[a].velocity, c.normal);
            const double e = std::min(bodies_[a].restitution, bodies_[b].restitution);
            contacts.push_back({a, b, c.normal,
                closing < -config_.restitutionThreshold ? -e * closing : 0.0});
        }
    }
    stats_.contacts += contacts.size();

    // за несколько проходов поправляем скорости соседних тел
    for (int iteration = 0; iteration < config_.velocityIterations; ++iteration) {
        for (const Pair& pair : contacts) {
            Body& a = bodies_[pair.a];
            Body& b = bodies_[pair.b];
            const double closing = dot(b.velocity - a.velocity, pair.normal);
            if (closing >= pair.target) continue;
            const double j = (pair.target - closing) / (a.mobility() + b.mobility());
            a.velocity -= pair.normal * (j * a.mobility());
            b.velocity += pair.normal * (j * b.mobility());
        }
    }
    // заново ищем пересечения и понемногу раздвигаем тела
    for (int iteration = 0; iteration < config_.positionIterations; ++iteration) {
        for (std::size_t ai = 0; ai < bodies_.size(); ++ai) {
            for (std::size_t bi = ai + 1; bi < bodies_.size(); ++bi) {
                if (!accepts(ai, bi, pairs)) continue;
                Body& a = bodies_[ai];
                Body& b = bodies_[bi];
                Contact c;
                if (!overlapContact(a.bounds, b.bounds, 0.0, c)) continue;
                const double depth = std::max(0.0, c.penetration - config_.penetrationSlop);
                const double amount = depth * config_.positionCorrection /
                                      (a.mobility() + b.mobility());
                a.bounds.center -= c.normal * (amount * a.mobility());
                b.bounds.center += c.normal * (amount * b.mobility());
            }
        }
    }
}

// обновляем скорость, двигаем тела за весь шаг и проверяем контакты
void World::discrete(double dt) {
    kick(dynamic_, dt);
    drift(dynamic_, dt);
    solve(Pairs::All);
}

// ищем ближайший удар, двигаемся до него и дорабатываем остаток шага
void World::ccd(double dt) {
    kick(dynamic_, dt);
    solve(Pairs::All);
    double remaining = dt;
    while (remaining > 0.0) {
        SweepHit first;
        std::size_t ia = 0, ib = 0;
        for (std::size_t a = 0; a < bodies_.size(); ++a) {
            for (std::size_t b = a + 1; b < bodies_.size(); ++b) {
                if (!accepts(a, b, Pairs::All)) continue;
                const auto hit = sweptAabb(bodies_[a], bodies_[b], remaining, config_);
                if (hit.hit && (!first.hit || hit.time < first.time)) {
                    first = hit;
                    ia = a;
                    ib = b;
                }
            }
        }
        if (!first.hit) {
            drift(dynamic_, remaining);
            break;
        }
        if (stats_.ccdEvents >= config_.maxCcdEvents) {
            ++stats_.ccdLimitHits;
            throw std::runtime_error("CCD event limit exceeded; reset scene or raise maxCcdEvents");
        }
        drift(dynamic_, first.time);
        // The bounce changes velocity, but the rest of this timestep still has to run.
        remaining = std::max(0.0, remaining - first.time);
        Body& a = bodies_[ia];
        Body& b = bodies_[ib];
        const double speed = -dot(b.velocity - a.velocity, first.normal);
        const double e = speed > config_.restitutionThreshold ?
                         std::min(a.restitution, b.restitution) : 0.0;
        impulse(a, b, first.normal, e);
        ++stats_.ccdEvents;
        ++stats_.contacts;
        solve(Pairs::All);
    }
    solve(Pairs::All);
}

// обычные тела получают гравитацию раз за базовый шаг быстрые на каждом своём подшаге
// перед проверкой контактов продвигаем обычные тела к моменту быстрого подшага
void World::multiRate(double dt) {
    kick(normal_, dt);
    ++stats_.normalSubsteps;
    const double h = config_.fastDt();
    double eventTime = h - fastAccumulator_;
    double normalTime = 0.0;
    fastAccumulator_ += dt;
    while (fastAccumulator_ + config_.timeEpsilon >= h) {
        const double tick = std::min(dt, eventTime);
        drift(normal_, tick - normalTime);
        normalTime = tick;
        kick(fast_, h);
        drift(fast_, h);
        solve(Pairs::Fast);
        ++stats_.fastSubsteps;
        fastAccumulator_ = std::max(0.0, fastAccumulator_ - h);
        eventTime += h;
    }
    drift(normal_, dt - normalTime);
    solve(Pairs::Normal);
}

// выполняем обычный дискретный подшаг для всех тел
// каждый раз, когда накопилось достаточно времени для шага pureDt
void World::pureRate(double dt) {
    pureAccumulator_ += dt;
    const double h = config_.pureDt();
    while (pureAccumulator_ + config_.timeEpsilon >= h) {
        discrete(h);
        ++stats_.normalSubsteps;
        ++stats_.fastSubsteps;
        pureAccumulator_ = std::max(0.0, pureAccumulator_ - h);
    }
}

// выполняет один базовый шаг в выбранном режиме и возвращает статистику
StepStats World::step() {
    stats_ = {};
    const double dt = config_.normalDt();
    switch (mode_) {
    case Mode::Discrete:
        discrete(dt);
        stats_.normalSubsteps = 1;
        break;
    case Mode::Ccd:
        ccd(dt);
        stats_.normalSubsteps = 1;
        break;
    case Mode::MultiRate: multiRate(dt); break;
    case Mode::Pure240: pureRate(dt); break;
    }
    return stats_;
}

// копит прошедшее время и запускает столько целых шагов, сколько поместится
std::size_t World::advance(double elapsedSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0)
        throw std::invalid_argument("Elapsed time must be finite and nonnegative");
    frameAccumulator_ += elapsedSeconds;
    std::size_t steps = 0;
    while (frameAccumulator_ + config_.timeEpsilon >= config_.normalDt()) {
        step();
        frameAccumulator_ = std::max(0.0, frameAccumulator_ - config_.normalDt());
        ++steps;
    }
    return steps;
}
}
