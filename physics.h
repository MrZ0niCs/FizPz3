#pragma once
#include "config.h"

namespace pz3 {

// Contact хранит нормаль и глубину текущего пересечения
struct Contact {
    Vec2 normal;
    double penetration = 0.0;
};
// SweepHit хранит признак столкновения, время до него и нормаль
struct SweepHit {
    bool hit = false;
    double time = 0.0;
    Vec2 normal;
};

// проверяем текущие положения двух AABB и если есть контакт заполняем contact
bool overlapContact(const Aabb& a, const Aabb& b, double tolerance, Contact& contact);
// ищем первое касание двух тел за время dt с учётом их скоростей
SweepHit sweptAabb(const Body& a, const Body& b, double dt, const Config& config);

// хранит и обновляет состояние мира (тела, гравитацию, выбранный режим)
class World {
public:
    World(Config config, Scene scene, Mode mode);
    StepStats step(); // один базовый шаг симуляции длительностью normalDt
    std::size_t advance(double elapsedSeconds); // накапливает прошедшее время и выполняет целые базовые шаги
    const std::vector<Body>& bodies() const { return bodies_; }
    const StepStats& lastStats() const { return stats_; }
    const Config& config() const { return config_; }
    Mode mode() const { return mode_; }
private:
    enum class Pairs { All, Fast, Normal }; // Выбираем все пары для обработки с быстрым телом или без быстрых тел
    bool accepts(std::size_t a, std::size_t b, Pairs pairs) const;
    void kick(const std::vector<std::size_t>& group, double dt); // kick обновляет скорости от гравитации
    void drift(const std::vector<std::size_t>& group, double dt); // drift сдвигает тела
    void impulse(Body& a, Body& b, Vec2 normal, double restitution); // impulse меняет скорости при ударе
    void solve(Pairs pairs); // solve обрабатывает контакт
    void discrete(double dt);
    void ccd(double dt);
    void multiRate(double dt);
    void pureRate(double dt);

    Config config_;
    std::vector<Body> bodies_;
    Vec2 gravity_;
    Mode mode_;
    std::vector<std::size_t> normal_, fast_, dynamic_;
    StepStats stats_;
    double fastAccumulator_ = 0.0;
    double pureAccumulator_ = 0.0;
    double frameAccumulator_ = 0.0;
};
}