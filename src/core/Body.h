#pragma once
// Физическое тело: круг или прямоугольник без вращения (AABB).
#include <cstdint>
#include <deque>
#include "Vec2.h"

enum class Shape { Circle, Box };

// Группа тела определяет частоту его обновления в режиме Multi-rate.
enum class Group {
    Static, // неподвижные препятствия (бесконечная масса)
    Normal, // обычные тела: 60 Гц
    Fast    // быстрые тела (снаряды): 240 Гц
};

struct Body {
    Shape shape = Shape::Circle;
    Group group = Group::Normal;

    Vec2 pos;      // центр тела
    Vec2 vel;      // скорость, px/с
    Vec2 prevPos;  // положение в начале шага (для интерполяции при отрисовке)

    double radius = 0.0; // для круга
    Vec2   half;         // полуразмеры для AABB

    double invMass = 0.0; // 1/m; 0 у статических тел
    uint32_t color = 0xFFFFFFFF; // RGBA

    // Перезапуск тела (снаряды в демо-сценах): по таймеру или при вылете за сцену.
    bool   respawns = false;
    Vec2   spawnPos, spawnVel;
    double lifetime = 0.0; // с; 0 — только при вылете за границы
    double age = 0.0;

    // След из положений тела после каждого его шага (наглядно показывает,
    // где тело «было» в дискретные моменты времени).
    bool trackTrail = false;
    std::deque<Vec2> trail;

    bool isStatic() const { return group == Group::Static; }
    bool isDynamic() const { return group != Group::Static; }
};

// Удобные «конструкторы» тел.
inline Body makeCircle(Vec2 p, double r, Group g, double density = 1.0) {
    Body b;
    b.shape = Shape::Circle;
    b.group = g;
    b.pos = b.prevPos = p;
    b.radius = r;
    b.invMass = (g == Group::Static) ? 0.0 : 1.0 / (density * 3.14159265358979 * r * r);
    return b;
}

inline Body makeBox(Vec2 p, Vec2 half, Group g, double density = 1.0) {
    Body b;
    b.shape = Shape::Box;
    b.group = g;
    b.pos = b.prevPos = p;
    b.half = half;
    b.invMass = (g == Group::Static) ? 0.0 : 1.0 / (density * 4.0 * half.x * half.y);
    return b;
}
