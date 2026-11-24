#pragma once
// Геометрия столкновений:
//   collide()       — дискретная проверка пересечения «здесь и сейчас»;
//   timeOfImpact()  — CCD: момент первого касания внутри шага.
#include "Body.h"

// Результат дискретной проверки. normal направлена от A к B.
struct Manifold {
    bool hit = false;
    Vec2 normal;
    double penetration = 0.0; // > 0 — тела перекрываются; < 0 — зазор
};

// Пересечение тел A и B. Тела считаются касающимися, если
// penetration > -margin (margin > 0 позволяет ловить «почти касание»).
Manifold collide(const Body& a, const Body& b, double margin = 0.0);

// Результат CCD. t — время от начала (оставшегося) шага до касания,
// normal — нормаль контакта от A к B в момент касания.
struct Toi {
    bool hit = false;
    double t = 0.0;
    Vec2 normal;
};

// Время первого касания при равномерном движении тел со своими скоростями
// на интервале [0, tMax]. Поддерживаются пары круг-круг, круг-AABB, AABB-AABB.
Toi timeOfImpact(const Body& a, const Body& b, double tMax);
