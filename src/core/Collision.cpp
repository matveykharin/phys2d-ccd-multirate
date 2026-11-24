#include "Collision.h"
#include <algorithm>
#include <limits>

namespace {

constexpr double kEps = 1e-12;
constexpr double kInf = std::numeric_limits<double>::infinity();

double sgn(double v) { return v >= 0.0 ? 1.0 : -1.0; }

// ---------------------------------------------------------------------
//  Дискретные проверки
// ---------------------------------------------------------------------

Manifold circleCircle(const Body& a, const Body& b, double margin) {
    Manifold m;
    const Vec2 d = b.pos - a.pos;
    const double dist = d.length();
    const double pen = a.radius + b.radius - dist;
    if (pen <= -margin) return m;
    m.hit = true;
    m.penetration = pen;
    if (dist > kEps) {
        m.normal = d / dist;
    } else {
        // Центры совпали: берём направление по относительной скорости.
        const Vec2 rv = b.vel - a.vel;
        const double l = rv.length();
        m.normal = (l > kEps) ? -rv / l : Vec2(1.0, 0.0);
    }
    return m;
}

// Круг (c) против AABB (box). Нормаль возвращается от КРУГА к ПРЯМОУГОЛЬНИКУ.
Manifold circleBox(const Body& c, const Body& box, double margin) {
    Manifold m;
    const Vec2 h = box.half;
    const Vec2 local = c.pos - box.pos;
    const Vec2 q(std::clamp(local.x, -h.x, h.x), std::clamp(local.y, -h.y, h.y));
    const Vec2 diff = local - q;
    const double dist = diff.length();

    if (dist > kEps) {
        // Центр круга снаружи прямоугольника: ближайшая точка на границе.
        const double pen = c.radius - dist;
        if (pen <= -margin) return m;
        m.hit = true;
        m.penetration = pen;
        m.normal = -(diff / dist); // от круга к прямоугольнику
        return m;
    }

    // Центр круга оказался ВНУТРИ прямоугольника (глубокое проникновение,
    // типично для быстрых тел). Выталкиваем по оси наименьшего проникновения,
    // а сторону выбираем по положению в НАЧАЛЕ шага (prevPos): с какой
    // стороны тело пришло, туда и выталкиваем. Иначе тело, прошедшее
    // середину тонкой стенки, вытолкнулось бы на дальнюю сторону.
    const Vec2 prevLocal = c.prevPos - box.prevPos;
    const double dx = h.x - std::abs(local.x);
    const double dy = h.y - std::abs(local.y);
    Vec2 nBoxToCircle;
    double pen;
    if (dx < dy) {
        const double s = (std::abs(prevLocal.x) > h.x) ? sgn(prevLocal.x) : sgn(local.x);
        nBoxToCircle = Vec2(s, 0.0);
        pen = h.x + c.radius - s * local.x;
    } else {
        const double s = (std::abs(prevLocal.y) > h.y) ? sgn(prevLocal.y) : sgn(local.y);
        nBoxToCircle = Vec2(0.0, s);
        pen = h.y + c.radius - s * local.y;
    }
    m.hit = true;
    m.penetration = pen;
    m.normal = -nBoxToCircle;
    return m;
}

Manifold boxBox(const Body& a, const Body& b, double margin) {
    Manifold m;
    const Vec2 d = b.pos - a.pos;
    const double ox = a.half.x + b.half.x - std::abs(d.x);
    const double oy = a.half.y + b.half.y - std::abs(d.y);
    if (ox <= -margin || oy <= -margin) return m;
    m.hit = true;
    // Сторону берём по положению в начале шага, если тогда тела были
    // разделены по этой оси (защита от «проталкивания насквозь»).
    const Vec2 pd = b.prevPos - a.prevPos;
    const Vec2 H = a.half + b.half;
    if (ox < oy) {
        m.penetration = ox;
        const double s = (std::abs(pd.x) >= H.x) ? sgn(pd.x) : sgn(d.x);
        m.normal = Vec2(s, 0.0);
        if (s != sgn(d.x)) m.penetration = H.x + std::abs(d.x);
    } else {
        m.penetration = oy;
        const double s = (std::abs(pd.y) >= H.y) ? sgn(pd.y) : sgn(d.y);
        m.normal = Vec2(0.0, s);
        if (s != sgn(d.y)) m.penetration = H.y + std::abs(d.y);
    }
    return m;
}

// ---------------------------------------------------------------------
//  CCD
// ---------------------------------------------------------------------

// Наименьший неотрицательный корень уравнения |p + v t| = R на [0, tMax].
// Возведение в квадрат даёт a t^2 + b t + c = 0, где
//   a = v·v,  b = 2 p·v,  c = p·p − R^2.
bool sweepPointCircle(const Vec2& p, const Vec2& v, double R, double tMax, double& tOut) {
    const double a = dot(v, v);
    const double b = 2.0 * dot(p, v);
    const double c = dot(p, p) - R * R;
    if (c <= 0.0) {               // уже касаются/перекрываются
        if (b < 0.0) { tOut = 0.0; return true; } // и сближаются
        return false;
    }
    if (a < kEps || b >= 0.0) return false;       // не движутся навстречу
    const double disc = b * b - 4.0 * a * c;
    if (disc < 0.0) return false;                 // траектории не пересекаются
    const double t = (-b - std::sqrt(disc)) / (2.0 * a); // меньший корень
    if (t < 0.0 || t > tMax) return false;
    tOut = t;
    return true;
}

// Slab-метод: луч p + v t против AABB с полуразмерами E (центр в начале координат).
// Возвращает время входа и ось входа (0 — X, 1 — Y).
bool raySlab(const Vec2& p, const Vec2& v, const Vec2& E, double tMax,
             double& tEnter, int& axis) {
    tEnter = -kInf;
    double tExit = kInf;
    axis = 0;
    const double P[2] = {p.x, p.y};
    const double V[2] = {v.x, v.y};
    const double H[2] = {E.x, E.y};
    for (int k = 0; k < 2; ++k) {
        if (std::abs(V[k]) < kEps) {
            if (std::abs(P[k]) > H[k]) return false; // параллельно и снаружи слоя
            continue;
        }
        double t1 = (-H[k] - P[k]) / V[k];
        double t2 = ( H[k] - P[k]) / V[k];
        if (t1 > t2) std::swap(t1, t2);
        if (t1 > tEnter) { tEnter = t1; axis = k; }
        tExit = std::min(tExit, t2);
    }
    if (tEnter > tExit || tExit < 0.0 || tEnter > tMax) return false;
    return true;
}

// Круг против AABB. Сумма Минковского прямоугольника и круга — прямоугольник
// со скруглёнными углами. Центр круга движется как точка; сначала ищем вход
// в прямоугольник, расширенный на r (slab-метод), а если точка входа
// попала в угловую область — уточняем по окружности радиуса r вокруг угла.
// Нормаль возвращается от круга к прямоугольнику.
Toi circleBoxToi(const Body& c, const Body& box, double tMax) {
    Toi res;
    const Vec2 p = c.pos - box.pos;   // относительное положение
    const Vec2 v = c.vel - box.vel;   // относительная скорость
    const Vec2 h = box.half;
    const double r = c.radius;

    double tEnter;
    int axis;
    if (!raySlab(p, v, Vec2(h.x + r, h.y + r), tMax, tEnter, axis)) return res;
    if (tEnter < 0.0) tEnter = 0.0; // старт внутри расширенного прямоугольника

    const Vec2 q = p + v * tEnter;
    const bool outX = std::abs(q.x) > h.x;
    const bool outY = std::abs(q.y) > h.y;

    if (outX && outY) {
        // Угловая область: касание со скруглением (окружность радиуса r вокруг угла).
        const Vec2 corner(sgn(q.x) * h.x, sgn(q.y) * h.y);
        double t;
        if (!sweepPointCircle(p - corner, v, r, tMax, t)) return res;
        const Vec2 n = (p + v * t - corner) / r; // от угла к центру круга
        res.hit = true;
        res.t = t;
        res.normal = -n;
        return res;
    }
    if (tEnter == 0.0) {
        // Уже касаемся грани — считаем контакт в момент 0, если сближаемся.
        const Manifold m = circleBox(c, box, 0.0);
        if (!m.hit || dot(v, m.normal) <= 0.0) return res;
        res.hit = true;
        res.t = 0.0;
        res.normal = m.normal;
        return res;
    }
    // Касание грани.
    Vec2 nBoxToCircle = (axis == 0) ? Vec2(sgn(q.x), 0.0) : Vec2(0.0, sgn(q.y));
    res.hit = true;
    res.t = tEnter;
    res.normal = -nBoxToCircle;
    return res;
}

Toi circleCircleToi(const Body& a, const Body& b, double tMax) {
    Toi res;
    const Vec2 p = a.pos - b.pos;
    const Vec2 v = a.vel - b.vel;
    double t;
    if (!sweepPointCircle(p, v, a.radius + b.radius, tMax, t)) return res;
    const Vec2 d = p + v * t;
    const double l = d.length();
    res.hit = true;
    res.t = t;
    res.normal = (l > kEps) ? -(d / l) : Vec2(1.0, 0.0);
    return res;
}

// AABB против AABB: сумма Минковского двух прямоугольников — прямоугольник
// с полуразмерами hA + hB, далее тот же slab-метод для относительного движения.
Toi boxBoxToi(const Body& a, const Body& b, double tMax) {
    Toi res;
    const Vec2 p = a.pos - b.pos;
    const Vec2 v = a.vel - b.vel;
    double tEnter;
    int axis;
    if (!raySlab(p, v, a.half + b.half, tMax, tEnter, axis)) return res;
    if (tEnter < 0.0) {
        const Manifold m = boxBox(a, b, 0.0);
        if (!m.hit || dot(b.vel - a.vel, m.normal) >= 0.0) return res;
        res.hit = true;
        res.t = 0.0;
        res.normal = m.normal;
        return res;
    }
    const Vec2 q = p + v * tEnter;
    const Vec2 nBtoA = (axis == 0) ? Vec2(sgn(q.x), 0.0) : Vec2(0.0, sgn(q.y));
    res.hit = true;
    res.t = tEnter;
    res.normal = -nBtoA;
    return res;
}

} // namespace

Manifold collide(const Body& a, const Body& b, double margin) {
    if (a.shape == Shape::Circle && b.shape == Shape::Circle) return circleCircle(a, b, margin);
    if (a.shape == Shape::Circle && b.shape == Shape::Box)    return circleBox(a, b, margin);
    if (a.shape == Shape::Box && b.shape == Shape::Circle) {
        Manifold m = circleBox(b, a, margin);
        m.normal = -m.normal;
        return m;
    }
    return boxBox(a, b, margin);
}

Toi timeOfImpact(const Body& a, const Body& b, double tMax) {
    if (a.shape == Shape::Circle && b.shape == Shape::Circle) return circleCircleToi(a, b, tMax);
    if (a.shape == Shape::Circle && b.shape == Shape::Box)    return circleBoxToi(a, b, tMax);
    if (a.shape == Shape::Box && b.shape == Shape::Circle) {
        Toi t = circleBoxToi(b, a, tMax);
        t.normal = -t.normal;
        return t;
    }
    return boxBoxToi(a, b, tMax);
}
