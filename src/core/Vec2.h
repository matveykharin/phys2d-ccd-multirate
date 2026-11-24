#pragma once
// Простой 2D-вектор на double (double выбран ради точности при поиске
// времени столкновения в CCD и для детерминированности).
#include <cmath>

struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Vec2() = default;
    constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}

    constexpr Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator*(double s) const { return {x * s, y * s}; }
    constexpr Vec2 operator/(double s) const { return {x / s, y / s}; }
    constexpr Vec2 operator-() const { return {-x, -y}; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
    Vec2& operator*=(double s) { x *= s; y *= s; return *this; }

    double lengthSq() const { return x * x + y * y; }
    double length() const { return std::sqrt(lengthSq()); }
};

inline constexpr Vec2 operator*(double s, const Vec2& v) { return {v.x * s, v.y * s}; }
inline double dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
inline Vec2 lerp(const Vec2& a, const Vec2& b, double t) { return a + (b - a) * t; }
