#include "Render.h"
#include <cmath>
#include <vector>
#include "Font.h"

namespace {
constexpr double kPi = 3.14159265358979323846;
SDL_Color toColor(uint32_t rgba) {
    return SDL_Color{Uint8(rgba >> 24), Uint8(rgba >> 16), Uint8(rgba >> 8), Uint8(rgba)};
}
uint32_t withAlpha(uint32_t rgba, int a) { return (rgba & 0xFFFFFF00u) | Uint32(a & 0xFF); }
uint32_t darker(uint32_t rgba) {
    const SDL_Color c = toColor(rgba);
    return (Uint32(c.r * 0.55) << 24) | (Uint32(c.g * 0.55) << 16) | (Uint32(c.b * 0.55) << 8) | c.a;
}
} // namespace

void Render::setColor(uint32_t rgba) {
    const SDL_Color c = toColor(rgba);
    SDL_SetRenderDrawColor(r_, c.r, c.g, c.b, c.a);
}

void Render::clear(uint32_t rgba) {
    setColor(rgba);
    SDL_RenderClear(r_);
}

void Render::fillRect(double x, double y, double w, double h, uint32_t rgba) {
    setColor(rgba);
    const SDL_FRect rc{float(x), float(y), float(w), float(h)};
    SDL_RenderFillRectF(r_, &rc);
}

void Render::rectOutline(double x, double y, double w, double h, uint32_t rgba) {
    setColor(rgba);
    const SDL_FRect rc{float(x), float(y), float(w), float(h)};
    SDL_RenderDrawRectF(r_, &rc);
}

// Закрашенный круг — «веер» треугольников через SDL_RenderGeometry.
void Render::fillCircle(double cx, double cy, double radius, uint32_t rgba) {
    const int seg = radius < 8 ? 16 : 32;
    const SDL_Color c = toColor(rgba);
    std::vector<SDL_Vertex> v;
    v.reserve(seg * 3);
    for (int i = 0; i < seg; ++i) {
        const double a0 = 2.0 * kPi * i / seg;
        const double a1 = 2.0 * kPi * (i + 1) / seg;
        v.push_back({{float(cx), float(cy)}, c, {0, 0}});
        v.push_back({{float(cx + radius * std::cos(a0)), float(cy + radius * std::sin(a0))}, c, {0, 0}});
        v.push_back({{float(cx + radius * std::cos(a1)), float(cy + radius * std::sin(a1))}, c, {0, 0}});
    }
    SDL_RenderGeometry(r_, nullptr, v.data(), int(v.size()), nullptr, 0);
}

void Render::circleOutline(double cx, double cy, double radius, uint32_t rgba) {
    setColor(rgba);
    const int seg = 24;
    SDL_FPoint pts[seg + 1];
    for (int i = 0; i <= seg; ++i) {
        const double a = 2.0 * kPi * i / seg;
        pts[i] = {float(cx + radius * std::cos(a)), float(cy + radius * std::sin(a))};
    }
    SDL_RenderDrawLinesF(r_, pts, seg + 1);
}

void Render::line(double x0, double y0, double x1, double y1, uint32_t rgba) {
    setColor(rgba);
    SDL_RenderDrawLineF(r_, float(x0), float(y0), float(x1), float(y1));
}

int Render::text(int x, int y, const std::string& s, uint32_t rgba, int scale) {
    setColor(rgba);
    std::vector<SDL_Rect> px;
    int cx = x;
    for (char ch : s) {
        const int code = static_cast<unsigned char>(ch);
        if (code >= 32 && code < 127) {
            const unsigned char* g = kFont[code - 32];
            for (int row = 0; row < kGlyphH; ++row)
                for (int col = 0; col < kGlyphW; ++col)
                    if (g[row] & (0x80 >> col))
                        px.push_back({cx + col * scale, y + row * scale, scale, scale});
        }
        cx += kGlyphW * scale;
    }
    if (!px.empty()) SDL_RenderFillRects(r_, px.data(), int(px.size()));
    return cx - x;
}

void Render::textPanel(int x, int y, int w, int h) {
    fillRect(x, y, w, h, 0x000000B4);
}

void Render::drawWorld(const World& w, const DrawOptions& o) {
    const double a = o.interpolate ? o.alpha : 1.0;

    // 1) Следы: положения снаряда после каждого его шага физики.
    if (o.trails) {
        for (const Body& b : w.bodies) {
            if (!b.trackTrail || b.trail.empty()) continue;
            const int n = int(b.trail.size());
            for (int i = 0; i < n; ++i) {
                const int alpha = 40 + 160 * (i + 1) / n;
                circleOutline(b.trail[i].x, b.trail[i].y, b.radius, withAlpha(b.color, alpha));
            }
        }
    }

    // 2) Тела (неподвижные — первыми).
    for (int pass = 0; pass < 2; ++pass) {
        for (const Body& b : w.bodies) {
            if ((pass == 0) != b.isStatic()) continue;
            const Vec2 p = lerp(b.prevPos, b.pos, a);
            if (b.shape == Shape::Circle) {
                fillCircle(p.x, p.y, b.radius, b.color);
                if (b.radius >= 8) {
                    // Радиус-«стрелка» по скорости, чтобы было видно направление движения.
                    const double l = b.vel.length();
                    if (l > 1.0)
                        line(p.x, p.y, p.x + b.vel.x / l * b.radius, p.y + b.vel.y / l * b.radius,
                             darker(b.color));
                }
            } else {
                fillRect(p.x - b.half.x, p.y - b.half.y, 2 * b.half.x, 2 * b.half.y, b.color);
                rectOutline(p.x - b.half.x, p.y - b.half.y, 2 * b.half.x, 2 * b.half.y, darker(b.color));
            }
        }
    }

    // 3) Точки контакта, найденные CCD (жёлтые крестики, гаснут за 0.6 с).
    for (const ContactMark& m : w.marks) {
        const int alpha = int(255 * (1.0 - m.age / 0.6));
        const uint32_t c = withAlpha(0xFFE14DFF, alpha < 0 ? 0 : alpha);
        line(m.p.x - 7, m.p.y - 7, m.p.x + 7, m.p.y + 7, c);
        line(m.p.x - 7, m.p.y + 7, m.p.x + 7, m.p.y - 7, c);
    }
}
