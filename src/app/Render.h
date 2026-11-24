#pragma once
// Отрисовка сцены и текста средствами SDL2 (без SDL2_gfx / SDL_ttf).
#include <string>
#include <SDL.h>
#include "core/World.h"

struct DrawOptions {
    double alpha = 1.0;      // доля интерполяции между prevPos и pos
    bool interpolate = true; // плавная отрисовка между шагами физики
    bool trails = true;      // следы снарядов по дискретным положениям
};

class Render {
public:
    explicit Render(SDL_Renderer* r) : r_(r) {}

    void clear(uint32_t rgba);
    void fillRect(double x, double y, double w, double h, uint32_t rgba);
    void rectOutline(double x, double y, double w, double h, uint32_t rgba);
    void fillCircle(double cx, double cy, double radius, uint32_t rgba);
    void circleOutline(double cx, double cy, double radius, uint32_t rgba);
    void line(double x0, double y0, double x1, double y1, uint32_t rgba);

    // Текст растровым шрифтом 7x14. Возвращает ширину строки в пикселях.
    int text(int x, int y, const std::string& s, uint32_t rgba, int scale = 1);
    void textPanel(int x, int y, int w, int h);

    void drawWorld(const World& w, const DrawOptions& o);

private:
    void setColor(uint32_t rgba);
    SDL_Renderer* r_;
};
