#include "Scenes.h"
#include <cmath>

namespace {

constexpr double W = cfg::kWindowWidth;
constexpr double H = cfg::kWindowHeight;

constexpr uint32_t kStaticColor = 0x3FA34DFF; // зелёный
constexpr uint32_t kFastColor   = 0xFF4A3DFF; // красный
constexpr uint32_t kSlowColor   = 0x4C9BE8FF; // голубой (снаряд в группе 60 Гц)
const uint32_t kPalette[] = {0x4C9BE8FF, 0xF2B134FF, 0x9C6ADEFF, 0x3DC6C1FF, 0xE8774CFF};

// Детерминированный генератор (одинаковые сцены при каждом запуске).
struct Lcg {
    uint32_t s;
    uint32_t next() { s = s * 1664525u + 1013904223u; return s >> 8; }
    double uniform(double a, double b) { return a + (b - a) * (next() % 100000) / 100000.0; }
};

int addStaticBox(World& w, Vec2 center, Vec2 half) {
    Body b = makeBox(center, half, Group::Static);
    b.color = kStaticColor;
    return w.addBody(b);
}

// Стартовая координата X снаряда в «худшей фазе» для дискретной проверки:
// середина между двумя соседними положениями снаряда совпадает с центром
// стенки. Тогда снаряд проскакивает тогда и только тогда, когда v·Δt > d + 2r.
double worstPhaseStartX(double wallX, double step, double minX) {
    double x = wallX - step * 0.5;
    while (x - step >= minX) x -= step;
    return x;
}

Body makeProjectile(Vec2 pos, Vec2 vel, Group g, uint32_t color, double lifetime) {
    Body b = makeCircle(pos, cfg::kProjectileRadius, g, 1.0);
    b.vel = vel;
    b.color = color;
    b.respawns = true;
    b.spawnPos = pos;
    b.spawnVel = vel;
    b.lifetime = lifetime;
    b.trackTrail = true;
    return b;
}

// ---------------------------------------------------------------------
void buildTunnel(World& w, const SceneParams& p) {
    w.gravity = Vec2(0, 0);
    const double wallX = W * 0.5;
    addStaticBox(w, Vec2(wallX, H * 0.5), Vec2(cfg::kWallThickness * 0.5, 200));

    // Путь снаряда за один его шаг: в Multi-rate снаряд (группа Fast)
    // обновляется с частотой physicsHz * fastSubsteps.
    const double hz = p.physicsHz * (w.mode == Mode::MultiRate ? p.fastSubsteps : 1);
    const double step = p.projectileSpeed / hz;
    const double x0 = worstPhaseStartX(wallX, step, 120.0);
    w.addBody(makeProjectile(Vec2(x0, H * 0.5), Vec2(p.projectileSpeed, 0), Group::Fast,
                             kFastColor, 1.6));
    w.bounds = {-50, -50, W + 50, H + 50};
}

// ---------------------------------------------------------------------
void buildExperiment(World& w, const SceneParams& p) {
    w.gravity = Vec2(0, 0);
    const double wallX = W * 0.5;
    const double laneY[2] = {250, 480};
    for (double y : laneY)
        addStaticBox(w, Vec2(wallX, y), Vec2(cfg::kWallThickness * 0.5, 90));

    // Один и тот же снаряд (та же точка старта и скорость) в двух группах.
    const double step60 = p.projectileSpeed / p.physicsHz;
    const double x0 = worstPhaseStartX(wallX, step60, 120.0);
    w.addBody(makeProjectile(Vec2(x0, laneY[0]), Vec2(p.projectileSpeed, 0), Group::Normal,
                             kSlowColor, 1.6));
    w.addBody(makeProjectile(Vec2(x0, laneY[1]), Vec2(p.projectileSpeed, 0), Group::Fast,
                             kFastColor, 1.6));
    w.bounds = {-50, -50, W + 50, H + 50};
}

// ---------------------------------------------------------------------
void buildArena(World& w, const SceneParams& p) {
    w.gravity = Vec2(0, cfg::kGravity);
    const double t = 20.0; // толщина стен арены

    // Неподвижные тела (6 шт.): 4 стены и 2 препятствия 64x64 —
    // вдвое больше самого крупного обычного тела (диаметр 32 px).
    addStaticBox(w, Vec2(W * 0.5, t * 0.5), Vec2(W * 0.5, t * 0.5));
    addStaticBox(w, Vec2(W * 0.5, H - t * 0.5), Vec2(W * 0.5, t * 0.5));
    addStaticBox(w, Vec2(t * 0.5, H * 0.5), Vec2(t * 0.5, H * 0.5 - t));
    addStaticBox(w, Vec2(W - t * 0.5, H * 0.5), Vec2(t * 0.5, H * 0.5 - t));
    addStaticBox(w, Vec2(430, 470), Vec2(32, 32));
    addStaticBox(w, Vec2(850, 470), Vec2(32, 32));

    // Обычные тела (50 шт.): 40 кругов и 10 прямоугольников разного размера.
    Lcg rng{12345u};
    int k = 0;
    for (int row = 0; row < 5; ++row) {
        for (int col = 0; col < 10; ++col, ++k) {
            const Vec2 pos(150 + col * 110 + (row % 2) * 30, 90 + row * 58);
            Body b;
            if (k % 5 == 4) {
                const double hs = rng.uniform(9, 14);
                b = makeBox(pos, Vec2(hs, hs), Group::Normal);
            } else {
                b = makeCircle(pos, rng.uniform(10, 16), Group::Normal);
            }
            b.vel = Vec2(rng.uniform(-80, 80), rng.uniform(-40, 40));
            b.color = kPalette[k % 5];
            w.addBody(b);
        }
    }

    // Быстрые тела (8 шт.): снаряды 2000 px/с в разных направлениях.
    for (int i = 0; i < 8; ++i) {
        const double ang = (22.5 + 45.0 * i) * 3.14159265358979 / 180.0;
        const Vec2 pos(220 + i * 120, 330);
        const Vec2 vel(std::cos(ang) * p.projectileSpeed, std::sin(ang) * p.projectileSpeed);
        w.addBody(makeProjectile(pos, vel, Group::Fast, kFastColor, 2.0));
    }
    // Вылет за пределы арены = снаряд проскочил стену.
    w.bounds = {0, 0, W, H};
}

// ---------------------------------------------------------------------
void buildStack(World& w, const SceneParams&) {
    w.gravity = Vec2(0, cfg::kGravity);
    const double floorTop = H - 40;
    addStaticBox(w, Vec2(W * 0.5, H - 20), Vec2(W * 0.5, 20));         // пол
    addStaticBox(w, Vec2(10, H * 0.5), Vec2(10, H * 0.5 - 40));        // левая стена
    addStaticBox(w, Vec2(W - 10, H * 0.5), Vec2(10, H * 0.5 - 40));    // правая стена
    addStaticBox(w, Vec2(860, floorTop - 130), Vec2(10, 130));         // «стакан»
    addStaticBox(w, Vec2(1150, floorTop - 130), Vec2(10, 130));

    // Башня из 8 блоков 40x40.
    for (int i = 0; i < 8; ++i) {
        Body b = makeBox(Vec2(250, floorTop - 20 - 40 * i), Vec2(20, 20), Group::Normal);
        b.color = kPalette[i % 2 ? 1 : 0];
        w.addBody(b);
    }
    // Пирамида 5-4-3-2-1.
    for (int row = 0; row < 5; ++row) {
        for (int i = 0; i < 5 - row; ++i) {
            const Vec2 pos(450 + row * 21 + i * 42, floorTop - 20 - 40 * row);
            Body b = makeBox(pos, Vec2(20, 20), Group::Normal);
            b.color = kPalette[2 + (row % 2)];
            w.addBody(b);
        }
    }
    // 40 шаров, падающих в «стакан».
    Lcg rng{777u};
    for (int row = 0; row < 5; ++row) {
        for (int col = 0; col < 8; ++col) {
            Body b = makeCircle(Vec2(895 + col * 32 + (row % 2) * 8, 120 + row * 34),
                                rng.uniform(10, 14), Group::Normal);
            b.color = kPalette[(row + col) % 5];
            w.addBody(b);
        }
    }
    w.bounds = {-200, -2000, W + 200, H + 200};
}

} // namespace

const char* sceneTitle(int id) {
    switch (id) {
        case kSceneTunnel:     return "1: Tunneling - projectile vs 10 px wall";
        case kSceneExperiment: return "2: Experiment - same projectile at 60 Hz (top) and 240 Hz (bottom)";
        case kSceneArena:      return "3: Multi-rate arena - 6 static, 50 normal, 8 fast";
        case kSceneStack:      return "4: Stack stability - tower, pyramid, cup with balls";
    }
    return "?";
}

Mode scenePreferredMode(int id) {
    switch (id) {
        case kSceneTunnel:     return Mode::Discrete;
        case kSceneExperiment: return Mode::MultiRate;
        case kSceneArena:      return Mode::MultiRate;
        case kSceneStack:      return Mode::Discrete;
    }
    return Mode::Discrete;
}

void buildScene(int id, World& w, const SceneParams& p) {
    const Mode keepMode = w.mode;
    w.clear();
    w.mode = keepMode;
    w.restitution = p.restitution;
    w.fastSubsteps = p.fastSubsteps;
    switch (id) {
        case kSceneTunnel:     buildTunnel(w, p); break;
        case kSceneExperiment: buildExperiment(w, p); break;
        case kSceneArena:      buildArena(w, p); break;
        case kSceneStack:      buildStack(w, p); break;
    }
}

void fireProjectiles(int id, World& w, const SceneParams& p) {
    if (id == kSceneStack) {
        // Выстрел в башню слева.
        Body b = makeCircle(Vec2(60, H - 40 - 100), cfg::kProjectileRadius, Group::Fast, 1.0);
        b.vel = Vec2(p.projectileSpeed, -80);
        b.color = kFastColor;
        b.trackTrail = true;
        w.addBody(b);
        return;
    }
    for (int i : w.dynamicIds)
        if (w.bodies[i].respawns) w.respawn(w.bodies[i]);
}
