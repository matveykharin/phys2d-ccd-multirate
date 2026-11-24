// Консольные замеры для отчёта (без графики):
//  - минимальный эксперимент: туннелирование на 60/120/240 Гц и с CCD;
//  - время шага физики и время на кадр (кадр = 1/60 с модельного времени);
//  - детерминированность (хэш состояния двух одинаковых прогонов);
//  - устойчивость стопки (RMS-скорость тел в покое, проникновение).
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <functional>
#include <string>
#include "../core/Collision.h"
#include "../core/Scenes.h"

using Clock = std::chrono::steady_clock;

struct RunResult {
    double msPerFrame = 0;   // среднее время физики на кадр 1/60 с
    double msPerStep = 0;    // среднее время одного вызова World::step
    double stepsPerFrame = 0;
    double fastStepsPerFrame = 0;
    double toiPerFrame = 0;
    int toiMaxPerStep = 0;
    int toiCapHits = 0;
    uint64_t hash = 0;
};

// Прогнать frames кадров по 1/60 с. На кадр приходится physicsHz/60 шагов.
RunResult run(World& w, double physicsHz, int frames,
              const std::function<void(World&)>& afterStep = nullptr) {
    RunResult r;
    const int stepsPerFrame = static_cast<int>(std::lround(physicsHz / 60.0));
    const double dt = 1.0 / physicsHz;
    double total = 0;
    long steps = 0, fast = 0, toi = 0;
    for (int f = 0; f < frames; ++f) {
        for (int s = 0; s < stepsPerFrame; ++s) {
            const auto t0 = Clock::now();
            w.step(dt);
            total += std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            ++steps;
            fast += w.stats.fastSteps;
            toi += w.stats.toiEvents;
            r.toiMaxPerStep = std::max(r.toiMaxPerStep, w.stats.toiEvents);
            r.toiCapHits += w.stats.toiCapHits;
            if (afterStep) afterStep(w);
        }
    }
    r.msPerFrame = total / frames;
    r.msPerStep = total / double(steps);
    r.stepsPerFrame = double(steps) / frames;
    r.fastStepsPerFrame = double(fast) / frames;
    r.toiPerFrame = double(toi) / frames;
    r.hash = w.stateHash();
    return r;
}

// ---------------------------------------------------------------------
// Минимальный эксперимент: один снаряд и стенка 10 px.
void tunnelExperiment(Mode mode, double hz, int substeps, const char* label) {
    SceneParams p;
    p.physicsHz = hz;
    p.fastSubsteps = substeps;
    World w;
    w.mode = mode;
    buildScene(kSceneTunnel, w, p);
    const int proj = w.dynamicIds[0];
    const Body& wall = w.bodies[w.staticIds[0]];
    const double wallRight = wall.pos.x + wall.half.x;

    // Первый пролёт: до первого перезапуска снаряда (~0.6 c).
    bool tunneled = false, bounced = false;
    double minGap = 1e9; // минимальное расстояние от поверхности снаряда до стенки в дискретные моменты
    World probe = w;
    for (int i = 0; i < int(0.55 * hz); ++i) {
        probe.step(1.0 / hz);
        const Body& b = probe.bodies[proj];
        if (b.age == 0.0) break;
        if (b.vel.x < 0) bounced = true;
        if (b.pos.x - b.radius > wallRight && b.vel.x > 0) tunneled = true;
        const Manifold m = collide(b, probe.bodies[probe.staticIds[0]], 1e9);
        minGap = std::min(minGap, -m.penetration);
    }
    // Время: 60000 кадров (1000 с модельного времени), снаряд перезапускается.
    // Одиночный шаг длится доли микросекунды, поэтому усредняем по многим кадрам.
    const RunResult r = run(w, hz, 60000);
    std::printf("  %-26s step=%6.2f px  %-11s  ms/step=%.5f  steps/frame=%2.0f  ms/frame=%.5f\n",
                label, p.projectileSpeed / hz,
                tunneled ? "TUNNEL" : (bounced ? "BOUNCE" : "?"),
                r.msPerStep, r.stepsPerFrame * (mode == Mode::MultiRate ? substeps : 1),
                r.msPerFrame);
}

// Эксперимент с двумя дорожками (multi-rate).
void laneExperiment() {
    SceneParams p;
    World w;
    w.mode = Mode::MultiRate;
    buildScene(kSceneExperiment, w, p);
    bool tun[2] = {false, false}, bounce[2] = {false, false};
    for (int i = 0; i < 33; ++i) {
        w.step(1.0 / 60.0);
        for (int k = 0; k < 2; ++k) {
            const Body& b = w.bodies[w.dynamicIds[k]];
            const Body& wall = w.bodies[w.staticIds[k]];
            if (b.vel.x < 0) bounce[k] = true;
            if (b.pos.x - b.radius > wall.pos.x + wall.half.x && b.vel.x > 0) tun[k] = true;
        }
    }
    std::printf("  lane 60 Hz : %s\n  lane 240 Hz: %s\n",
                tun[0] ? "TUNNEL" : (bounce[0] ? "BOUNCE" : "?"),
                tun[1] ? "TUNNEL" : (bounce[1] ? "BOUNCE" : "?"));
}

// ---------------------------------------------------------------------
struct ModeCase { const char* name; Mode mode; double hz; int sub; };

void arena(const ModeCase& mc, int frames) {
    SceneParams p;
    p.physicsHz = mc.hz;
    p.fastSubsteps = mc.sub;
    World w1, w2;
    w1.mode = w2.mode = mc.mode;
    buildScene(kSceneArena, w1, p);
    buildScene(kSceneArena, w2, p);
    RunResult r1 = run(w1, mc.hz, frames);
    RunResult r2 = run(w2, mc.hz, frames);
    // Время — минимум из трёх прогонов (меньше влияние фоновых процессов ОС).
    World w3;
    w3.mode = mc.mode;
    buildScene(kSceneArena, w3, p);
    const RunResult r3 = run(w3, mc.hz, frames);
    const double best = std::min({r1.msPerFrame, r2.msPerFrame, r3.msPerFrame});
    std::printf("  %-24s ms/frame=%.4f  steps(normal/fast)/frame=%.0f/%.0f  TOI/frame=%.2f (max %d, cap %d)"
                "  escaped=%d  deterministic=%s\n",
                mc.name, best, r1.stepsPerFrame,
                mc.mode == Mode::MultiRate ? r1.fastStepsPerFrame : r1.stepsPerFrame,
                r1.toiPerFrame, r1.toiMaxPerStep, r1.toiCapHits, w1.escaped,
                r1.hash == r2.hash ? "yes" : "NO");
}

void stack(const ModeCase& mc, int frames) {
    SceneParams p;
    p.physicsHz = mc.hz;
    World w1, w2;
    w1.mode = w2.mode = mc.mode;
    buildScene(kSceneStack, w1, p);
    buildScene(kSceneStack, w2, p);
    const int towerTop = w1.dynamicIds[7];
    const Vec2 towerTop0 = w1.bodies[towerTop].pos;

    // RMS-скорость и макс. проникновение — за последние 2 секунды.
    double sumV2 = 0, maxV = 0, maxPen = 0;
    long samples = 0;
    const int measureFrom = frames - 120;
    int frame = 0, stepInFrame = 0;
    const int spf = int(std::lround(mc.hz / 60.0));
    auto probe = [&](World& w) {
        if (++stepInFrame == spf) { stepInFrame = 0; ++frame; }
        if (frame < measureFrom) return;
        for (int i : w.dynamicIds) {
            const double v2 = w.bodies[i].vel.lengthSq();
            sumV2 += v2;
            maxV = std::max(maxV, std::sqrt(v2));
            ++samples;
        }
        for (size_t i = 0; i < w.bodies.size(); ++i)
            for (size_t j = i + 1; j < w.bodies.size(); ++j) {
                if (w.bodies[i].isStatic() && w.bodies[j].isStatic()) continue;
                const Manifold m = collide(w.bodies[i], w.bodies[j], 0.0);
                if (m.hit) maxPen = std::max(maxPen, m.penetration);
            }
    };
    RunResult r1 = run(w1, mc.hz, frames, probe);
    RunResult r2 = run(w2, mc.hz, frames);
    World w3;
    w3.mode = mc.mode;
    buildScene(kSceneStack, w3, p);
    const RunResult r3 = run(w3, mc.hz, frames);
    const double best = std::min(r2.msPerFrame, r3.msPerFrame); // r1 включает время замеров
    const Vec2 d = w1.bodies[towerTop].pos - towerTop0;
    std::printf("  %-24s ms/frame=%.4f  RMS v=%.3f px/s  max v=%.2f px/s  max pen=%.3f px"
                "  tower top shift=(%.2f, %.2f)  TOI/frame=%.2f cap=%d  deterministic=%s\n",
                mc.name, best, std::sqrt(sumV2 / double(std::max(1L, samples))), maxV, maxPen,
                d.x, d.y, r1.toiPerFrame, r1.toiCapHits, r1.hash == r2.hash ? "yes" : "NO");
}

int main() {
    std::printf("=== 1. Minimal experiment: projectile %.0f px/s, wall %.0f px, r=%.0f px (d+2r = %.0f px)\n",
                cfg::kProjectileSpeed, cfg::kWallThickness, cfg::kProjectileRadius,
                cfg::kWallThickness + 2 * cfg::kProjectileRadius);
    tunnelExperiment(Mode::Discrete, 60, 1, "Discrete 60 Hz");
    tunnelExperiment(Mode::Discrete, 120, 1, "Discrete 120 Hz");
    tunnelExperiment(Mode::Discrete, 240, 1, "Discrete 240 Hz");
    tunnelExperiment(Mode::CCD, 60, 1, "CCD 60 Hz");
    tunnelExperiment(Mode::MultiRate, 60, 4, "Multi-rate 60 Hz x4");

    std::printf("\n=== 2. Two lanes, Multi-rate (same projectile, 60 vs 240 Hz)\n");
    laneExperiment();

    const ModeCase arenaCases[] = {
        {"Discrete 60 Hz", Mode::Discrete, 60, 1},
        {"Discrete 240 Hz (all)", Mode::Discrete, 240, 1},
        {"CCD 60 Hz", Mode::CCD, 60, 1},
        {"Multi-rate 60/60", Mode::MultiRate, 60, 1},
        {"Multi-rate 60/240", Mode::MultiRate, 60, 4},
    };
    std::printf("\n=== 3. Arena (6 static, 50 normal, 8 fast), 30 s of simulation\n");
    for (const auto& c : arenaCases) arena(c, 1800);

    const ModeCase stackCases[] = {
        {"Discrete 60 Hz", Mode::Discrete, 60, 4},
        {"CCD 60 Hz", Mode::CCD, 60, 4},
        {"Multi-rate 60/240", Mode::MultiRate, 60, 4},
    };
    std::printf("\n=== 4. Stack (8-box tower, 15-box pyramid, 40 balls in a cup), 10 s\n");
    for (const auto& c : stackCases) stack(c, 600);
    return 0;
}
