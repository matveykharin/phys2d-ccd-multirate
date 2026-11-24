// Демо-приложение: 2D rigid body физика, CCD и multi-rate симуляция.
// Главный цикл с фиксированным шагом физики (аккумулятор времени).
#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <string>
#include "Render.h"
#include "core/Scenes.h"

namespace {

std::string fmt(const char* f, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof(buf), f, ap);
    va_end(ap);
    return buf;
}

// Скользящая статистика для HUD (обновляется 2 раза в секунду).
struct HudStats {
    double physMs = 0, simTime = 0, normalSteps = 0, fastSteps = 0, toi = 0, contacts = 0, steps = 0;
    double shownMsPerFrame = 0, shownMsPerStep = 0, shownNormal = 0, shownFast = 0, shownToi = 0;
    double shownContacts = 0;
    double sinceUpdate = 0;

    void add(double ms, double dt, const StepStats& s) {
        physMs += ms;
        simTime += dt;
        normalSteps += s.normalSteps;
        fastSteps += s.fastSteps;
        toi += s.toiEvents;
        contacts += s.contacts;
        steps += 1;
    }
    void update(double realDt) {
        sinceUpdate += realDt;
        if (sinceUpdate < 0.5 || simTime <= 0) return;
        const double frames = simTime * 60.0; // кадров по 1/60 с модельного времени
        shownMsPerFrame = physMs / frames;
        shownMsPerStep = physMs / steps;
        shownNormal = normalSteps / frames;
        shownFast = fastSteps / frames;
        shownToi = toi / frames;
        shownContacts = contacts / steps;
        physMs = simTime = normalSteps = fastSteps = toi = contacts = steps = 0;
        sinceUpdate = 0;
    }
};

struct App {
    SDL_Window* window = nullptr;
    SDL_Renderer* sdl = nullptr;
    World world;
    SceneParams params;
    int scene = kSceneTunnel;
    bool paused = false;
    bool stepOnce = false;
    DrawOptions draw;
    HudStats hud;
    int shotCounter = 0;

    void loadScene(int id, bool setPreferredMode) {
        scene = id;
        if (setPreferredMode) world.mode = scenePreferredMode(id);
        buildScene(id, world, params);
        hud = HudStats{};
    }

    // Один шаг физики с замером времени.
    void physicsStep() {
        const double dt = 1.0 / params.physicsHz;
        const auto t0 = std::chrono::steady_clock::now();
        world.step(dt);
        const double ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        hud.add(ms, dt, world.stats);
    }

    bool screenshot(const std::string& path) {
        int w = 0, h = 0;
        SDL_GetRendererOutputSize(sdl, &w, &h);
        SDL_Surface* s = SDL_CreateRGBSurfaceWithFormat(0, w, h, 32, SDL_PIXELFORMAT_ARGB8888);
        if (!s) return false;
        SDL_RenderReadPixels(sdl, nullptr, SDL_PIXELFORMAT_ARGB8888, s->pixels, s->pitch);
        const bool ok = SDL_SaveBMP(s, path.c_str()) == 0;
        SDL_FreeSurface(s);
        if (ok) std::printf("Saved %s\n", path.c_str());
        return ok;
    }

    void renderFrame(double alpha) {
        Render r(sdl);
        r.clear(0x15181EFF);
        draw.alpha = alpha;
        r.drawWorld(world, draw);
        drawHud(r);
    }

    void drawHud(Render& r) {
        const uint32_t white = 0xEDEDEDFF, gray = 0xA8B0BAFF, yellow = 0xFFE14DFF;
        const int x = 30, lh = 16;
        int y = 28;
        r.textPanel(x - 8, y - 6, 760, scene == kSceneArena ? 6 * lh + 10 : 5 * lh + 10);

        r.text(x, y, sceneTitle(scene), yellow);
        y += lh;
        std::string rates = fmt("Mode: %s   Physics: %.0f Hz (dt = %.2f ms)", modeName(world.mode),
                                params.physicsHz, 1000.0 / params.physicsHz);
        if (world.mode == Mode::MultiRate)
            rates += fmt("   Fast: %.0f Hz (%d substeps)", params.physicsHz * world.fastSubsteps,
                         world.fastSubsteps);
        r.text(x, y, rates, white);
        y += lh;
        r.text(x, y,
               fmt("Physics time: %.4f ms/frame  (%.4f ms/step)   steps per frame: normal %.0f, fast %.0f",
                   hud.shownMsPerFrame, hud.shownMsPerStep, hud.shownNormal, hud.shownFast),
               white);
        y += lh;
        const double step = params.projectileSpeed / params.physicsHz;
        std::string pathLine =
            fmt("e = %.2f   projectile %.0f px/s   path per step v*dt = %.1f px", world.restitution,
                params.projectileSpeed, step);
        if (world.mode == Mode::MultiRate)
            pathLine += fmt(" (fast substep %.1f px)", step / world.fastSubsteps);
        pathLine += fmt("  vs  d+2r = %.0f px", cfg::kWallThickness + 2 * cfg::kProjectileRadius);
        r.text(x, y, pathLine, white);
        y += lh;
        r.text(x, y,
               fmt("CCD events/frame: %.2f   contacts/step: %.1f   t = %.2f s   hash %08llx",
                   hud.shownToi, hud.shownContacts, world.time,
                   static_cast<unsigned long long>(world.stateHash() & 0xFFFFFFFFull)),
               gray);
        y += lh;
        if (scene == kSceneArena)
            r.text(x, y, fmt("Projectiles escaped through walls: %d", world.escaped), 0xFF7A6EFF);

        // Подсказка по клавишам.
        const int H = cfg::kWindowHeight;
        r.textPanel(0, H - 22, cfg::kWindowWidth, 22);
        r.text(8, H - 18,
               "[1-4] scene [M] mode [H] Hz [S] substeps [<-/->] e [Up/Dn] speed [F] fire [R] reset "
               "[P] pause [N] step [T] trails [I] interp [F12] shot",
               gray);
        if (paused) r.text(cfg::kWindowWidth - 90, 30, "PAUSED", 0xFF7A6EFF, 2);
    }

    void handleKey(SDL_Keycode k) {
        switch (k) {
            case SDLK_1: case SDLK_2: case SDLK_3: case SDLK_4:
                loadScene(int(k - SDLK_1), true);
                break;
            case SDLK_r: loadScene(scene, false); break;
            case SDLK_m:
                world.mode = Mode((int(world.mode) + 1) % 3);
                hud = HudStats{};
                // Сцена 1 перестраивается: стартовая точка зависит от шага снаряда.
                if (scene == kSceneTunnel) loadScene(scene, false);
                break;
            case SDLK_h: // 60 -> 120 -> 240 -> 60 Гц; сцена перестраивается
                params.physicsHz = params.physicsHz >= 240 ? 60 : params.physicsHz * 2;
                loadScene(scene, false);
                break;
            case SDLK_s: { // подшаги быстрых тел: 1 -> 2 -> 4 -> 8 -> 1
                int s = world.fastSubsteps * 2;
                if (s > 8) s = 1;
                world.fastSubsteps = params.fastSubsteps = s;
                hud = HudStats{};
                if (scene == kSceneTunnel) loadScene(scene, false);
                break;
            }
            case SDLK_LEFT: case SDLK_RIGHT: {
                double e = world.restitution + (k == SDLK_RIGHT ? 0.1 : -0.1);
                e = e < 0 ? 0 : (e > 1 ? 1 : e);
                world.restitution = params.restitution = std::round(e * 10) / 10;
                break;
            }
            case SDLK_UP: case SDLK_DOWN: {
                double v = params.projectileSpeed + (k == SDLK_UP ? 250 : -250);
                params.projectileSpeed = v < 250 ? 250 : v;
                loadScene(scene, false);
                break;
            }
            case SDLK_f: fireProjectiles(scene, world, params); break;
            case SDLK_p: paused = !paused; break;
            case SDLK_n: if (paused) stepOnce = true; break;
            case SDLK_t: draw.trails = !draw.trails; break;
            case SDLK_i: draw.interpolate = !draw.interpolate; break;
            case SDLK_F12: screenshot(fmt("screenshot_%03d.bmp", shotCounter++)); break;
            default: break;
        }
    }
};

Mode parseMode(const char* s) {
    if (!std::strcmp(s, "ccd")) return Mode::CCD;
    if (!std::strcmp(s, "multirate")) return Mode::MultiRate;
    return Mode::Discrete;
}

// Режим автоматического захвата кадров (для отчёта):
//   phys_demo --scene 1 --mode discrete --frames 60 --shots 10,20 --out shots/tunnel
// Каждая итерация = 1/60 с модельного времени, отрисовка без интерполяции.
struct Capture {
    bool enabled = false;
    int frames = 0;
    int dumpEvery = 0;
    std::set<int> shots;
    std::string out = "frame";
};

} // namespace

int main(int argc, char** argv) {
    App app;
    Capture cap;
    int startScene = kSceneTunnel;
    const char* forcedMode = nullptr;
    bool fire = false;
    int fireAt = -1;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--scene") startScene = std::atoi(next()) - 1;
        else if (a == "--mode") forcedMode = next();
        else if (a == "--hz") app.params.physicsHz = std::atof(next());
        else if (a == "--substeps") app.params.fastSubsteps = std::atoi(next());
        else if (a == "--e") app.params.restitution = std::atof(next());
        else if (a == "--speed") app.params.projectileSpeed = std::atof(next());
        else if (a == "--frames") { cap.enabled = true; cap.frames = std::atoi(next()); }
        else if (a == "--dump-every") cap.dumpEvery = std::atoi(next());
        else if (a == "--out") cap.out = next();
        else if (a == "--no-trails") app.draw.trails = false;
        else if (a == "--fire-at") { fire = true; fireAt = std::atoi(next()); }
        else if (a == "--shots") {
            std::string list = next();
            size_t pos = 0;
            while (pos < list.size()) {
                const size_t c = list.find(',', pos);
                cap.shots.insert(std::atoi(list.substr(pos, c - pos).c_str()));
                if (c == std::string::npos) break;
                pos = c + 1;
            }
        }
    }
    if (startScene < 0 || startScene >= kSceneCount) startScene = 0;

    SDL_SetMainReady();
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
    app.window = SDL_CreateWindow("2D rigid body: CCD and multi-rate", SDL_WINDOWPOS_CENTERED,
                                  SDL_WINDOWPOS_CENTERED, cfg::kWindowWidth, cfg::kWindowHeight,
                                  SDL_WINDOW_RESIZABLE | (cap.enabled ? SDL_WINDOW_HIDDEN : 0));
    if (!app.window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return 1;
    }
    Uint32 flags = cap.enabled ? SDL_RENDERER_SOFTWARE : (SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    app.sdl = SDL_CreateRenderer(app.window, -1, flags);
    if (!app.sdl) app.sdl = SDL_CreateRenderer(app.window, -1, SDL_RENDERER_SOFTWARE);
    if (!app.sdl) {
        std::fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_SetRenderDrawBlendMode(app.sdl, SDL_BLENDMODE_BLEND);
    SDL_RenderSetLogicalSize(app.sdl, cfg::kWindowWidth, cfg::kWindowHeight);

    app.world.mode = scenePreferredMode(startScene);
    if (forcedMode) app.world.mode = parseMode(forcedMode);
    app.loadScene(startScene, false);

    // ---------------- Режим захвата кадров ----------------
    if (cap.enabled) {
        const int stepsPerFrame = int(app.params.physicsHz / 60.0 + 0.5);
        for (int f = 0; f <= cap.frames; ++f) {
            if (fire && f == fireAt) fireProjectiles(app.scene, app.world, app.params);
            if (f > 0)
                for (int s = 0; s < stepsPerFrame; ++s) app.physicsStep();
            app.hud.update(1.0); // обновлять показания каждый кадр
            app.renderFrame(1.0);
            if (cap.shots.count(f) || (cap.dumpEvery > 0 && f % cap.dumpEvery == 0))
                app.screenshot(fmt("%s_%04d.bmp", cap.out.c_str(), f));
        }
        SDL_Quit();
        return 0;
    }

    // ---------------- Интерактивный режим ----------------
    // Фиксированный шаг: реальное время копится в аккумуляторе и «тратится»
    // шагами физики ровно по dt. Остаток (alpha) используется для интерполяции.
    const double freq = double(SDL_GetPerformanceFrequency());
    Uint64 prev = SDL_GetPerformanceCounter();
    double accumulator = 0.0;
    bool running = true;
    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            if (e.type == SDL_KEYDOWN) {
                if (e.key.keysym.sym == SDLK_ESCAPE) running = false;
                else app.handleKey(e.key.keysym.sym);
            }
        }
        const Uint64 now = SDL_GetPerformanceCounter();
        double frameTime = double(now - prev) / freq;
        prev = now;
        if (frameTime > 0.25) frameTime = 0.25; // защита от «спирали смерти»

        const double dt = 1.0 / app.params.physicsHz;
        if (!app.paused) {
            accumulator += frameTime;
            while (accumulator >= dt) {
                app.physicsStep();
                accumulator -= dt;
            }
        } else {
            accumulator = 0.0;
            if (app.stepOnce) {
                app.physicsStep();
                app.stepOnce = false;
            }
        }
        app.hud.update(frameTime);
        app.renderFrame(app.paused ? 1.0 : accumulator / dt);
        SDL_RenderPresent(app.sdl);
    }

    SDL_DestroyRenderer(app.sdl);
    SDL_DestroyWindow(app.window);
    SDL_Quit();
    return 0;
}
