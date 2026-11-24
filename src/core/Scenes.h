#pragma once
// Демонстрационные сцены.
#include "World.h"

struct SceneParams {
    double physicsHz       = cfg::kPhysicsHz;
    double projectileSpeed = cfg::kProjectileSpeed;
    double restitution     = cfg::kRestitution;
    int    fastSubsteps    = cfg::kFastSubsteps;
};

enum SceneId {
    kSceneTunnel = 0,     // Часть 1: снаряд и тонкая стенка (Discrete vs CCD)
    kSceneExperiment = 1, // Часть 2: один и тот же снаряд на 60 и 240 Гц
    kSceneArena = 2,      // Часть 2: 6 неподвижных, 50 обычных, 8 быстрых тел
    kSceneStack = 3,      // Сравнение устойчивости: башня, пирамида, «стакан» с шарами
    kSceneCount = 4
};

const char* sceneTitle(int id);
Mode scenePreferredMode(int id);

// Построить сцену заново (мир очищается). Режим мира не меняется.
void buildScene(int id, World& w, const SceneParams& p);

// Клавиша F: перезапустить снаряды / выстрелить снарядом в стопку.
void fireProjectiles(int id, World& w, const SceneParams& p);
