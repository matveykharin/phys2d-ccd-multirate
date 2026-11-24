#include "World.h"
#include <algorithm>
#include <cstring>
#include "Collision.h"

const char* modeName(Mode m) {
    switch (m) {
        case Mode::Discrete:  return "Discrete";
        case Mode::CCD:       return "CCD";
        case Mode::MultiRate: return "Multi-rate";
    }
    return "?";
}

int World::addBody(const Body& b) {
    const int id = static_cast<int>(bodies.size());
    bodies.push_back(b);
    switch (b.group) {
        case Group::Static: staticIds.push_back(id); break;
        case Group::Normal: normalIds.push_back(id); dynamicIds.push_back(id); break;
        case Group::Fast:   fastIds.push_back(id);   dynamicIds.push_back(id); break;
    }
    return id;
}

void World::clear() {
    bodies.clear();
    staticIds.clear();
    normalIds.clear();
    fastIds.clear();
    dynamicIds.clear();
    marks.clear();
    time = 0.0;
    stepCount = 0;
    escaped = 0;
    stats = {};
}

void World::respawn(Body& b) {
    b.pos = b.prevPos = b.spawnPos;
    b.vel = b.spawnVel;
    b.age = 0.0;
    b.trail.clear();
}

// =====================================================================
//  Главная точка входа
// =====================================================================
void World::step(double dt) {
    stats = {};
    for (Body& b : bodies) b.prevPos = b.pos;

    switch (mode) {
        case Mode::Discrete:
            // Все тела — одним шагом dt, проверка пересечений в конце шага.
            stepDiscrete(dt, dynamicIds, true);
            stats.normalSteps = stats.fastSteps = 1;
            break;
        case Mode::CCD:
            stepCCD(dt);
            stats.normalSteps = stats.fastSteps = 1;
            break;
        case Mode::MultiRate:
            stepMultiRate(dt);
            break;
    }

    postStep(dt);
    time += dt;
    ++stepCount;
}

// =====================================================================
//  Дискретный шаг (полунеявный Эйлер + разрешение пересечений)
// =====================================================================
void World::stepDiscrete(double dt, const std::vector<int>& movers, bool collideWithFast) {
    // 1. Интегрирование: сначала скорость, затем положение.
    for (int i : movers) {
        Body& b = bodies[i];
        b.vel += gravity * dt;
        b.pos += b.vel * dt;
    }
    // 2. Проверка пересечений ТОЛЬКО в конечных положениях —
    //    отсюда и туннелирование: что было между началом и концом шага, не видно.
    gatherContacts(movers, collideWithFast, 0.0, contacts_);
    stats.contacts += static_cast<int>(contacts_.size());
    // 3. Импульсы (скорости) и выталкивание (положения).
    solveVelocities(contacts_);
    correctPositions(movers, collideWithFast);
    pushTrail(movers);
}

// =====================================================================
//  CCD-шаг
// =====================================================================
void World::stepCCD(double dt) {
    const std::vector<int>& dyn = dynamicIds;

    // 1. Внешние силы меняют скорости в начале шага.
    for (int i : dyn) bodies[i].vel += gravity * dt;

    // 2. Тела, которые уже касаются (лежат друг на друге), обрабатываются
    //    обычным решателем контактов: убираем скорость сближения.
    gatherContacts(dyn, true, cfg::kContactMargin, contacts_);
    stats.contacts = static_cast<int>(contacts_.size());
    solveVelocities(contacts_);

    // 3. Цикл по событиям столкновения внутри шага.
    auto advance = [&](double t) {
        for (int i : dyn) bodies[i].pos += bodies[i].vel * t;
    };

    std::vector<Contact> eventContacts;
    double remaining = dt;
    while (remaining > 0.0) {
        // 3a. Ищем самое раннее столкновение t_c среди всех пар,
        //     которые сейчас НЕ касаются.
        double best = remaining;
        int ba = -1, bb = -1;
        Vec2 bn;
        isMover_.assign(bodies.size(), 0);
        for (int i : dyn) isMover_[i] = 1;
        for (int i : dyn) {
            for (int j = 0; j < static_cast<int>(bodies.size()); ++j) {
                if (j == i || (isMover_[j] && j < i)) continue;
                const Body& A = bodies[i];
                const Body& B = bodies[j];
                if (collide(A, B, cfg::kContactMargin).hit) continue; // уже в контакте
                const Toi toi = timeOfImpact(A, B, best);
                if (toi.hit && toi.t < best) {
                    best = toi.t;
                    ba = i; bb = j; bn = toi.normal;
                }
            }
        }

        if (ba < 0) { advance(remaining); break; }          // столкновений нет
        if (stats.toiEvents >= cfg::kMaxToiEvents) {        // защита от зацикливания
            ++stats.toiCapHits;
            advance(remaining);
            break;
        }

        // 3b. Синхронно переносим все тела к моменту t_c.
        advance(best);
        remaining -= best;
        ++stats.toiEvents;

        // Отметка точки контакта для отрисовки.
        const Body& A = bodies[ba];
        const Body& B = bodies[bb];
        Vec2 cp = (A.pos + B.pos) * 0.5;
        if (A.shape == Shape::Circle) cp = A.pos + bn * A.radius;
        else if (B.shape == Shape::Circle) cp = B.pos - bn * B.radius;
        marks.push_back({cp, 0.0});

        // 3c. Реакция: импульс с коэффициентом восстановления. Решаем все
        //     касающиеся пары разом — так корректно обрабатываются
        //     одновременные удары (например, в угол).
        gatherContacts(dyn, true, cfg::kContactMargin, eventContacts);
        solveVelocities(eventContacts);
        // 3d. Цикл продолжается для оставшейся части шага (dt - t_c).
    }

    // 4. Подчищаем накопившиеся мелкие перекрытия (стопки, ошибки округления).
    correctPositions(dyn, true);
    pushTrail(dyn);
}

// =====================================================================
//  Multi-rate шаг
// =====================================================================
void World::stepMultiRate(double dt) {
    // 1. Обычные тела: один шаг dt (1/60 с). Столкновения обычных
    //    с обычными и с неподвижными.
    stepDiscrete(dt, normalIds, false);
    stats.normalSteps = 1;

    // 2. Быстрые тела: N подшагов dt/N (1/240 с при N = 4). Проверяются
    //    против ВСЕХ тел, при этом обычные тела уже стоят в конце кадра.
    const double sub = dt / fastSubsteps;
    for (int k = 0; k < fastSubsteps; ++k) stepDiscrete(sub, fastIds, true);
    stats.fastSteps = fastSubsteps;
}

// =====================================================================
//  Вспомогательные функции
// =====================================================================
void World::gatherContacts(const std::vector<int>& movers, bool collideWithFast,
                           double margin, std::vector<Contact>& out) {
    out.clear();
    isMover_.assign(bodies.size(), 0);
    for (int i : movers) isMover_[i] = 1;

    for (int i : movers) {
        for (int j = 0; j < static_cast<int>(bodies.size()); ++j) {
            if (j == i) continue;
            if (isMover_[j] && j < i) continue;  // каждую пару — ровно один раз
            const Body& B = bodies[j];
            if (!collideWithFast && B.group == Group::Fast) continue;
            const Manifold m = collide(bodies[i], B, margin);
            if (!m.hit) continue;
            Contact c{};
            c.a = i;
            c.b = j;
            c.n = m.normal;
            c.pen = m.penetration;
            out.push_back(c);
        }
    }
}

// Последовательные импульсы (sequential impulses) с накоплением импульса.
// Для каждого контакта требуем, чтобы нормальная скорость после удара
// была >= target, где target = e * |v_n| (отскок) или 0 (покой).
void World::solveVelocities(std::vector<Contact>& cs) {
    for (Contact& c : cs) {
        const Body& A = bodies[c.a];
        const Body& B = bodies[c.b];
        c.invMassSum = A.invMass + B.invMass;
        const double vn = dot(B.vel - A.vel, c.n); // < 0 — сближаются
        c.target = (vn < -cfg::kRestingSpeed) ? -restitution * vn : 0.0;
        c.accum = 0.0;
    }
    for (int it = 0; it < cfg::kSolverIterations; ++it) {
        for (Contact& c : cs) {
            if (c.invMassSum <= 0.0) continue;
            Body& A = bodies[c.a];
            Body& B = bodies[c.b];
            const double vn = dot(B.vel - A.vel, c.n);
            double lambda = (c.target - vn) / c.invMassSum;
            const double newAccum = std::max(c.accum + lambda, 0.0); // только «толкать»
            lambda = newAccum - c.accum;
            c.accum = newAccum;
            const Vec2 J = c.n * lambda;
            A.vel -= J * A.invMass;
            B.vel += J * B.invMass;
        }
    }
}

// Позиционная коррекция: выталкиваем перекрывающиеся тела пропорционально
// обратным массам, оставляя небольшой допуск kSlop (иначе стопки дрожат).
void World::correctPositions(const std::vector<int>& movers, bool collideWithFast) {
    std::vector<Contact> cs;
    for (int it = 0; it < cfg::kPositionIterations; ++it) {
        gatherContacts(movers, collideWithFast, 0.0, cs);
        bool any = false;
        for (const Contact& c : cs) {
            Body& A = bodies[c.a];
            Body& B = bodies[c.b];
            const Manifold m = collide(A, B, 0.0); // свежие данные
            if (!m.hit || m.penetration <= cfg::kSlop) continue;
            const double ims = A.invMass + B.invMass;
            if (ims <= 0.0) continue;
            const double k = (m.penetration - cfg::kSlop) * cfg::kCorrectionPercent / ims;
            A.pos -= m.normal * (k * A.invMass);
            B.pos += m.normal * (k * B.invMass);
            any = true;
        }
        if (!any) break;
    }
}

void World::pushTrail(const std::vector<int>& ids) {
    for (int i : ids) {
        Body& b = bodies[i];
        if (!b.trackTrail) continue;
        b.trail.push_back(b.pos);
        while (static_cast<int>(b.trail.size()) > trailLength) b.trail.pop_front();
    }
}

void World::postStep(double dt) {
    for (int i : dynamicIds) {
        Body& b = bodies[i];
        if (!b.respawns) continue;
        b.age += dt;
        const bool out = b.pos.x < bounds.x0 || b.pos.x > bounds.x1 ||
                         b.pos.y < bounds.y0 || b.pos.y > bounds.y1;
        if (out) {
            ++escaped;
            respawn(b);
        } else if (b.lifetime > 0.0 && b.age >= b.lifetime) {
            respawn(b);
        }
    }
    for (ContactMark& m : marks) m.age += dt;
    marks.erase(std::remove_if(marks.begin(), marks.end(),
                               [](const ContactMark& m) { return m.age > 0.6; }),
                marks.end());
}

uint64_t World::stateHash() const {
    // FNV-1a по байтам положений и скоростей всех тел.
    uint64_t h = 1469598103934665603ull;
    auto mix = [&h](double v) {
        unsigned char bytes[sizeof(double)];
        std::memcpy(bytes, &v, sizeof(double));
        for (unsigned char c : bytes) { h ^= c; h *= 1099511628211ull; }
    };
    for (const Body& b : bodies) {
        mix(b.pos.x); mix(b.pos.y); mix(b.vel.x); mix(b.vel.y);
    }
    return h;
}
