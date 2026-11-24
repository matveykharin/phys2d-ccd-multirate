#pragma once
// Физический мир: хранение тел и три режима шага симуляции.
#include <cstdint>
#include <string>
#include <vector>
#include "Body.h"
#include "Config.h"

enum class Mode {
    Discrete,  // дискретная проверка столкновений в конце шага
    CCD,       // непрерывное обнаружение (поиск момента столкновения t_c)
    MultiRate  // обычные тела 60 Гц, быстрые — N подшагов (240 Гц)
};

const char* modeName(Mode m);

// Статистика последнего вызова World::step().
struct StepStats {
    int contacts = 0;       // контактов, найденных дискретной проверкой
    int toiEvents = 0;      // CCD: столкновений, обработанных внутри шага
    int toiCapHits = 0;     // CCD: сколько раз упёрлись в kMaxToiEvents
    int normalSteps = 0;    // шагов обычных тел
    int fastSteps = 0;      // шагов быстрых тел (в multi-rate = числу подшагов)
};

struct Rect { double x0, y0, x1, y1; };

// Отметка точки, где CCD нашёл контакт (для отрисовки).
struct ContactMark { Vec2 p; double age; };

class World {
public:
    std::vector<Body> bodies;

    // Тела хранятся в одном массиве, а группы — отдельными наборами индексов:
    // неподвижные, обычные (60 Гц) и быстрые (240 Гц).
    std::vector<int> staticIds;
    std::vector<int> normalIds;
    std::vector<int> fastIds;
    std::vector<int> dynamicIds; // normal + fast (для режимов Discrete и CCD)

    Mode   mode = Mode::Discrete;
    Vec2   gravity;
    double restitution = cfg::kRestitution;
    int    fastSubsteps = cfg::kFastSubsteps;
    Rect   bounds{-100, -100, cfg::kWindowWidth + 100.0, cfg::kWindowHeight + 100.0};

    double   time = 0.0;
    uint64_t stepCount = 0;
    int      escaped = 0;   // сколько снарядов вылетело за пределы сцены
    int      trailLength = 48;
    StepStats stats;
    std::vector<ContactMark> marks;

    int  addBody(const Body& b);
    void clear();

    // Один шаг физики длительностью dt в текущем режиме.
    void step(double dt);

    // Хэш положений и скоростей — для проверки детерминированности.
    uint64_t stateHash() const;

    // Сбросить отдельное тело в точку появления.
    void respawn(Body& b);

private:
    struct Contact {
        int a, b;
        Vec2 n;            // нормаль от a к b
        double pen;
        double target;     // целевая нормальная скорость после удара
        double accum;      // накопленный импульс (для ограничения >= 0)
        double invMassSum;
    };

    void stepDiscrete(double dt, const std::vector<int>& movers, bool collideWithFast);
    void stepCCD(double dt);
    void stepMultiRate(double dt);

    // Найти пары, где хотя бы одно тело из movers. margin — см. collide().
    void gatherContacts(const std::vector<int>& movers, bool collideWithFast,
                        double margin, std::vector<Contact>& out);
    void solveVelocities(std::vector<Contact>& cs);
    void correctPositions(const std::vector<int>& movers, bool collideWithFast);
    void pushTrail(const std::vector<int>& ids);
    void postStep(double dt);

    std::vector<char> isMover_; // временная разметка
    std::vector<Contact> contacts_;
};
