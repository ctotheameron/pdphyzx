#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t lastUpdateMs;
  float accumulator;
  float fixedTimestep;
  uint8_t targetFps;
  uint8_t maxStepsPerFrame;
  bool isFirstUpdate;
} PxClock;

PxClock *pxClockInit(PxClock *clock, uint8_t targetFps);

void pxClockBeginFrame(PxClock *clock);

bool pxClockShouldStep(PxClock *clock);

float pxClockGetFixedDeltaTime(PxClock *clock);

void pxClockAdvance(PxClock *clock);

void pxClockSetTargetFPS(PxClock *clock, uint8_t targetFps);
