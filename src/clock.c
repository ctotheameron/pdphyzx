#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "px_math.h"

#include "clock.h"

PxClock *pxClockInit(PxClock *clock, uint8_t targetFps) {
  if (!clock) {
    return NULL;
  }

  clock->maxStepsPerFrame = 3;
  clock->targetFps = pxClamp(targetFps <= 0 ? 50 : targetFps, 1, 50);
  clock->fixedTimestep = pxFastRcp(clock->targetFps);
  clock->lastUpdateMs = 0;
  clock->accumulator = 0;
  clock->isFirstUpdate = true;

  return clock;
}

void pxClockBeginFrame(PxClock *clock) {
  uint32_t currentTime = pd->system->getCurrentTimeMilliseconds();

  // Initialize on first update
  if (clock->isFirstUpdate) {
    clock->lastUpdateMs = currentTime;
    clock->isFirstUpdate = false;
    return;
  }

  // Calculate elapsed time since last frame
  uint32_t elapsed = currentTime - clock->lastUpdateMs;
  float frameTime = pxClamp(pxFastDiv(elapsed, 1000), 0, 0.1);

  // Add to accumulator and update last time
  clock->accumulator += frameTime;
  clock->lastUpdateMs = currentTime;
}

bool pxClockShouldStep(PxClock *clock) {
  return clock->accumulator >= clock->fixedTimestep;
}

float pxClockGetFixedDeltaTime(PxClock *clock) { return clock->fixedTimestep; }

void pxClockAdvance(PxClock *clock) {
  clock->accumulator -= clock->fixedTimestep;
}

void pxClockSetTargetFPS(PxClock *clock, uint8_t targetFps) {
  clock->targetFps = pxClamp(targetFps <= 0 ? 50 : targetFps, 1, 50);
  clock->fixedTimestep = pxFastRcp(clock->targetFps);
  clock->accumulator = 0;
}
