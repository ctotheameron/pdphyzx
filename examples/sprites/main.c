#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "pd_api.h"
#include "pd_api/pd_api_sys.h"

#define PDPHYZX_IMPLEMENTATION
#include "../../dist/pdphyzx.h"

#include "circle.h"
#include "ground.h"

extern PlaydateAPI *pd;

PdPhyzxAPI *px = NULL;
PxWorld *world = NULL;
PxVec2 g;

static bool showDebugInfo = false;

#define MAX_CIRCLES 50
static Circle circles[MAX_CIRCLES];
static int activeCircleCount = 0;

Ground ground;

static void spawnCircle(void) {
  if (activeCircleCount >= MAX_CIRCLES) {
    return;
  }

  int index = -1;
  for (int i = 0; i < MAX_CIRCLES; i++) {
    if (!circles[i].body && !circles[i].sprite) {
      index = i;
      break;
    }
  }

  if (index == -1) {
    return;
  }

  float x = 200.0f + (rand() % 100 - 50); // Random position near center
  float y = 100.0f;                       // Near the top of the screen

  circles[index] = newCircle(world, pxVec2(x, y));
  activeCircleCount++;
}

static void cleanCircles(void) {
  if (activeCircleCount == 0) {
    return;
  }

  for (uint8_t i = 0; i < MAX_CIRCLES; i++) {
    if (!circles[i].body || !circles[i].sprite) {
      continue;
    }

    PxVec2 pos = circles[i].body->position;

    if (pos.y > 300 || pos.x < -50 || pos.x > 450 || pos.y < -50) {
      freeCircle(&circles[i], world);
      circles[i].body = NULL;
      circles[i].sprite = NULL;
      activeCircleCount--;
    }
  }
}

void handleInput(void) {
  PDButtons current, pressed;
  pd->system->getButtonState(&current, &pressed, NULL);

  if (pressed) {
    // Check for A button press to spawn a circle
    if (current == kButtonA) {
      spawnCircle();
    }

    if (current == kButtonDown) {
      showDebugInfo = !showDebugInfo;
    }
  }

  // Handle crank rotation
  if (!pd->system->isCrankDocked()) {
    float angle = pd->system->getCrankAngle();
    float radians = angle * 0.0174533f;
    px->body->setOrientation(ground.body, radians);
  }
}

static int update(void *userdata) {
  (void)userdata;

  handleInput();

  // Step the physics simulation
  px->world->step(world, g);

  // Draw the screen
  pd->sprite->updateAndDrawSprites();

  if (showDebugInfo) {
    px->world->drawDebug(world);

    // Show debug status
    pd->graphics->drawText("Debug Mode", strlen("Debug Mode"), kASCIIEncoding,
                           5, 5);
  }

  // Display FPS
  pd->system->drawFPS(0, 0);

  // Clean up circles
  cleanCircles();

  return 1;
}

int eventHandler(PlaydateAPI *playdate, PDSystemEvent event, uint32_t arg) {
  (void)arg;

  switch (event) {
  case kEventInit:
    // Configure playdate
    pd = playdate;
    pd->display->setRefreshRate(0); // 50 fps
    pd->system->setUpdateCallback(update, NULL);

    // Setup the physics engine
    g = pxVec2(0, 9.8);
    px = registerPdPhyzx(playdate);
    world = px->world->new(232, 99, 7);

    ground = newGround(world, pxVec2(200, 150));

    break;

  case kEventTerminate:
    cleanCircles();
    px->world->free(world);
    break;

  default:
    break;
  }

  return 0;
}
