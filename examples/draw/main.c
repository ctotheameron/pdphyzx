#include <stdio.h>
#include <stdlib.h>

#include "pd_api.h"

// pdphyzx.h is an STB-style header-only library, so you need to include it
// and define the implementation macro (only once) before using it.
#define PDPHYZX_IMPLEMENTATION

// This macro adjusts various constants internal in the physics engine to work
// more naturally at the scale you want to work with.
//
// The default is 1 unit = 1 mm.
#define PDPHYZX_UNIT PDPHYZX_UNIT_MM
#include "pdphyzx.h"

PdPhyzxAPI *px = NULL;

#include "includes/circle.h"
#include "includes/ground.h"
#include "includes/object.h"
#include "includes/player.h"

#define TARGET_FPS 50.0f

static PxWorld *world = NULL;
static Objects objects = {0};
static Object *player = NULL;
static bool showDebugInfo = false;

static void spawnCircle(void) {
  float x = 200.0f + (rand() % 100 - 50); // Random position near center
  float y = 100.0f;                       // Near the top of the screen

  Object circle = newCircle(world, pxVec2(x, y));
  objectsAdd(&objects, circle);
}

static void spawnPlayer(void) {
  const float x = 200.0f; // Center of screen
  const float y = 120.0f; // Start above ground

  // Remove the old player if it exists
  if (player) {
    player = NULL;
    objectsRemove(&objects, world, player);
  }

  player = objectsAdd(&objects, newPlayer(world, pxVec2(x, y)));
}

static void spawnGround(void) {
  Object ground = newGround(world, pxVec2(200, 190));
  objectsAdd(&objects, ground);
}

static void initScene(void) {
  objectsRemoveAll(&objects, world);
  spawnGround();
  spawnPlayer();
}

static void handleInput(void) {
  PDButtons current, pressed;
  pd->system->getButtonState(&current, &pressed, NULL);

  // Check for A button press to spawn a circle
  if (pressed & kButtonA) {
    spawnCircle();
  }

  if (pressed & kButtonB) {
    initScene();
  }

  if (pressed & kButtonDown) {
    showDebugInfo = !showDebugInfo;
  }
}

static int update(void *userdata) {
  (void)userdata;

  // Clear the screen
  pd->graphics->clear(kColorWhite);

  handleInput();

  // Step the physics simulation
  px->world->step(world);

  updateAndDrawObjects(&objects, world);

  if (showDebugInfo) {
    px->world->drawDebug(world);
  }

  // Display FPS
  pd->system->drawFPS(0, 0);

  return 1;
}

int eventHandler(PlaydateAPI *playdate, PDSystemEvent event, uint32_t arg) {
  (void)arg;

  switch (event) {
  case kEventInit:
    // Configure playdate
    pd = playdate;
    pd->display->setRefreshRate(TARGET_FPS);
    pd->system->setUpdateCallback(update, NULL);

    // Setup the physics engine
    px = registerPdPhyzx(playdate);

    // Note that you need to provide the target FPS to the physics engine.
    // pdphyx will automatically adjust the time step to match.
    world = px->world->new(TARGET_FPS);
    px->world->setIterations(world, 10);

    // For the sake of example, imagine down simulates objects falling down the
    // incline on a pinball table (~6 degrees) = g·sin(6°) ≈ 0.1045g
    //
    // g = 9.8 * 0.1045 = 1.025m/s^2
    px->world->setGravity(world, 0, 1.025);

    initScene();
    break;

  case kEventTerminate:
    px->world->free(world);
    break;

  default:
    break;
  }

  return 0;
}
