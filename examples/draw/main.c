#include <stdio.h>
#include <stdlib.h>

#include "pd_api.h"

#define PDPHYZX_IMPLEMENTATION
#include "../../dist/pdphyzx.h"

extern PlaydateAPI *pd;
static PdPhyzxAPI *px = NULL;
static PxWorld *world = NULL;
static PxVec2 g;
static bool showDebugInfo = false;

// Structure for player-controlled box
typedef struct {
  PxBody *body;
  bool isGrounded;
} Player;

static Player player = {0};

// Structure for our circle
typedef struct {
  PxBody *body;
  bool active;
} Circle;

#define MAX_CIRCLES 50
static Circle circles[MAX_CIRCLES] = {0};
static int activeCircleCount = 0;

static PxBody *groundBody = NULL;
static int groundWidth = 400;
static int groundHeight = 10;

// Draw all physics objects
static void drawPhysicsObjects(void) {
  // Draw ground
  if (groundBody) {
    PxVec2 pos = groundBody->position;
    pd->graphics->fillRect(pos.x - pxFastDiv(groundWidth, 2),
                           pos.y - pxFastDiv(groundHeight, 2), groundWidth,
                           groundHeight, kColorBlack);
  }

  // Draw circles
  for (int i = 0; i < MAX_CIRCLES; i++) {
    if (circles[i].active && circles[i].body) {
      PxVec2 pos = circles[i].body->position;

      // // Remove circle if it has gone off-screen
      if (pos.y > 300 || pos.x < -50 || pos.x > 450 || pos.y < -50) {
        px->world->freeBody(world, circles[i].body);
        circles[i].active = false;
        circles[i].body = NULL;
        activeCircleCount--;
      } else {
        float radius = circles[i].body->collider.shape.circle.radius;
        // Draw the circle
        pd->graphics->fillEllipse(pos.x - radius, pos.y - radius, radius * 2,
                                  radius * 2, 0.0f, 360.0f, kColorBlack);
      }
    }
  }
}

// Spawn a new circle
static void spawnCircle(void) {
  if (activeCircleCount >= MAX_CIRCLES) {
    return; // Maximum circles reached
  }

  // Find an available slot
  int index = -1;
  for (int i = 0; i < MAX_CIRCLES; i++) {
    if (!circles[i].active) {
      index = i;
      break;
    }
  }

  if (index == -1) {
    // No slots available (shouldn't happen if activeCircleCount is accurate
    return;
  }

  // Create physics body
  int diameter = 20;                      // 20 pixels diameter
  float x = 200.0f + (rand() % 100 - 50); // Random position near center
  float y = 100.0f;                       // Near the top of the screen

  PxShape circleShape = {.circle = {.radius = diameter / 2.0f}};
  PxBody *body =
      px->world->newDynamicBody(world, circleShape, 1.0f, pxVec2(x, y));

  circles[index].body = body;
  circles[index].active = true;
  activeCircleCount++;
}

static void initPlayer(void) {
  int width = 20;
  int height = 30;
  float x = 200.0f; // Center of screen
  float y = 120.0f; // Start above ground

  PxShape boxShape = {.box = {.width = width, .height = height}};
  PxBody *body = px->world->newDynamicBody(world, boxShape, 1.0f, pxVec2(x, y));

  // Set player properties
  body->restitution = 0.1f;
  body->dynamicFriction = 0.5f;
  body->staticFriction = 0.7f;
  body->density = 0.1f;

  player.body = body;
  player.isGrounded = false;
}

static void drawPlayer(void) {
  if (player.body == NULL) {
    return;
  }

  PxVec2 pos = player.body->position;
  float width = player.body->aabb.max.x - player.body->aabb.min.x;
  float height = player.body->aabb.max.y - player.body->aabb.min.y;

  pd->graphics->fillRect(pos.x - width / 2, pos.y - height / 2, width, height,
                         kColorBlack);
}

static bool checkPlayerGrounded(void) {
  if (player.body == NULL || groundBody == NULL) {
    return false;
  }

  PxVec2 pos = player.body->position;
  float height = player.body->aabb.max.y - player.body->aabb.min.y;

  // Check if bottom of player is near ground surface
  return (
      pos.y + height / 2 >= groundBody->position.y - groundHeight / 2.0f - 2 &&
      pos.y + height / 2 <= groundBody->position.y - groundHeight / 2.0f + 2);
}

static int update(void *userdata) {
  (void)userdata;

  // Clear the screen
  pd->graphics->clear(kColorWhite);

  PDButtons current, pressed;
  pd->system->getButtonState(&current, &pressed, NULL);

  // Check for player input
  // Update grounded state
  player.isGrounded = checkPlayerGrounded();

  // Left/Right movement
  if (current & kButtonLeft) {
    PxVec2 force = pxVec2(-9000, 0);
    px->body->applyForce(player.body, force, pxVec2(20, 0));
  }

  // Left/Right movement
  if (current & kButtonRight) {
    PxVec2 force = pxVec2(9000, 0);
    px->body->applyForce(player.body, force, pxVec2(0, 0));
  }

  // Jump with up button
  if (pressed & kButtonUp /**&& player.isGrounded*/) {
    PxVec2 impulse = pxVec2(0, -9000.0f); // Negative y is up
    px->body->applyImpulse(player.body, impulse, pxVec2(0, 0));
  }

  // Check for A button press to spawn a circle
  if (pressed & kButtonA) {
    spawnCircle();
  }

  if (pressed & kButtonB) {
    // Reset all circles
    for (int i = 0; i < MAX_CIRCLES; i++) {
      if (circles[i].active) {
        px->world->freeBody(world, circles[i].body);
        circles[i].active = false;
        circles[i].body = NULL;
      }
    }
    activeCircleCount = 0;

    // Reset player
    px->world->freeBody(world, player.body);
    initPlayer();
  }

  if (pressed & kButtonDown) {
    showDebugInfo = !showDebugInfo;
  }

  // Step the physics simulation
  px->world->step(world, g);

  // Draw all physics objects
  drawPhysicsObjects();
  drawPlayer();

  if (showDebugInfo) {
    px->world->drawDebug(world);

    // Show debug status
    pd->graphics->drawText("Debug Mode", strlen("Debug Mode"), kASCIIEncoding,
                           5, 5);
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
    pd->display->setRefreshRate(50); // 50 fps
    pd->system->setUpdateCallback(update, NULL);

    // Initialize all circles to inactive
    for (int i = 0; i < MAX_CIRCLES; i++) {
      circles[i].active = false;
      circles[i].body = NULL;
    }
    activeCircleCount = 0;

    // Setup the physics engine
    g = pxVec2(0, 9.8); // Note: positive Y is down in pixel coordinates
    px = registerPdPhyzx(playdate);
    world = px->world->new(10, 50, 8);

    groundWidth = 200;
    groundHeight = 10;
    float groundY = 190;

    PxShape ground = {.box = {.width = groundWidth, .height = groundHeight}};
    groundBody = px->world->newStaticBody(world, ground, pxVec2(200, groundY));
    groundBody->restitution = 0.5f;
    groundBody->dynamicFriction = 1.8f;
    groundBody->staticFriction = 0.9f;

    initPlayer();
    break;

  case kEventTerminate:
    // Free resources when app terminates
    if (world) {
      px->world->free(world);
    }
    break;

  default:
    break;
  }

  return 0;
}
