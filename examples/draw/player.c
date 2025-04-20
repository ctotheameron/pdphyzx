#include "pd_api.h"
#include "pdphyzx.h"

#include "includes/player.h"

#define WIDTH 20.0f  // millimeters
#define HEIGHT 30.0f // millimeters
#define H_WIDTH (WIDTH / 2.0f)
#define H_HEIGHT (HEIGHT / 2.0f)

extern PdPhyzxAPI *px;
extern PlaydateAPI *pd;

static ObjectUpdateResult update(Object *self) {
  (void)self;

  PDButtons current, pressed;
  pd->system->getButtonState(&current, &pressed, NULL);

  float velocityX = self->body->velocity.x;
  float velocityY = self->body->velocity.y;

  bool isGrounded = velocityY > -1.0f && velocityY < 1.0f;

  // Left movement
  if (current & kButtonLeft && velocityX > -100.0f) {
    px->body->applyForce(self->body, -0.005f, 0, 0, 0);
  }

  // Right movement
  if (current & kButtonRight && velocityX < 100.0f) {
    px->body->applyForce(self->body, 0.005f, 0, 0, 0);
  }

  // Jump with up button
  if (pressed & kButtonUp && isGrounded) {
    px->body->applyImpulse(self->body, 0, -0.0001f, 0, 0);
  }

  return UPDATE_DRAW;
}

static void draw(Object *self) {
  PxVec2 pos = self->body->position;
  PxVec2Array vertices = self->body->collider.shape.polygon.vertices;
  PxMat2 orientation = self->body->orientation;

  // Draw lines connecting each vertex
  for (uint8_t i = 0; i < vertices.length; i++) {
    // Get current and next vertex (loop back to first for the last one)
    PxVec2 vCur = vertices.items[i];
    PxVec2 vNext = vertices.items[(i + 1) % vertices.length];

    // Apply rotation to vertices
    PxVec2 rotatedCurrent = pxMat2MultVec2(orientation, vCur);
    PxVec2 rotatedNext = pxMat2MultVec2(orientation, vNext);

    // Add body position to get world coordinates
    PxVec2 v0 = pxVec2Add(pos, rotatedCurrent);
    PxVec2 v1 = pxVec2Add(pos, rotatedNext);

    // Draw the edge
    pd->graphics->drawLine(v0.x, v0.y, v1.x, v1.y, 1, kColorBlack);
  }
}

Object newPlayer(PxWorld *world, PxVec2 position) {
  // Create physics body
  PxShape shape = {.box = {.width = WIDTH, .height = HEIGHT}};
  float density = 0.8f;
  PxBody *body = px->world->newDynamicBody(world, shape, density, position);

  // Physics properties
  body->restitution = 0.1f;
  body->dynamicFriction = 0.0f;
  body->staticFriction = 0.5f;

  return (Object){.body = body, .update = update, .draw = draw};
}
