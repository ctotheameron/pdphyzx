#include "pd_api.h"
#include "pdphyzx.h"

#include "includes/circle.h"

#define RADIUS 10.0f

extern PdPhyzxAPI *px;
extern PlaydateAPI *pd;

static ObjectUpdateResult update(Object *self) {
  PxBody *body = self->body;

  // Remove circle if it has gone off-screen
  PxVec2 pos = body->position;
  if (pos.y > 300 || pos.x < -50 || pos.x > 450 || pos.y < -50) {
    return UPDATE_REMOVE;
  }

  return UPDATE_DRAW;
}

static void draw(Object *self) {
  PxBody *body = self->body;
  float radius = body->collider.shape.circle.radius;
  float diameter = radius * 2.0f;
  PxVec2 origin = pxVec2Subf(body->position, radius);

  float degrees = body->orientationAngle * pxFastDiv(180.0f, PX_PI);

  // Draw the circle
  pd->graphics->fillEllipse(origin.x, origin.y, diameter, diameter,
                            degrees + 10, 360 + degrees, kColorBlack);
}

Object newCircle(PxWorld *world, PxVec2 position) {
  // Create physics body
  PxShape shape = {.circle = {.radius = RADIUS}};
  float density = 1.0f;
  PxBody *body = px->world->newDynamicBody(world, shape, density, position);

  // Physics properties
  body->restitution = 0.8f; // Bounciness
  body->dynamicFriction = 0.1f;
  body->staticFriction = 0.1f;

  return (Object){.body = body, .update = update, .draw = draw};
}
