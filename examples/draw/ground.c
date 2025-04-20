#include "pd_api.h"
#include "pdphyzx.h"

#include "includes/ground.h"

#define WIDTH 200.0f
#define HEIGHT 10.0f
#define H_WIDTH (WIDTH / 2.0f)
#define H_HEIGHT (HEIGHT / 2.0f)

extern PdPhyzxAPI *px;
extern PlaydateAPI *pd;

static ObjectUpdateResult update(Object *self) {
  (void)self;

  return UPDATE_DRAW;
}

static void draw(Object *self) {
  PxVec2 pos = self->body->position;

  pd->graphics->fillRect(pos.x - H_WIDTH, pos.y - H_HEIGHT, WIDTH, HEIGHT,
                         kColorBlack);
}

Object newGround(PxWorld *world, PxVec2 position) {
  // Create physics body
  PxShape shape = {.box = {.width = WIDTH, .height = HEIGHT}};
  PxBody *body = px->world->newStaticBody(world, shape, position);

  // Physics properties
  body->restitution = 0.5f;
  body->dynamicFriction = 0.1f;
  body->staticFriction = 0.1f;

  Object ground = {.body = body, .update = update, .draw = draw};

  return ground;
}
