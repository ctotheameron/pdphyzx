
#include "../../dist/pdphyzx.h"

typedef struct {
  struct LCDSprite *sprite;
  PxBody *body;
} Ground;

Ground newGround(PxWorld *world, PxVec2 position);
void freeGround(Ground *ground, PxWorld *world);
