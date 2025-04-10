#include "../../dist/pdphyzx.h"

typedef struct {
  struct LCDSprite *sprite;
  PxBody *body;
} Circle;

Circle newCircle(PxWorld *world, PxVec2 position);
void freeCircle(Circle *circle, PxWorld *world);
