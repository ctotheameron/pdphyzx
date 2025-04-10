#include "bmp.h"

#include "circle.h"

#define SPRITE_MAP "images/circle"
#define SPRITE_R 10
#define SPRITE_D SPRITE_R * 2
#define SPRITE_MAP_FRAMES 20

// Calculate frame in quadrant - optimize with pre-calculated constant
// 2xSPRITE_MAP_FRAMES is the scaling factor to convert from [0, π/2] to [0, 9]
#define ROT_FRAME (SPRITE_MAP_FRAMES * 2.0f) / PX_PI

extern PlaydateAPI *pd;
extern PdPhyzxAPI *px;
static LCDBitmapTable *bmpTable;

static void preloadImage(void) { bmpTable = loadBmpTableAtPath(SPRITE_MAP); }

static void getFrameAndFlip(float radians, int *outBmpIndex,
                            LCDBitmapFlip *outFlip) {

  // Fast quadrant determination (0-3)
  int quadrant = (int)(pxFastDiv(radians, PX_H_PI));
  if (quadrant >= 4) {
    quadrant = 0; // Safety check for edge cases
  }

  // Get the normalized angle within the quadrant (0 to π/2)
  float normalizedAngle = radians - (quadrant * PX_H_PI);

  int maxFrameIdx = SPRITE_MAP_FRAMES - 1;

  int frameInQuadrant = (int)(normalizedAngle * ROT_FRAME);
  if (frameInQuadrant > maxFrameIdx) {
    frameInQuadrant = maxFrameIdx; // Clamp for safety
  }

  if (quadrant & 0x1) { // Odd quadrants (1, 3) use reverse ordering
    *outBmpIndex = maxFrameIdx - frameInQuadrant;
    *outFlip = (quadrant == 1) ? kBitmapFlippedY : kBitmapFlippedX;
  } else { // Even quadrants (0, 2) use normal ordering
    *outBmpIndex = frameInQuadrant;
    *outFlip = (quadrant == 0) ? kBitmapUnflipped : kBitmapFlippedXY;
  }
}

static void updateSpriteImage(LCDSprite *sprite, float radians) {
  int bmpIndex;
  LCDBitmapFlip flip;

  getFrameAndFlip(radians, &bmpIndex, &flip);

  // Find the corresponding rotation sprite in our bitmap table
  LCDBitmap *bmp = pd->graphics->getTableBitmap(bmpTable, bmpIndex);

  pd->sprite->setImage(sprite, bmp, flip);
}

static void updateSprite(LCDSprite *sprite) {
  PxBody *body = pd->sprite->getUserdata(sprite);

  updateSpriteImage(sprite, body->orientationAngle);
  pd->sprite->moveTo(sprite, body->position.x, body->position.y);
}

Circle newCircle(PxWorld *world, PxVec2 position) {
  if (!bmpTable) {
    preloadImage();
  }

  PxBody *body = px->world->newDynamicBody(
      world, (PxShape){.circle = {.radius = SPRITE_R}}, 1, position);

  LCDSprite *sprite = pd->sprite->newSprite();
  pd->sprite->setCollisionsEnabled(sprite, false);
  pd->sprite->setUpdateFunction(sprite, updateSprite);
  pd->sprite->setUserdata(sprite, body);
  pd->sprite->addSprite(sprite);

  return (Circle){.sprite = sprite, .body = body};
}

void freeCircle(Circle *circle, PxWorld *world) {
  px->world->freeBody(world, circle->body);
  pd->sprite->removeSprite(circle->sprite);
}
