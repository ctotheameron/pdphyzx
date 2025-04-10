#include "bmp.h"
#include "pd_api/pd_api_gfx.h"

#include "ground.h"

#define SPRITE "images/ground"
#define SPRITE_W 104
#define SPRITE_H 20
#define SPRITE_MAP_FRAMES 40

// Calculate frame in quadrant - optimize with pre-calculated constant
// 2xSPRITE_MAP_FRAMES is the scaling factor to convert from [0, π/2] to [0, 9]
#define ROT_FRAME (SPRITE_MAP_FRAMES * 2.0f) / PX_PI

extern PdPhyzxAPI *px;
static struct LCDBitmapTable *bmpTable = NULL;

static void preloadImage(void) { bmpTable = loadBmpTableAtPath(SPRITE); }

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

Ground newGround(PxWorld *world, PxVec2 position) {
  if (!bmpTable) {
    preloadImage();
  }

  PxBody *body = px->world->newStaticBody(
      world, (PxShape){.box = {.height = SPRITE_H, .width = SPRITE_W}},
      position);

  // Physics properties
  body->restitution = 0.4f;
  body->dynamicFriction = 0.1f;
  body->staticFriction = 0.1f;

  LCDSprite *sprite = pd->sprite->newSprite();
  pd->sprite->setCollisionsEnabled(sprite, false);
  pd->sprite->setUpdateFunction(sprite, updateSprite);
  pd->sprite->setUserdata(sprite, body);
  pd->sprite->addSprite(sprite);

  return (Ground){.sprite = sprite, .body = body};
}

void freeGround(Ground *ground, PxWorld *world) {
  px->world->freeBody(world, ground->body);
  pd->sprite->removeSprite(ground->sprite);
}
