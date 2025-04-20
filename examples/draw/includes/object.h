#ifndef OBJECT_H
#define OBJECT_H

#include <stdint.h>

#include "pdphyzx.h"

#define MAX_OBJECTS 64

extern PdPhyzxAPI *px;

typedef enum {
  UPDATE_DRAW,
  UPDATE_REMOVE,
} ObjectUpdateResult;

typedef struct Object {
  PxBody *body;
  ObjectUpdateResult (*update)(struct Object *self);
  void (*draw)(struct Object *self);
  uint8_t id; // Index in the objects array
} Object;

typedef struct {
  Object items[MAX_OBJECTS];
  uint8_t activeIndices[MAX_OBJECTS];
  uint8_t freedIndices[MAX_OBJECTS];
  uint8_t length;     // Number of active objects
  uint8_t freedCount; // Number of freed slots
} Objects;

static inline Object *objectsAdd(Objects *objects, Object obj) {
  uint8_t index;
  if (objects->freedCount > 0) {
    // Reuse a freed slot
    index = objects->freedIndices[--objects->freedCount];
  } else if (objects->length < MAX_OBJECTS) {
    index = objects->length;
  } else {
    return NULL; // No space
  }

  // Update the activeIndices and increment length
  objects->items[index] = obj;
  objects->items[index].id = index; // Set the ID to the index
  objects->activeIndices[objects->length++] = index;

  return &objects->items[index];
}

static inline void objectsRemove(Objects *objs, PxWorld *world, Object *obj) {
  if (!obj || objs->length == 0 || obj->id >= MAX_OBJECTS) {
    return;
  }

  // Find the index of the object in activeIndices
  uint8_t idx = 0;
  for (uint8_t i = 0; i < objs->length; i++) {
    if (objs->activeIndices[i] == obj->id) {
      idx = i;
      break;
    }
  }

  // Remove the object from the world
  px->world->freeBody(world, obj->body);

  uint8_t objIdx = objs->activeIndices[idx];

  // Add to freed stack
  objs->freedIndices[objs->freedCount++] = objIdx;

  // Remove from activeIndices by swapping with last
  uint8_t lastIdx = --objs->length;
  objs->activeIndices[idx] = objs->activeIndices[lastIdx];

  // Update the id of the swapped-in object
  objs->items[objs->activeIndices[idx]].id = objs->activeIndices[idx];
}

static inline void objectsRemoveAll(Objects *objects, PxWorld *world) {
  for (uint8_t i = 0; i < objects->length; i++) {
    Object *obj = &objects->items[objects->activeIndices[i]];

    if (obj->body) {
      px->world->freeBody(world, obj->body);
    }
  }

  objects->length = 0;
  objects->freedCount = 0;
}

static inline void updateAndDrawObjects(Objects *objects, PxWorld *world) {

  for (uint8_t i = 0; i < objects->length; i++) {
    Object *obj = &objects->items[objects->activeIndices[i]];

    switch (obj->update(obj)) {
    case UPDATE_REMOVE:
      objectsRemove(objects, world, obj);
      --i; // Adjust index after removal
      continue;

    case UPDATE_DRAW:
      obj->draw(obj);
      continue;

    default:
      continue;
    }
  }
}

#endif // OBJECT_H
