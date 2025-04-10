
#include "bmp.h"

extern PlaydateAPI *pd;

LCDBitmap *loadBmpAtPath(const char *path) {
  const char *outErr = NULL;
  LCDBitmap *img = pd->graphics->loadBitmap(path, &outErr);

  if (outErr != NULL) {
    pd->system->logToConsole("Error loading bmp at path '%s': %s", path,
                             outErr);
  }

  return img;
}

LCDBitmapTable *loadBmpTableAtPath(const char *path) {
  const char *outErr = NULL;
  LCDBitmapTable *img = pd->graphics->loadBitmapTable(path, &outErr);

  if (outErr != NULL) {
    pd->system->logToConsole("Error loading bmpTable at path '%s': %s", path,
                             outErr);
  }

  return img;
}
