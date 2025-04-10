#ifndef PDPHYZX_PLATFORM_H
#define PDPHYZX_PLATFORM_H

#include "pd_api.h"

extern PlaydateAPI *pd;

#ifndef pxalloc
#define pxalloc(x) pd->system->realloc(NULL, (x))
#endif

#ifndef pxfree
#define pxfree(a) pd->system->realloc((a), 0)
#endif

#ifndef pxcalloc
#define pxcalloc(a, b) pd->system->realloc(NULL, ((a) * (b)))
#endif

#ifndef pxrealloc
#define pxrealloc pd->system->realloc
#endif

#ifndef pxlog
#define pxlog(s, ...) pd->system->logToConsole((s), ##__VA_ARGS__)
#endif

#endif // PDPHYZX_PLATFORM_H
