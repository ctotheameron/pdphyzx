#ifndef PDPHYZX_PX_PLATFORM_H
#define PDPHYZX_PX_PLATFORM_H

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
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wgnu-zero-variadic-macro-arguments"
#define pxlog(s, ...) pd->system->logToConsole((s), ##__VA_ARGS__)
#pragma clang diagnostic pop
#endif

#endif // PDPHYZX_PX_PLATFORM_H
