# pdphyzx

![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/ctotheameron/pdphyzx/build-and-release.yml?branch=main&style=flat-square)
![GitHub Release](https://img.shields.io/github/v/release/ctotheameron/pdphyzx?style=flat-square&color=%23fac852&link=https%3A%2F%2Fgithub.com%2Fctotheameron%2Fpdphyzx%2Freleases%2Flatest)

A lightweight, impulse-based 2D physics engine designed specifically for the
Playdate console. Handle up to 256 physics bodies at smooth framerates while
staying within Playdate's performance constraints.

<img alt="demo" src="https://github.com/user-attachments/assets/c07b3a99-bbab-4808-987c-981f15a7d249" width="500" />

## ✨ Features

- **📦 Lightweight**: STB-style single-header library with no external dependencies
- **🚀 Performance-Focused**: Optimized for Playdate's hardware
- **🧩 Multiple Shape Types**: Circles, boxes, and convex polygons
- **🎮 Simple API**: Direct integration with Playdate's C SDK

## 📋 Quick Start

### Installation

```bash
# 1. Download the single header file
curl -O https://github.com/ctotheameron/pdphyzx/releases/latest/download/pdphyzx.h

# 2. Add to your project and include it
```

```c
// In your source file
#define PDPHYZX_IMPLEMENTATION
#include "pdphyzx.h"
```

### Basic Usage

#### Setup and configure world constants

```c
#include "pd_api.h"

// STB-style - only add the #define before the first include.
// Note that this must be included **after** the Playdate API header.
#define PDPHYZX_IMPLEMENTATION

// (optional)
// Define the unit system before including the library:
// PDPHYZX_UNIT_MM, PDPHYZX_UNIT_CM, or PDPHYZX_UNIT_M
//
// Default is PDPHYZX_UNIT_M
#define PDPHYZX_UNIT PDPHYZX_UNIT_MM // 1 unit = 1 mm

#include "pdphyzx.h"
#include "pdphyzx.h"

// These don't have to be global, but for example...
extern PlaydateAPI *pd;
PdPhyzxAPI *px = NULL;
PxWorld *world = NULL;

// Should match your target refresh rate
float targetFps = 50
```

#### Initialize simulation

```c
int eventHandler(PlaydateAPI *playdate, PDSystemEvent event, uint32_t arg) {
  switch (event) {
    case: kEventInit:
      pd = playdate;
      pd->display->setRefreshRate(targetFps);
      pd->system->setUpdateCallback(update, NULL);

      px = registerPdPhyzx(playdate);
      world = px->world->new(targetFps);

      // Number of constraint solving iterations
      // (higher = more accurate but slower)
      px->world->setIterations(world, 10);

      // Set the gravity vector (m/s^2)
      // This can change at runtime
      px->world->setGravity(world, 0, 9.8);

      // Create a bouncing ball
      PxShape ballShape = {.circle = {.radius = 10.0f}};
      float ballDensity = 0.5f;
      PxVec2 ballPos = pxVec2(200, 100);
      ball = px->world->newDynamicBody(world, ballShape, ballDensity, ballPos);

      // Configure physics properties
      ball->restitution = 0.5f;        // Bounciness
      ball->dynamicFriction = 0.5f;    // Friction while moving
      ball->staticFriction = 0.7f;     // Friction while still

      // Create a static ground platform
      PxShape groundShape = {.box = {.width = 200.0f, .height = 10.0f};
      PxVec2 groundPos = pxVec2(200, 200);
      ground = px->world->newStaticBody(world, groundShape, groundPos);
      break;

    case kEventTerminate:
      px->world->free(world);
      break;
  }

  return 0;
}
```

#### Event loop

```c

int handleInput(void) {
  PDButtons current, pressed;
  pd->system->getButtonState(&current, &pressed, NULL);

  if (current & kButtonA) {
    // Apply impulse (x,y components for both impulse and contact point)
    float impulseX = 0, impulseY = -100.0f;  // negative y is up
    float contactX = 0, contactY = 0;        // center of body
    px->body->applyImpulse(ball, impulseX, impulseY, contactX, contactY);
  }
}

int update(void *userdata) {
  // Game logic
  handleInput();
  // ...

  // Step physics simulation
  px->world->step(world);

  // Rendering can be handled manually or using Playdate's sprite system.
  // See `examples/` for more details.
  //
  // For manual rendering:
  // pd->graphics->clear(kColorWhite);
  // pd->graphics->fillRect(...);
  // ...
  //
  // When using sprites, simply:
  // pd->sprite->updateAndDrawSprites();

  return 1;
}
```

## 🎮 Example Code

Check out the [examples directory](examples/) for complete demos:

- **[draw](examples/draw/)** - Manual rendering without sprites
- **[sprites](examples/sprites/)** - Integration with Playdate's sprite system

## 📚 API Overview

The API is organized into logical domains for easier usage:

```c
// Core physics domains
px->world     // World creation and management
px->body      // Body manipulation and forces
```

See the [source code](src/) for detailed API documentation.

## 🔍 Unit System

pdphyzx supports multiple unit scales to make it easier to work with different
sized objects:

- `PDPHYZX_UNIT_MM`: 1 unit = 1 millimeter
- `PDPHYZX_UNIT_CM`: 1 unit = 1 centimeter
- `PDPHYZX_UNIT_M`: 1 unit = 1 meter (default)

Define this before including the library:

```c
#define PDPHYZX_UNIT PDPHYZX_UNIT_MM  // Choose your preferred scale
#include "pdphyzx.h"
```

When using `PDPHYZX_UNIT`, sizes and positions are in the specified scale, and
physical quantities like gravity and forces are automatically scaled to match.

Under the covers, this macro tunes internal constants and thresholds to work with
the selected unit system.

If you need further tuning for rendering, you'll need to convert positions and
dimensions manually.

## 🔧 Local Development Setup

### Prerequisites

- [Playdate SDK](https://play.date/dev/) (latest version recommended)
- Clang compiler (currently only supports MacOS + Linux)

### Using the Custom Build System

This project uses a custom build system based on [nob.h](https://github.com/tsoding/nob).

#### Bootstrap the build system

```bash
cc -o nob .nob/nob.c
```

#### Build & Run

```bash
./nob build # build all examples
./nob run sprites # run any project in examples/
./nob help # show available commands
```

## 🛠️ Development Status

This library is under active development. Key features being worked on:

- Performance vs accuracy tuning
- Tunneling prevention for fast-moving objects
- Collision callbacks system
- Additional shape types (ellipses, capsules)
- Kinematic bodies for manual positioning

## 🔬 Under the Hood

### Performance Optimizations

- **No Dynamic Memory Allocation**: Fixed-size arrays and memory pools eliminate
  runtime allocations
- **Sweep and Prune Algorithm**: Efficient broad-phase collision detection using
  sorted AABBs along x-axis
- **Baumgarte Stabilization**: Position correction bias to prevent sinking
  without excessive jitter
- **Sleeping Bodies System**: Inactive objects enter sleep state to save CPU
  cycles
- **Fast Math Approximations**: Optimized sqrt (Quake III), sin/cos using lookup
  tables, and reciprocal calculations

### Collision Detection

- **GJK Algorithm**: For polygon-polygon collision detection using support functions
- **Clipping Algorithm**: For contact point generation between convex polygons
- **Voronoi Region Analysis**: For precise circle-polygon collision detection
- **AABB Overlap Tests**: Quick rejection tests before detailed collision checks

### Physics Solver

- **Sequential Impulse Resolution**: Multi-iteration constraint solver for stability
- **Coulomb Friction Model**: Static and dynamic friction handling with
  realistic transitions
- **Mixed Restitution**: Collision response with coefficient of restitution
  determined by material properties
- **Resting Contact Detection**: Uses velocity thresholds to identify and
  optimize stacking scenarios
- **Separate Linear/Angular Resolution**: Independent handling of linear and
  rotational motion

## 🙌 Acknowledgements

This project wouldn't be possible without these excellent resources:

- **[Randy Gaul's Impulse Engine](https://github.com/RandyGaul/ImpulseEngine)**
- **[Box2D Lite](https://github.com/erincatto/box2d-lite)**
- **[Chipmunk2D](https://github.com/slembcke/Chipmunk2D)**
- **[nob.h](https://github.com/tsoding/nob)**

## 📄 License

MIT License - see the [LICENSE](LICENSE) file for details.

---

Built with 💛 for the [Playdate console](https://play.date)
