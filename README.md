# pdphyzx

<img src="https://github.com/user-attachments/assets/c07b3a99-bbab-4808-987c-981f15a7d249" width="500" />

A high-performance impulse-based physics engine built specifically for the
Playdate console.

> Handles up to 256 physics bodies at max FPS with precise collision detection
> and response - all while staying within the Playdate's performance constraints.

## Project Status

This library is still under active development and is not yet ready for
production use. Key aspects being worked on include:

- **Performance vs. Accuracy Tuning**: Finding optimal parameters to balance
  performance and simulation accuracy
- **Tunneling Prevention**: Implementing systems to prevent fast-moving objects
  from passing through thin objects

### Planned Features

The following features are on the roadmap but not yet implemented:

- **Collision Callbacks**: A system for responding to collision events
  - Considering function pointer callbacks
  - Exploring global current collisions query
  - Investigating collisions along raycasts
- **Additional Shape Types**:
  - Ellipses
  - Pills (capsules)
- **Kinematic Bodies**: Bodies that can be manually positioned while still
  participating in physics

Feel free to open issues or contribute if you're interested in helping develop
these features.

## Features

- **Impulse-Based Simulation**: Uses a sequential impulse solver for realistic
  physical interactions
- **Lightweight**: Single-header STB-style C library
- **Optimized for Playdate**: Handles up to 256 bodies at smooth framerates
- **Simple API**: Direct integration with Playdate's C SDK
- **Shape Support**:
  - Circles with accurate bouncing and rolling
  - Boxes with proper inertia and rotation
  - Convex Polygons (up to 64 vertices) with automatic center-of-mass calculation

## Installation

1. Download the `dist/pdphyzx.h` header file
2. Copy it into your Playdate project
3. Include the header in your code:

```c
#define PDPHYZX_IMPLEMENTATION
#include "pdphyzx.h"
```

## Basic Usage

### Initializing the Physics World

The following example creates a simple physics scene with a bouncing circle, a
box, and a static ground platform:

```c
// Get reference to PlaydateAPI
extern PlaydateAPI *pd;

// Register with the Playdate SDK
PdPhyzxAPI *px = registerPdPhyzx(pd);

// Set up a new physics world
PxVec2 gravity = pxVec2(0, 9.8);  // Note: positive Y is down on Playdate
int iterations = 10;              // Physics solver iterations
int targetFps = 50;               // Target framerate
int scale = 8;                    // Physics to pixel scale
PxWorld *world = px->world->new(iterations, targetFps, scale);
```

### Creating Bodies

```c
// Create a circle
float radius = 10.0f;
PxShape circleShape = {.circle = {.radius = radius}};
PxBody *circleBody = px->world->newDynamicBody(
    world, circleShape, 1.0f, pxVec2(100, 100));

// Create a box
float width = 30.0f;
float height = 20.0f;
PxShape boxShape = {.box = {.width = width, .height = height}};
PxBody *boxBody = px->world->newDynamicBody(
    world, boxShape, 1.0f, pxVec2(200, 100));

// Create a static ground platform
float groundWidth = 200.0f;
float groundHeight = 10.0f;
PxShape groundShape = {.box = {.width = groundWidth, .height = groundHeight}};
PxBody *groundBody = px->world->newStaticBody(
    world, groundShape, pxVec2(200, 190));

// Configure physics properties
circleBody->restitution = 0.5f;        // Bounciness
circleBody->dynamicFriction = 0.5f;    // Friction while moving
circleBody->staticFriction = 0.7f;     // Friction while still
```

### Applying Forces and Impulses

```c
// Apply an impulse (instant force)
PxVec2 impulse = pxVec2(0, -500.0f);  // Negative y is upward
PxVec2 point = pxVec2(0, 0);          // Apply at center of mass
px->body->applyImpulse(circleBody, impulse, point);

// Apply a continuous force
PxVec2 force = pxVec2(200.0f, 0);     // Force to the right
px->body->applyForce(circleBody, force, point);
```

### Updating the Simulation

```c
// Call this each frame to advance physics
px->world->step(world, gravity);
```

## API Reference

pdphyzx uses a function pointer-based API design to maintain a clean interface
while allowing for efficient internal implementation. The API is organized into
logical domains:

### Core API

```c
// Initialize the physics system
PdPhyzxAPI *px = registerPdPhyzx(playdate);

// Access specific domains
const PxWorldAPI *worldAPI = px->world;
const PxBodyAPI *bodyAPI = px->body;
```

### World API

The World API handles physics world creation, simulation, and body management:

```c
// Create a new physics world
PxWorld *world = px->world->new(
    10,               // Solver iterations (more = more accurate but slower)
    50,               // Target frames per second
    8);               // Scale (pixels per meter)

// Create physics bodies
PxBody *staticBody = px->world->newStaticBody(
    world,            // Physics world
    groundShape,      // Shape definition
    position);        // Initial position

PxBody *dynamicBody = px->world->newDynamicBody(
    world,            // Physics world
    circleShape,      // Shape definition
    1.0f,             // Density (kg/m²)
    position);        // Initial position

// Advance the physics simulation
px->world->step(
    world,            // Physics world
    pxVec2(0, 9.8));  // Gravity vector

// Show debug visualization
px->world->drawDebug(world);

// Remove a body
px->world->freeBody(world, body);

// Destroy the world
px->world->free(world);
```

### Body API

The Body API handles operations on individual physics bodies:

```c
// Set the exact orientation of a body
px->body->setOrientation(
    body,             // The body to modify
    PX_H_PI);         // Orientation angle in radians

// Apply a rotation to the current orientation
px->body->rotate(
    body,             // The body to modify
    0.1f);            // Angle to rotate by in radians

// Apply a continuous force
px->body->applyForce(
    body,             // The body to modify
    pxVec2(0, -100),  // Force vector
    pxVec2(10, 0));   // Contact point relative to center of mass

// Apply an instantaneous impulse
px->body->applyImpulse(
    body,             // The body to modify
    pxVec2(0, -500),  // Impulse vector
    pxVec2(0, 0));    // Contact point (center of mass in this example)
```

### Direct Property Access

While the API methods should be used for most operations, some properties can be
accessed and modified directly:

```c
// Material properties
body->restitution = 0.5f;        // Bounciness (0-1)
body->dynamicFriction = 0.3f;    // Friction for moving bodies
body->staticFriction = 0.5f;     // Friction for stationary bodies

// Query body state
PxVec2 position = body->position;
PxVec2 velocity = body->velocity;
float angle = body->orientationAngle;
float angularVelocity = body->angularVelocity;

// Access body shape information
if (body->collider.type == PX_CIRCLE) {
    float radius = body->collider.shape.circle.radius;
}
else if (body->collider.type == PX_POLYGON) {
    PxPolygonData poly = body->collider.shape.polygon;
    uint8_t vertexCount = poly.vertices.length;
}

// Check body AABB (useful for quick checks)
PxAABB aabb = body->aabb;
```

## Memory Management

pdphyzx uses an internally managed memory system for physics bodies, with some
important considerations:

### Body Lifecycle

- **Creation**: Bodies are created through the world API and remain valid until
  explicitly freed
- **Reference**: You receive and store pointers to bodies, which remain stable
- **Destruction**: Bodies must be freed using `world->freeBody()` when no longer
  needed
- **Invalidation**: After freeing a body, its pointer becomes invalid and should
  not be used

### Memory Management Examples

```c
// Create body through world API
PxBody *body = px->world->newDynamicBody(world, shape, density, pos);

// Use the body freely as long as it exists
body->restitution = 0.5f;
px->body->applyImpulse(body, impulse, contact);

// When done with the body, free it through the world API
px->world->freeBody(world, body);

// INCORRECT: Using the pointer after freeing
px->world->freeBody(world, body);
body->velocity = pxVec2(0, 0);  // INVALID - body no longer exists!
```

### Safe Usage Patterns

- Always check if pointers are still valid before using them if there's uncertainty
- Set pointers to NULL after freeing them to prevent accidental reuse
- Consider implementing a simple ID system for entities if you need to check validity

## Rendering Approaches

pdphyzx supports two approaches to rendering physics bodies:

### 1. Manual Drawing (No Sprites)

```c
// Draw a circle
if (body->collider.type == PX_CIRCLE) {
    float radius = body->collider.shape.circle.radius;
    pd->graphics->fillEllipse(
        body->position.x - radius,
        body->position.y - radius,
        radius * 2, radius * 2,
        0.0f, 360.0f, kColorBlack
    );
}

// Draw a box
else if (body->collider.type == PX_POLYGON) {
    float width = body->aabb.max.x - body->aabb.min.x;
    float height = body->aabb.max.y - body->aabb.min.y;

    pd->graphics->fillRect(
        body->position.x - width / 2,
        body->position.y - height / 2,
        width, height, kColorBlack
    );
}
```

### 2. Sprite Integration

pdphyzx integrates seamlessly with the Playdate SDK sprite system, enabling
physics-driven animations with optimized performance.

#### Entity Component Pattern

The recommended approach is to create structures that combine physics bodies
with sprites:

```c
// Define a game entity with physics and visual components
typedef struct {
    PxBody *body;
    LCDSprite *sprite;
} PhysicsEntity;
```

#### Creating Physics-Driven Sprites

```c
PhysicsEntity createEntity(PxWorld *world, PxVec2 position) {
    // 1. Create the physics body
    PxShape shape = {.circle = {.radius = 10.0f}};
    PxBody *body = px->world->newDynamicBody(world, shape, 1.0f, position);

    // 2. Set physics properties
    body->restitution = 0.4f;
    body->dynamicFriction = 0.1f;
    body->staticFriction = 0.1f;

    // 3. Create the sprite
    LCDSprite *sprite = pd->sprite->newSprite();

    // 4. Connect sprite to physics body via userdata
    pd->sprite->setUpdateFunction(sprite, updateSpriteCallback);
    pd->sprite->setUserdata(sprite, body);
    pd->sprite->addSprite(sprite);

    return (PhysicsEntity){.sprite = sprite, .body = body};
}

// Clean up resources
void freeEntity(PhysicsEntity *entity, PxWorld *world) {
    px->world->freeBody(world, entity->body);
    pd->sprite->removeSprite(entity->sprite);
}
```

#### Sprite Update Callback

```c
static void updateSpriteCallback(LCDSprite *sprite) {
    // Get physics body from sprite's userdata
    PxBody *body = pd->sprite->getUserdata(sprite);

    // Update sprite position to match physics body
    pd->sprite->moveTo(sprite, body->position.x, body->position.y);

    // Update sprite rotation or image based on physics body's orientation
    updateSpriteImage(sprite, body->orientationAngle);
}
```

#### Optimized Rotation with Bitmap Tables

For efficient rendering of rotated sprites, use pre-rendered bitmap tables
instead of runtime rotation:

```c
// Load bitmap table on initialization
static LCDBitmapTable *bmpTable = NULL;
static void preloadBitmaps() {
    if (!bmpTable) {
        bmpTable = loadBitmapTableAtPath("images/object");
    }
}

// Update sprite image based on orientation angle
static void updateSpriteImage(LCDSprite *sprite, float radians) {
    int frameIndex;
    LCDBitmapFlip flip;

    // Convert orientation angle to frame index and flip settings
    getFrameAndFlip(radians, &frameIndex, &flip);

    // Apply to sprite
    LCDBitmap *bmp = pd->graphics->getTableBitmap(bmpTable, frameIndex);
    pd->sprite->setImage(sprite, bmp, flip);
}

// Efficient angle to frame conversion with quadrant optimization
static void getFrameAndFlip(float radians, int *outFrameIndex,
                           LCDBitmapFlip *outFlip) {
    // Determine quadrant (0-3)
    int quadrant = (int)(radians / (PX_PI/2));

    // Normalize angle to 0-π/2 range
    float normalizedAngle = radians - (quadrant * (PX_PI/2));

    // Convert to frame index (example: 20 frames per quadrant)
    float framesPerRadian = 20 * 2.0f / PX_PI;
    int frameInQuadrant = (int)(normalizedAngle * framesPerRadian);

    // Apply quadrant-specific transformations
    if (quadrant & 1) { // Quadrants 1 and 3
        *outFrameIndex = MAX_FRAMES - 1 - frameInQuadrant;
        *outFlip = (quadrant == 1) ? kBitmapFlippedY : kBitmapFlippedX;
    } else { // Quadrants 0 and 2
        *outFrameIndex = frameInQuadrant;
        *outFlip = (quadrant == 0) ? kBitmapUnflipped : kBitmapFlippedXY;
    }
}
```

#### Game Loop Integration

```c
// In your main update function
void update() {
    // Step physics simulation
    px->world->step(world, gravity);

    // Update and draw all sprites (physics update happens in sprite callbacks)
    pd->sprite->updateAndDrawSprites();

    // Optional: Draw debug visualization
    if (showDebugInfo) {
        px->world->drawDebug(world);
    }
}
```

## Math Utilities

pdphyzx includes highly optimized math utilities that boost performance on the
Playdate's resource-constrained hardware. These functions sacrifice a small
amount of precision for significant speed improvements:

### Fast Math Operations

```c
// Fast inverse square root (Quake III algorithm)
float invSqrt = pxFastRsqrt(value);

// Fast reciprocal (1/x)
float reciprocal = pxFastRcp(value);

// Safe reciprocal that handles division by zero
float safeRecip = pxFastSafeRcp(value);

// Fast division
float quotient = pxFastDiv(numerator, denominator);

// Fast square root
float squareRoot = pxFastSqrt(value);
```

### Utility Functions

```c
// Clamp a value between minimum and maximum
float clamped = pxClamp(value, min, max);

// Convert float to rounded integer
int32_t rounded = pxToInt(floatValue);

// Biased comparison (for numerical stability in physics)
bool isGreater = pxBiasGt(a, b);

// Minimum of two values
float minValue = pxMin(a, b);

// Random float in range
float random = pxRandf(min, max);
```

### Mathematical Constants

```c
PX_PI        // π (3.14159...)
PX_2_PI      // 2π
PX_H_PI      // π/2
PX_EPSILON   // Small value for floating-point comparisons (0.0001)
```

### Vector Utilities

pdphyzx includes a powerful 2D vector system with a comprehensive set of
operations for game development:

#### Vector Creation and Access

```c
// Create a new vector
PxVec2 position = pxVec2(100.0f, 200.0f);

// Access components (multiple ways)
float x = position.x;        // Using named fields
float y = position.y;
float component = position.v[1];  // Using array access (v[0] is x, v[1] is y)

// Modify an existing vector
pxVec2Set(&position, 150.0f, 250.0f);
```

#### Basic Operations

```c
// Vector addition
PxVec2 sum = pxVec2Add(vec1, vec2);            // New vector
pxVec2AddAssign(&vec1, vec2);                  // In-place modification

// Vector subtraction
PxVec2 difference = pxVec2Sub(vec1, vec2);     // New vector
pxVec2SubAssign(&vec1, vec2);                  // In-place modification

// Vector negation
PxVec2 opposite = pxVec2Neg(vec);              // New vector
pxVec2NegAssign(&vec);                         // In-place modification

// Scalar operations
PxVec2 scaled = pxVec2Multf(vec, 2.0f);        // Multiply by scalar
pxVec2MultfAssign(&vec, 0.5f);                 // In-place scalar multiplication
PxVec2 increased = pxVec2Addf(vec, 10.0f);     // Add scalar to both components
```

#### Vector Products and Metrics

```c
// Dot product
float dot = pxVec2Dot(vec1, vec2);             // Scalar product

// Cross product
float cross = pxVec2Cross(vec1, vec2);         // 2D cross product (scalar)
PxVec2 crossVec = pxVec2Crossf(vec, scalar);   // Cross with scalar (vector)

// Length calculations
float lengthSq = pxVec2LenSqr(vec);            // Squared length (faster)
float length = pxVec2Len(vec);                 // Actual length

// Distance between vectors
float distSq = pxVec2DistSqr(vec1, vec2);      // Squared distance (faster)
```

#### Transformations

```c
// Normalize vector to unit length
pxVec2Normalize(&vec);                         // In-place normalization

// Rotate vector by angle
pxVec2Rotate(&vec, PX_H_PI);                   // Rotate by π/2 radians

// Calculate tangent vector
PxVec2 tangent = pxVec2Tangent(vec, normal);   // Project and subtract
```

#### Vector Arrays

pdphyzx includes built-in support for vector arrays, useful for polygon vertices
and contact points:

```c
// Fixed-size array for exactly 2 vectors (e.g., contact points)
PxVec2Array2 contactPoints = {
    .length = 2,
    .items = { pxVec2(10, 20), pxVec2(30, 40) }
};

// Variable-size array for up to 32 vectors (e.g., polygon vertices)
PxVec2Array vertices = { .length = 4 };
pxVec2ArraySet(&vertices, 0, pxVec2(0, 0));
pxVec2ArraySet(&vertices, 1, pxVec2(100, 0));
pxVec2ArraySet(&vertices, 2, pxVec2(100, 100));
pxVec2ArraySet(&vertices, 3, pxVec2(0, 100));

// Safe access (returns zero vector for out-of-bounds indices)
PxVec2 vertex = pxVec2ArrayGet(vertices, 2);
```

### Matrix Utilities

pdphyzx includes a 2×2 matrix system for handling rotations and transformations:

#### Matrix Structure and Creation

```c
// Create an identity matrix
PxMat2 identity = pxMat2Identity();

// Create a rotation matrix from angle
float angle = PX_H_PI;  // π/2 radians (90 degrees)
PxMat2 rotation = pxMat2Orientation(angle);

// Access matrix elements (multiple ways)
float element = rotation.m01;        // Named fields (m<row><col>)
float same = rotation.m[0][1];      // 2D array access
float alsoSame = rotation.v[1];     // Linear array access (row-major)
```

#### Matrix Operations

```c
// Set rotation of existing matrix
PxMat2 matrix;
mat2SetOrientation(&matrix, angle);

// Transpose a matrix
PxMat2 transposed = pxMat2Transpose(matrix);

// Multiply matrix by vector (transform vector)
PxVec2 rotated = pxMat2MultVec2(rotation, vector);
```

#### Common Usage Patterns

```c
// Transform from local to world space
PxVec2 localPoint = pxVec2(10, 5);
PxVec2 worldPoint = pxMat2MultVec2(body->orientation, localPoint);
worldPoint = pxVec2Add(worldPoint, body->position);

// Transform from world to local space
PxVec2 worldPoint = pxVec2(100, 100);
PxVec2 relativePoint = pxVec2Sub(worldPoint, body->position);
PxVec2 localPoint = pxMat2MultVec2(pxMat2Transpose(body->orientation),
                                  relativePoint);
```

## Technical Overview

pdphyzx implements a full-featured 2D physics pipeline:

### Collision Detection

- Efficient sweep and prune broadphase along x-axis
- SAT (Separating Axis Theorem) for polygon-polygon collisions
- Support mapping functions for GJK-derived algorithms
- Specialized circle-polygon and circle-circle detection
- Sutherland-Hodgman clipping for accurate contact point generation

### Physics Solver

- Sequential impulse solver with configurable iteration count
- Two-phase collision resolution (velocity + position)
- Advanced sleeping system for inactive bodies
- Resting contact detection to prevent jitter

### Shape Processing

- Automatic convex hull generation via Graham scan algorithm
- Runtime calculation of face normals for collision detection
- Automatic center of mass calculation and vertex recentering
- Pre-computed maximum radius for optimization

### Performance Optimizations

- AABB-based early rejection of collision pairs
- Spatial sorting for efficient broad-phase collision detection
- Object pooling for contacts to minimize allocations
- Semi-fixed timestep with configurable substeps

## Performance Tips

- Use the smallest number of iterations that gives acceptable stability
- Keep polygon vertex counts low for better performance
- Use bitmap tables instead of runtime sprite rotation
- Remove off-screen bodies to prevent unnecessary computation
- Leverage the sleep system by allowing static objects to sleep
- Use the debug visualization to identify performance issues

## Building and Contributing

This project uses a custom build system based on [nob.h](https://github.com/tsoding/nob).

### Prerequisites

- PlaydateSDK installed
- Clang compiler
- macOS environment (currently only tested on macOS)

### Setup

1. Bootstrap the build system:

   ```bash
   cc -o nob .nob/nob.c
   ```

2. Build the examples:

   ```bash
   ./nob build
   ```

3. Run examples:

   ```bash
   ./nob run sprites # name of example directory
   ```

4. View all available commands:

   ```bash
   ./nob help
   ```

The build tool also generates `compile_commands.json` for clangd LSP integration.

## Example Applications

pdphyzx includes two example projects:

1. **`/draw`** - Shows how to manually render shapes without sprites
2. **`/sprites`** - Demonstrates integration with the Playdate SDK sprite system

## License

MIT License - see the [LICENSE](LICENSE) file for details

## Acknowledgements

pdphyzx draws inspiration and references from:

- [box2d-lite](https://github.com/erincatto/box2d-lite)
- [Chipmunk2D](https://github.com/slembcke/Chipmunk2D)
- [Randy Gaul's Impulse Engine](https://github.com/RandyGaul/ImpulseEngine)
