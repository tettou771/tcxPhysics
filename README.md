# tcxPhysics

3D rigid body physics for [TrussC](https://github.com/TrussC-org/TrussC), powered by
[Jolt Physics](https://github.com/jrouwe/JoltPhysics) (the engine behind *Horizon
Forbidden West* and *Death Stranding 2*).

Throw hundreds — thousands — of simple primitives (boxes, spheres) into a scene
and let them tumble. Jolt is multi-core on desktop and **runs on the web**
(Emscripten / WebAssembly) too, automatically falling back to a single-threaded
job system there.

> Jolt is fetched and built for you via CMake `FetchContent` — no manual setup.
> It's the 2D addon `tcxBox2d`'s 3D sibling.

---

## Quick start

```cpp
#include <TrussC.h>
#include <tcxPhysics.h>
using namespace std;
using namespace tc;
using namespace tcx;

class tcApp : public App {
    EasyCam cam;
    PhysicsWorld world;
    Mesh cube;
    vector<PhysicsBody> blocks;

    void setup() override {
        cam.setDistance(440);
        cam.enableMouseInput();
        cube = createBox(1.0f);

        world.setup();
        world.setGravity(Vec3(0, -400, 0));   // punchy gravity for this scale
        world.addGroundPlane(0.0f);           // static floor at y = 0
    }

    void update() override {
        // drop a block each frame
        blocks.push_back(world.addBox(Vec3(random(-40, 40), 220, 0), Vec3(10)));
        world.update(1.0f / 60.0f);           // step the simulation
    }

    void draw() override {
        clear(0.1f, 0.1f, 0.12f);
        cam.begin();
        for (auto& b : blocks) {
            pushMatrix();
            translate(b.getPosition());
            rotate(b.getRotation());
            Vec3 s = b.getSize();
            scale(s.x, s.y, s.z);
            cube.draw();
            popMatrix();
        }
        cam.end();
    }
};
```

See **`example-cubeRain/`** for the full version: hold the mouse to pour ~100
apple-red cubes per second into a pile, lit with a key + fill light, with a live
cube count and FPS readout.

---

## API

### `PhysicsWorld`

| Method | What it does |
|--------|--------------|
| `setup(maxBodies = 10240)` | Initialize the simulation. Call once. |
| `setGravity(Vec3)` / `getGravity()` | Gravity vector (default `(0, -9.81, 0)`). |
| `update(dt = 1/60, collisionSteps = 1)` | Step the simulation once per frame. |
| `addBox(pos, size, dynamic = true)` | Add a box. `size` is full extents; `pos` is its center. Returns a `PhysicsBody`. |
| `addSphere(pos, radius, dynamic = true)` | Add a sphere. |
| `addGroundPlane(y = 0, size = 100000)` | A large static floor centered on `(0, y, 0)`. |
| `removeBody(body)` | Remove one body. |
| `clearDynamicBodies()` | Remove every dynamic body, keep static scenery. |
| `getNumBodies()` | Total body count. |

`dynamic = false` makes a body static (floor, walls, scenery) — it never moves
but everything collides against it.

### `PhysicsBody`

A lightweight copyable handle (world pointer + id). It owns nothing; the body
lives in the `PhysicsWorld`.

| Method | What it does |
|--------|--------------|
| `isValid()` | False if default-constructed or the body was removed. |
| `getPosition()` → `Vec3` | World-space center. |
| `getRotation()` → `Quaternion` | World-space orientation (feed to `rotate()`). |
| `getSize()` → `Vec3` | Full extents of the shape's bounding box. |

---

## Units & gravity

Jolt is unit-agnostic — it simulates in whatever units you feed it. The default
gravity is the physical `-9.81`, which looks *very slow* in a typical TrussC
scene where objects are tens of units across. For lively motion, scale gravity
up (the example uses `-400`), or work in smaller world units.

---

## Web (WebAssembly)

`tcxPhysics` builds for the web. The addon detects Emscripten and uses Jolt's
single-threaded job system, and the CMake build enables **wasm SIMD**
(`-msimd128`) so Jolt's math runs near native speed. Heavy CPU work (the physics
step) runs in wasm; rendering goes through TrussC's WebGPU backend — the two are
independent, so a few hundred blocks stay smooth.

> CI currently builds desktop platforms (macOS / Windows / Linux). Web is a
> supported target you build locally via the `web` preset.

---

## License

`tcxPhysics` is MIT. It builds Jolt Physics, which is also MIT. See
[`LICENSES.md`](LICENSES.md).
