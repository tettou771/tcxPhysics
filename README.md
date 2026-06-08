# tcxPhysics

![cubeRain demo](docs/preview.png)

> ⚠️ **Work in progress.** This addon works, but it's early — **the API may still
> change without notice** between versions. Pin a commit/tag if you need stability.
> Feedback welcome. [Live web demo →](https://tettou771.github.io/tcxPhysics/)

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
        // Metre-scale scene (see "Units & scale"): default gravity -9.81 is natural.
        cam.setTarget(0, 0.5f, 0);
        cam.setDistance(5);
        cam.setElevation(0.4f);               // oblique 3/4 view
        cam.enableMouseInput();
        cube = createBox(1.0f);

        world.setup();
        world.addGroundPlane(0.0f);           // static floor at y = 0
    }

    void update() override {
        // drop a ~0.2 m block each frame
        blocks.push_back(world.addBox(Vec3(random(-0.5f, 0.5f), 3, 0), Vec3(0.2f)));
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
| `addBox(pos, size, dynamic = true, density = 1000)` | Add a box. `size` is full extents; `pos` is its center. `mass = density × volume`. Returns a `PhysicsBody`. |
| `addSphere(pos, radius, dynamic = true, density = 1000)` | Add a sphere. |
| `addGroundPlane(y = 0, size = 100000)` | A large static floor centered on `(0, y, 0)`. |
| `removeBody(body)` | Remove one body. |
| `clearDynamicBodies()` | Remove every dynamic body, keep static scenery. |
| `getNumBodies()` | Total body count. |
| `updateAsyncStart(hz = 120)` / `updateAsyncStop()` / `isAsync()` | Step on a fixed-timestep clock — see [Async stepping](#async-stepping). |
| `contactBegan` / `contactEnded` | `tc::Event<ContactEventArgs>` — see [Contact events](#contact-events). |
| `nativeSystem()` / `nativeBodyInterface()` | Raw Jolt pointers (`void*`) — see [Advanced: raw Jolt](#advanced-raw-jolt-escape-hatch). |

`dynamic = false` makes a body static (floor, walls, scenery) — it never moves
but everything collides against it.

### `PhysicsBody`

A lightweight copyable handle (world pointer + id). It owns nothing; the body
lives in the `PhysicsWorld`. The setters return `*this`, so they chain.

| Method | What it does |
|--------|--------------|
| `isValid()` | False if default-constructed or the body was removed. |
| `getPosition()` → `Vec3` | World-space center. |
| `getRotation()` → `Quaternion` | World-space orientation (feed to `rotate()`). |
| `getSize()` → `Vec3` | Full extents of the shape's bounding box. |
| `applyForce(f)` / `applyForce(f, worldPoint)` | Accumulating push (world-space), applied over the next step. |
| `applyTorque(t)` | Accumulating spin. |
| `applyImpulse(i)` / `applyImpulse(i, worldPoint)` | One-shot kick (changes velocity instantly). |
| `applyAngularImpulse(i)` | One-shot spin kick. |
| `addVelocity(dv)` | Mass-independent kick: adds straight to the velocity (Δv). The intuitive "shove" — think in units/sec, not mass × impulse. |
| `getMass()` → `float` | Mass in sim units (`density × volume`). |
| `setLinearVelocity(v)` / `getLinearVelocity()` | Direct linear velocity. |
| `setAngularVelocity(v)` / `getAngularVelocity()` | Direct angular velocity. |
| `setPosition(p)` / `setRotation(q)` | Teleport (snaps transform, no collision sweep — for spawn/reset). |
| `setFriction(f)` / `getFriction()` | `0` = ice, `~1` = grippy. |
| `setRestitution(r)` / `getRestitution()` | `0` = dead, `1` = full bounce. |
| `activate()` / `isActive()` | Wake / query a sleeping body. |

Forces, impulses and velocity are no-ops on non-dynamic bodies, and they wake the
body for you. Example: `example-forces/` (impulse + force) and `example-bounce/`
(restitution side by side).

---

## Contact events

Subscribe to collisions with the `tc::Event`s on `PhysicsWorld`. Handlers fire on
the **main thread** — `tcxPhysics` collects Jolt's worker-thread contacts and
replays them during `update()` (or, in async mode, on the frame loop) — so it's
safe to touch app / render state from inside one.

```cpp
EventListener onHit;   // keep it alive (RAII; disconnects on destroy)

void setup() override {
    onHit = world.contactBegan.listen([](ContactEventArgs& c) {
        // c.a, c.b      : the two bodies (copyable handles)
        // c.point       : world-space contact point
        // c.normal      : world-space normal (a → b)
        // c.speed       : approach speed at impact — great for hit sfx/vfx volume
        playClick(c.speed);
    });
}
```

`contactEnded` fires when a pair stops touching (its `point`/`normal`/`speed` are
zero — only the body pair is meaningful). Example: `example-collision/`.

---

## Async stepping

By default you step the sim yourself with `update(dt)`. Alternatively, run it on
its own fixed-timestep clock so physics stays stable regardless of frame rate:

```cpp
world.updateAsyncStart(240);   // background thread stepping at 240 Hz
// ...don't call update() while async; body reads / force calls stay safe...
world.updateAsyncStop();
```

Reads and force/velocity calls are serialized against the step, so they remain
safe while the worker runs. Contact events still fire on the main thread.

**Web:** WebAssembly has no background threads, so async transparently falls back
to fixed-timestep stepping driven by the frame loop (logged once as a warning) —
same API, no code change. Example: `example-fixedTimestep/`.

---

## Advanced: raw Jolt (escape hatch)

The wrapper covers the common cases; for anything it doesn't surface yet
(constraints/joints, ray & shape casts, custom shapes, damping, …) reach straight
into Jolt:

```cpp
#include <tcxPhysics.h>
#include <tcxPhysicsJolt.h>          // opt in to raw Jolt

JPH::BodyInterface& bi = joltBodyInterface(world);
bi.SetLinearDamping(joltBodyId(myBody), 0.2f);   // a feature not wrapped
JPH::PhysicsSystem& sys = joltSystem(world);      // constraints, queries, settings…
```

Two things this costs you (both by design — it's why the default build is clean):

1. **Build.** Jolt is linked `PRIVATE` to the addon, so its headers and `JPH_*`
   compile defines don't reach your app. To use the hatch your app must link the
   same `Jolt` target — one line, ideally in a committed **`local.cmake`** so it
   survives `trusscli update`:

   ```cmake
   # local.cmake
   if(TARGET Jolt)
       target_link_libraries(${PROJECT_NAME} PRIVATE Jolt)
   endif()
   ```

   Linking the *same* target guarantees headers **and** defines match — mismatched
   `JPH_*` defines silently change struct layouts (ABI breakage).

2. **Threading.** These calls go straight to Jolt and bypass the step lock. With
   synchronous `update()` you're fine (main thread, between steps); in async mode
   call `updateAsyncStop()` first (or take your own lock).

Full working example: **`example-joltNativeAccess/`** — builds a ball-jointed
hanging chain using Jolt constraints, with the `local.cmake` shown above.

---

## Examples

| Example | Shows |
|---------|-------|
| `example-cubeRain/` | The headline demo — pour cubes into a pile. |
| `example-forces/` | `applyImpulse` / `applyForce` / `addVelocity` (click to explode, hold to levitate, V to jump). |
| `example-bounce/` | `setRestitution` / `setFriction` — a row of spheres, dead → bouncy. |
| `example-collision/` | `contactBegan` events — flash + spark + count on impact. |
| `example-fixedTimestep/` | `updateAsyncStart` — a fixed 240 Hz step keeps a tall stack solid; per-frame (capped to 30 fps) wobbles and topples. |
| `example-joltNativeAccess/` | The raw-Jolt escape hatch — a constraint-based chain. |

---

## Units & scale

Jolt is unit-agnostic, but tcxPhysics leans into a **metre / kilogram / second**
convention so the numbers feel real and you don't have to guess:

- **Gravity** defaults to the physical `-9.81` (an *acceleration* — mass-independent).
- **Mass** = `density × volume`, and `density` defaults to **1000** (water, kg/m³).
  So a `0.3 m` cube weighs ~27 kg.
- **Velocity** is units/second; a body at `v = 1` crosses one unit per second.

Practical upshot: **build at roughly metre scale** — boxes a few tenths of a unit,
camera a handful of units back — and gravity, masses and impulses all behave like
the real world. (The examples follow this; if you instead work at a "tens of units"
pixel scale, raise gravity or use a smaller `density` to compensate.)

Forces vs. velocity: `applyImpulse`/`applyForce` scale with mass (`impulse = mass × Δv`,
so use `getMass()`); `addVelocity` and `setLinearVelocity` are mass-independent and
usually the most intuitive way to move something.

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
