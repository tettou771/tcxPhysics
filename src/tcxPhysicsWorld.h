#pragma once

#include <TrussC.h>
#include <memory>
#include <cstdint>
#include "tcxPhysicsBody.h"

namespace tcx {

// =============================================================================
// ContactEventArgs - payload for PhysicsWorld::contactBegan / contactEnded.
//
// A value snapshot built on the MAIN thread right after the step (Jolt detects
// contacts on worker threads; we buffer them and replay them safely here). The
// two bodies are copyable handles, so a listener may freely read transforms,
// apply impulses, queue removals, etc.
// =============================================================================
struct ContactEventArgs {
    PhysicsBody a;        // the two bodies in contact (handles; always safe to copy)
    PhysicsBody b;
    tc::Vec3 point;       // world-space contact point (set on contactBegan; zero on ended)
    tc::Vec3 normal;      // world-space contact normal, from a toward b (began only)
    float speed = 0.0f;   // relative approach speed along the normal at impact
                          // (began only) — handy for impact sound / vfx intensity
};

// =============================================================================
// PhysicsWorld - owns the Jolt simulation and acts as a factory for bodies.
//
// Lifecycle:
//   setup()  once (e.g. in App::setup)
//   update() once per frame (e.g. in App::update)
//   addBox / addSphere / addGroundPlane to populate it
//
// Units are whatever your scene uses — Jolt itself is unit-agnostic. The default
// gravity is the physical -9.81 on Y; for small scenes you'll usually want a
// punchier value via setGravity() so things fall at a lively pace.
// =============================================================================
class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();
    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    // Initialize the simulation. Call once before adding bodies.
    //
    //   maxBodies              - cap on rigid bodies alive at once.
    //   maxBodyPairs           - broadphase collision-pair buffer (0 = auto).
    //   maxContactConstraints  - contact-constraint buffer (0 = auto).
    //
    // The two buffers default to generous values scaled from maxBodies. They
    // matter for dense piles: if either fills up, Jolt drops contacts and bodies
    // tunnel through each other and the floor. Raise them if you see that with
    // very large/dense scenes.
    void setup(int maxBodies = 10240, int maxBodyPairs = 0, int maxContactConstraints = 0);

    PhysicsWorld& setGravity(const tc::Vec3& gravity);
    tc::Vec3 getGravity() const;

    // Advance the simulation by dt seconds. collisionSteps subdivides the step
    // for stability at large dt (1 is fine for ~60fps).
    //
    // Use this OR the async stepping below — not both. While async is running,
    // update() is ignored (with a warning).
    void update(float dt = 1.0f / 60.0f, int collisionSteps = 1);

    // --- stepping mode -------------------------------------------------------
    // By default you drive the sim yourself with update(dt) once per frame.
    //
    // Alternatively, run the simulation on its own fixed-timestep clock so the
    // physics stays stable independent of frame rate / hitches:
    //   world.updateAsyncStart(120);   // step at 120 Hz on a background thread
    //   ...
    //   world.updateAsyncStop();
    // Body reads and force/velocity calls stay safe while async runs — they are
    // serialized against the step. Collision events still fire on the MAIN
    // thread (drained on the frame loop), so handlers can touch app state.
    //
    // Web (wasm) has no background threads: async there transparently falls back
    // to fixed-timestep stepping driven by the frame loop (events().update),
    // logged once as a warning. Same API, no code change needed.
    void updateAsyncStart(float hz = 120.0f);
    void updateAsyncStop();
    bool isAsync() const;

    // --- collision events ----------------------------------------------------
    // Fired on the MAIN thread (inside update(), or — in async mode — on the
    // frame's events().update) so handlers can safely touch app / render state
    // even though Jolt detects contacts on worker threads.
    //
    //   listener = world.contactBegan.listen([](ContactEventArgs& c){ ... });
    tc::Event<ContactEventArgs> contactBegan;  // two bodies started touching
    tc::Event<ContactEventArgs> contactEnded;  // two bodies stopped touching

    // --- body factory --------------------------------------------------------
    // dynamic = true  -> falls and collides (a thrown block)
    // dynamic = false -> never moves (floor, walls, static scenery)
    // size is the FULL box size (w, h, d); position is its center.
    //
    // mass = density * volume. The default density is 1000 (water, kg/m^3) — so
    // build at roughly METRE scale and the numbers feel real: a 0.3 m cube weighs
    // ~27 kg, default gravity -9.81 looks natural. (Jolt is unit-agnostic; 1000
    // is its native default. See the README on scale.)
    PhysicsBody addBox(const tc::Vec3& position, const tc::Vec3& size, bool dynamic = true, float density = 1000.0f);
    PhysicsBody addSphere(const tc::Vec3& position, float radius, bool dynamic = true, float density = 1000.0f);

    // A capsule aligned to the Y axis: a cylinder of `cylinderHeight` capped by
    // two hemispheres of `radius` (total height = cylinderHeight + 2*radius).
    // Draw it with tc::createCapsule(radius, cylinderHeight). Great for characters.
    PhysicsBody addCapsule(const tc::Vec3& position, float radius, float cylinderHeight, bool dynamic = true, float density = 1000.0f);
    // A cylinder aligned to the Y axis. Draw with tc::createCylinder(radius, height).
    PhysicsBody addCylinder(const tc::Vec3& position, float radius, float height, bool dynamic = true, float density = 1000.0f);

    // A convex body built as the convex hull of a mesh's vertices. Use for
    // arbitrary *convex* dynamic shapes (gems, dice, rocks). The mesh is only
    // sampled for its points — keep your own copy to draw. Concave detail is lost
    // (it's the hull); for concave geometry use addMesh (static).
    PhysicsBody addConvexHull(const tc::Vec3& position, const tc::Mesh& mesh, bool dynamic = true, float density = 1000.0f);

    // A triangle-mesh body for arbitrary (incl. concave) geometry — terrain,
    // level scenery. STATIC ONLY: Jolt can't make a mesh shape dynamic (no mass
    // properties), so `dynamic` is ignored with a warning if true. Draw the same
    // mesh yourself at the body's transform.
    PhysicsBody addMesh(const tc::Vec3& position, const tc::Mesh& mesh, bool dynamic = false);

    // A large flat static box acting as the ground, centered on (0, y, 0).
    PhysicsBody addGroundPlane(float y = 0.0f, float size = 100000.0f);

    void removeBody(const PhysicsBody& body);
    // Remove every dynamic body but keep static scenery (floor/walls).
    void clearDynamicBodies();
    int getNumBodies() const;

    // --- queried / driven by PhysicsBody (you rarely call these directly) ----
    // PhysicsBody forwards to these; all are serialized against the step so they
    // stay safe while async stepping runs.
    tc::Vec3 getBodyPosition(uint32_t id) const;
    tc::Quaternion getBodyRotation(uint32_t id) const;
    tc::Vec3 getBodySize(uint32_t id) const;

    void applyForceToBody(uint32_t id, const tc::Vec3& force);
    void applyForceToBody(uint32_t id, const tc::Vec3& force, const tc::Vec3& worldPoint);
    void applyTorqueToBody(uint32_t id, const tc::Vec3& torque);
    void applyImpulseToBody(uint32_t id, const tc::Vec3& impulse);
    void applyImpulseToBody(uint32_t id, const tc::Vec3& impulse, const tc::Vec3& worldPoint);
    void applyAngularImpulseToBody(uint32_t id, const tc::Vec3& angularImpulse);
    void addVelocityToBody(uint32_t id, const tc::Vec3& dv);

    float getBodyMass(uint32_t id) const;

    void setBodyLinearVelocity(uint32_t id, const tc::Vec3& v);
    tc::Vec3 getBodyLinearVelocity(uint32_t id) const;
    void setBodyAngularVelocity(uint32_t id, const tc::Vec3& v);
    tc::Vec3 getBodyAngularVelocity(uint32_t id) const;

    void setBodyPosition(uint32_t id, const tc::Vec3& p);
    void setBodyRotation(uint32_t id, const tc::Quaternion& q);

    void setBodyFriction(uint32_t id, float friction);
    float getBodyFriction(uint32_t id) const;
    void setBodyRestitution(uint32_t id, float restitution);
    float getBodyRestitution(uint32_t id) const;

    void activateBody(uint32_t id);
    bool isBodyActive(uint32_t id) const;

    // --- escape hatch (advanced) ---------------------------------------------
    // Raw pointers to the underlying Jolt objects, as void* so this header stays
    // Jolt-free. Don't cast these by hand — include <tcxPhysicsJolt.h> for typed
    // accessors (joltSystem(world) / joltBodyInterface(world)). Both are null
    // before setup(). Direct Jolt calls BYPASS the step lock: in async mode,
    // updateAsyncStop() first (or do your own locking).
    void* nativeSystem() const;          // -> JPH::PhysicsSystem*
    void* nativeBodyInterface() const;   // -> JPH::BodyInterface*

private:
    // Drain worker-collected contacts and fire contactBegan/Ended (main thread).
    void dispatchContacts();
#ifdef __EMSCRIPTEN__
    void webAsyncTick();   // fixed-timestep step driven by the frame loop
#else
    void asyncLoop();      // background-thread fixed-timestep loop
#endif

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace tcx
