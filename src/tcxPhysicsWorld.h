#pragma once

#include <TrussC.h>
#include <memory>
#include <cstdint>
#include "tcxPhysicsBody.h"

namespace tcx {

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
    void update(float dt = 1.0f / 60.0f, int collisionSteps = 1);

    // --- body factory --------------------------------------------------------
    // dynamic = true  -> falls and collides (a thrown block)
    // dynamic = false -> never moves (floor, walls, static scenery)
    // size is the FULL box size (w, h, d); position is its center.
    PhysicsBody addBox(const tc::Vec3& position, const tc::Vec3& size, bool dynamic = true);
    PhysicsBody addSphere(const tc::Vec3& position, float radius, bool dynamic = true);
    // A large flat static box acting as the ground, centered on (0, y, 0).
    PhysicsBody addGroundPlane(float y = 0.0f, float size = 100000.0f);

    void removeBody(const PhysicsBody& body);
    // Remove every dynamic body but keep static scenery (floor/walls).
    void clearDynamicBodies();
    int getNumBodies() const;

    // --- queried by PhysicsBody (you rarely call these directly) -------------
    tc::Vec3 getBodyPosition(uint32_t id) const;
    tc::Quaternion getBodyRotation(uint32_t id) const;
    tc::Vec3 getBodySize(uint32_t id) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace tcx
