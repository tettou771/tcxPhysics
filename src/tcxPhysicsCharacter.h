#pragma once

#include <TrussC.h>
#include <cstdint>
#include <memory>

namespace tcx {

class PhysicsWorld;

// =============================================================================
// PhysicsCharacter - a walking-character controller (Jolt CharacterVirtual).
//
// NOT a rigid body: it's a virtual capsule you steer with a desired velocity,
// and the world resolves the rest the way a game expects — climbing slopes up
// to the limit, stepping up stairs, sliding along walls, riding moving
// platforms, pushing dynamic bodies. Created by PhysicsWorld::addCharacter();
// this is the usual lightweight handle (world + id, nothing owned).
//
//   character.setMoveInput(dir * 4.0f);            // every frame, m/s, horizontal
//   if (jumpPressed && character.isGrounded()) character.jump(5.0f);
//
// The world advances every character inside update() (or on the frame loop in
// async mode) — there is nothing extra to call.
// =============================================================================
class PhysicsCharacter {
public:
    PhysicsCharacter() = default;
    PhysicsCharacter(PhysicsWorld* world, uint64_t id, std::weak_ptr<int> worldAlive)
        : world_(world), id_(id), worldAlive_(std::move(worldAlive)) {}

    bool isValid() const;

    // --- steering ------------------------------------------------------------
    // Desired HORIZONTAL velocity (m/s), world space — set it every frame
    // (sticks: stick * speed; WASD: normalized key direction * speed).
    // The Y component is ignored; gravity and jumping handle the vertical.
    const PhysicsCharacter& setMoveInput(const tc::Vec3& velocity) const;
    // One-shot upward kick, applied at the next update IF grounded.
    const PhysicsCharacter& jump(float speed) const;

    // --- state ---------------------------------------------------------------
    bool isGrounded() const;          // standing on walkable ground
    bool isOnSteepSlope() const;      // touching ground too steep to climb
    tc::Vec3 getGroundNormal() const;
    tc::Vec3 getPosition() const;     // capsule center, world space
    const PhysicsCharacter& setPosition(const tc::Vec3& p) const;   // teleport
    tc::Vec3 getLinearVelocity() const;

    uint64_t getId() const { return id_; }

private:
    bool worldOk() const { return world_ != nullptr && !worldAlive_.expired(); }

    PhysicsWorld* world_ = nullptr;
    uint64_t id_ = 0;
    std::weak_ptr<int> worldAlive_;
};

} // namespace tcx
