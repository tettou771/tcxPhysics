#pragma once

#include <TrussC.h>
#include <cstdint>

namespace tcx {

class PhysicsWorld;
enum class MotionType;   // defined in tcxPhysicsWorld.h (Static / Kinematic / Dynamic)

// =============================================================================
// PhysicsBody - a lightweight handle to one rigid body inside a PhysicsWorld.
//
// It owns no memory: it's just (world pointer + Jolt body id). Copy it freely,
// store it in a vector, etc. The actual body lives in the PhysicsWorld and dies
// when the world is cleared/destroyed — a handle to a removed body simply
// reports isValid() == false on use.
// =============================================================================
class PhysicsBody {
public:
    static constexpr uint32_t kInvalidId = 0xffffffffu;

    PhysicsBody() = default;
    PhysicsBody(PhysicsWorld* world, uint32_t id) : world_(world), id_(id) {}

    bool isValid() const { return world_ != nullptr && id_ != kInvalidId; }

    // Current world-space transform, read back from the simulation.
    tc::Vec3 getPosition() const;
    tc::Quaternion getRotation() const;
    // Full extents (width, height, depth) of the body's shape bounding box —
    // handy for feeding straight into a draw call.
    tc::Vec3 getSize() const;

    // The underlying Jolt body id (index+sequence packed into a uint32).
    uint32_t getId() const { return id_; }

    // --- dynamics (only meaningful for dynamic bodies) -----------------------
    // World-space. Forces accumulate and apply over the next step; impulses
    // change velocity immediately. The body is auto-woken. All chainable.
    const PhysicsBody& applyForce(const tc::Vec3& force) const;
    const PhysicsBody& applyForce(const tc::Vec3& force, const tc::Vec3& worldPoint) const;
    const PhysicsBody& applyTorque(const tc::Vec3& torque) const;
    const PhysicsBody& applyImpulse(const tc::Vec3& impulse) const;
    const PhysicsBody& applyImpulse(const tc::Vec3& impulse, const tc::Vec3& worldPoint) const;
    const PhysicsBody& applyAngularImpulse(const tc::Vec3& angularImpulse) const;

    // Mass-independent kick: directly adds to the velocity (Δv), ignoring mass.
    // Often the most intuitive way to "shove" something — you think in units/sec,
    // not in mass * impulse. Same effect on a heavy and a light body.
    const PhysicsBody& addVelocity(const tc::Vec3& dv) const;

    // Mass in the sim's units (= density * volume; default density 1000).
    float getMass() const;

    const PhysicsBody& setLinearVelocity(const tc::Vec3& v) const;
    tc::Vec3 getLinearVelocity() const;
    const PhysicsBody& setAngularVelocity(const tc::Vec3& v) const;
    tc::Vec3 getAngularVelocity() const;

    // Teleport — snaps the transform, bypassing the collision sweep. Use for
    // spawning / resetting, not for driving motion (use velocity for that).
    const PhysicsBody& setPosition(const tc::Vec3& p) const;
    const PhysicsBody& setRotation(const tc::Quaternion& q) const;

    // --- motion type ---------------------------------------------------------
    // Switch between static / kinematic / dynamic at runtime.
    const PhysicsBody& setMotionType(MotionType type) const;
    // Drive a KINEMATIC body toward (pos, rot) over dt so it pushes dynamics with
    // the right momentum (call every frame with the frame dt). See PhysicsWorld.
    const PhysicsBody& moveKinematic(const tc::Vec3& pos, const tc::Quaternion& rot, float dt) const;

    // --- sensor (trigger) ----------------------------------------------------
    // A sensor reports overlaps (via contact events) but never blocks motion —
    // bodies pass through it. Toggle it on a normal body to make a trigger volume.
    const PhysicsBody& setSensor(bool sensor) const;
    bool isSensor() const;

    // --- material ------------------------------------------------------------
    const PhysicsBody& setFriction(float friction) const;        // 0 = ice, ~1 = grippy
    float getFriction() const;
    const PhysicsBody& setRestitution(float restitution) const;  // 0 = dead, 1 = full bounce
    float getRestitution() const;

    // --- activation ----------------------------------------------------------
    const PhysicsBody& activate() const;   // wake a sleeping body
    bool isActive() const;

private:
    PhysicsWorld* world_ = nullptr;
    uint32_t id_ = kInvalidId;
};

} // namespace tcx
