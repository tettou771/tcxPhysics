#pragma once

#include <TrussC.h>
#include <cstdint>
#include <memory>
#include "tcxPhysicsBody.h"

namespace tcx {

class PhysicsWorld;

// =============================================================================
// Joints - constrain two bodies (or a body and the world) to each other.
//
//   Joint        - a declarative description: which kind, where, with what
//                  options. Built with named factories + chainable setters:
//                      Joint::hinge(pivot, axis).limits(-TAU/4, TAU/4)
//   PhysicsJoint - a lightweight handle to a LIVE joint inside a PhysicsWorld
//                  (world pointer + id, like PhysicsBody). Copy it freely; a
//                  handle to a removed joint just reports isValid() == false.
//
// The world OWNS every joint. Handles never cache state — each accessor asks
// the world, so there is no lifetime to manage on the caller's side.
// =============================================================================

enum class JointType { Point, Hinge, Slider, Distance, Fixed, Cone, SwingTwist, Gear, RackAndPinion, SixDof };

// What to build. All positions / axes are WORLD-space at creation time —
// place your bodies first, then describe the joint where it should bite.
struct Joint {
    JointType type = JointType::Point;
    tc::Vec3 anchorA;                    // pivot / attachment on body A
    tc::Vec3 anchorB;                    // attachment on body B (distance only)
    tc::Vec3 axis{0.0f, 1.0f, 0.0f};     // hinge / slider axis
    float limitMin = 0.0f, limitMax = 0.0f;        // hinge: rad, slider: m
    bool  hasLimits = false;
    float minDist = -1.0f, maxDist = -1.0f;        // distance (-1 = current)
    float springHz = 0.0f, springDamping = 0.0f;   // distance spring
    bool  hasSpring = false;
    float motorVelocity = 0.0f, motorMaxForce = -1.0f;  // hinge / slider motor
    bool  hasMotor = false;
    float coneAngle = 0.0f;                             // cone / swingTwist swing (rad, half-angle)
    float twistMin = 0.0f, twistMax = 0.0f;             // swingTwist twist range (rad)
    tc::Vec3 sixTransMin, sixTransMax;                  // sixDof per-axis travel (m)
    tc::Vec3 sixRotMin, sixRotMax;                      // sixDof per-axis rotation (rad)
    bool sixTransFree = false, sixRotFree = false;      // sixDof: fully open groups
    float breakForceN = -1.0f, breakTorqueNm = -1.0f;   // breakable (-1 = never)

    // A ball joint: pins the two bodies together at one world point. Chains,
    // ragdoll joints, pendulums.
    static Joint point(const tc::Vec3& worldPivot) {
        Joint j; j.type = JointType::Point;
        j.anchorA = j.anchorB = worldPivot; return j;
    }
    // Rotation around one axis through a world pivot. Doors, wheels, levers.
    static Joint hinge(const tc::Vec3& worldPivot, const tc::Vec3& axis) {
        Joint j; j.type = JointType::Hinge;
        j.anchorA = j.anchorB = worldPivot; j.axis = axis; return j;
    }
    // Straight-line travel along an axis (no rotation). Pistons, drawers.
    // Attachment points are taken from the bodies' poses at creation.
    static Joint slider(const tc::Vec3& axis) {
        Joint j; j.type = JointType::Slider; j.axis = axis; return j;
    }
    // Keeps a point on A and a point on B at a distance. A rope by default
    // (range = the creation distance); see range() / spring().
    static Joint distance(const tc::Vec3& worldAnchorOnA, const tc::Vec3& worldAnchorOnB) {
        Joint j; j.type = JointType::Distance;
        j.anchorA = worldAnchorOnA; j.anchorB = worldAnchorOnB; return j;
    }
    // Welds the two bodies in their current relative pose (no relative motion).
    static Joint fixed() {
        Joint j; j.type = JointType::Fixed; return j;
    }
    // A ball joint whose swing is capped to a cone of halfAngle around `axis`.
    // Like point(), but the body can't fold back on itself.
    static Joint cone(const tc::Vec3& worldPivot, const tc::Vec3& axis, float halfAngle) {
        Joint j; j.type = JointType::Cone;
        j.anchorA = j.anchorB = worldPivot; j.axis = axis; j.coneAngle = halfAngle; return j;
    }
    // The ragdoll joint: swing capped to a cone around `twistAxis` PLUS a limit
    // on the twist around it. Shoulders, hips, necks. See swing() / twist().
    static Joint swingTwist(const tc::Vec3& worldPivot, const tc::Vec3& twistAxis) {
        Joint j; j.type = JointType::SwingTwist;
        j.anchorA = j.anchorB = worldPivot; j.axis = twistAxis; return j;
    }
    // The generic 6-degrees-of-freedom joint. Starts as a WELD (all six axes
    // fixed) — open exactly the freedom you want with translation() /
    // rotation() / freeTranslation() / freeRotation(). Axes are world X/Y/Z.
    //   Joint::sixDof(p).translation({0,-0.1f,0}, {0,0.1f,0})   // bouncy mount
    //                   .rotation({-0.2f,0,-0.2f}, {0.2f,0,0.2f})
    static Joint sixDof(const tc::Vec3& worldPivot) {
        Joint j; j.type = JointType::SixDof;
        j.anchorA = j.anchorB = worldPivot; return j;
    }

    // --- chainable options ---------------------------------------------------
    // hinge: angle range in radians (min in [-pi,0], max in [0,pi]).
    // slider: travel range in metres (min <= 0 <= max).
    Joint& limits(float min, float max) { limitMin = min; limitMax = max; hasLimits = true; return *this; }
    // distance: keep the points between min and max apart (rigid rod: min == max).
    Joint& range(float min, float max) { minDist = min; maxDist = max; return *this; }
    // distance: make the limits springy instead of hard (frequency Hz, damping 0..1).
    Joint& spring(float hz, float damping) { springHz = hz; springDamping = damping; hasSpring = true; return *this; }
    // hinge / slider: drive at a constant velocity from the start (rad/s or m/s).
    // maxForce caps the motor's torque (N·m) / force (N); -1 = unlimited.
    // For position-mode driving use PhysicsJoint::setMotorTarget() at runtime.
    Joint& motor(float velocity, float maxForce = -1.0f) { motorVelocity = velocity; motorMaxForce = maxForce; hasMotor = true; return *this; }
    // swingTwist: half-angle of the swing cone around the twist axis (rad).
    Joint& swing(float halfAngle) { coneAngle = halfAngle; return *this; }
    // swingTwist: allowed twist range around the twist axis (rad, in [-pi, pi]).
    Joint& twist(float min, float max) { twistMin = min; twistMax = max; return *this; }
    // sixDof: per-axis travel range in metres (an axis with min == max == 0
    // stays fixed). World X/Y/Z components.
    Joint& translation(const tc::Vec3& min, const tc::Vec3& max) { sixTransMin = min; sixTransMax = max; return *this; }
    // sixDof: per-axis rotation range in radians (0,0 = fixed axis).
    Joint& rotation(const tc::Vec3& min, const tc::Vec3& max) { sixRotMin = min; sixRotMax = max; return *this; }
    // sixDof: open a whole group completely.
    Joint& freeTranslation() { sixTransFree = true; return *this; }
    Joint& freeRotation()    { sixRotFree = true;  return *this; }
    // Make the joint BREAKABLE: it removes itself when the force it transmits
    // exceeds `newtons` (checked after every step; PhysicsWorld::jointBroke
    // fires so you can react). Rough feel: holding 1 kg still ≈ 10 N.
    Joint& breakForce(float newtons) { breakForceN = newtons; return *this; }
    // Same for torque (N·m) — lets a weld snap when twisted (fixed/hinge...).
    Joint& breakTorque(float newtonMetres) { breakTorqueNm = newtonMetres; return *this; }
};

// Fired by PhysicsWorld::jointBroke when a breakable joint snaps. The joint is
// already removed (handles to it report isValid() == false); this tells you
// where and how hard, e.g. to spawn debris or play a crack.
struct JointBreakEventArgs {
    uint64_t jointId = 0;
    JointType type = JointType::Point;
    PhysicsBody bodyA, bodyB;     // the sides (world side = invalid handle)
    tc::Vec3 point;               // world-space anchor where it snapped
    float force = 0.0f;           // transmitted force at the break (N)
    float torque = 0.0f;          // transmitted torque at the break (N·m)
};

// A handle to one live joint. Owns nothing; all accessors query the world.
class PhysicsJoint {
public:
    PhysicsJoint() = default;
    PhysicsJoint(PhysicsWorld* world, uint64_t id, std::weak_ptr<int> worldAlive)
        : world_(world), id_(id), worldAlive_(std::move(worldAlive)) {}

    // False once the joint was removed (explicitly, or because one of its
    // bodies / the world went away).
    bool isValid() const;

    JointType getType() const;
    // A is the BASE, B the side that moves positively (see PhysicsWorld::addJoint).
    // The side that is "the world" reports an invalid handle.
    PhysicsBody getBodyA() const;
    PhysicsBody getBodyB() const;
    // Current WORLD-space attachment points (they move with the bodies).
    tc::Vec3 getAnchorA() const;
    tc::Vec3 getAnchorB() const;
    tc::Vec3 getAxis() const;            // current world axis (hinge / slider)

    uint64_t getId() const { return id_; }

    // --- motor (hinge / slider only; warns on other types) -------------------
    // Drive at a velocity: rad/s (hinge) or m/s (slider). Wakes the bodies.
    const PhysicsJoint& setMotorVelocity(float velocity, float maxForce = -1.0f) const;
    // Drive toward a target: angle in rad (hinge) or position in m (slider).
    const PhysicsJoint& setMotorTarget(float target, float maxForce = -1.0f) const;
    // Cut the drive (the joint itself stays).
    const PhysicsJoint& setMotorOff() const;

    // Remove this joint from its world (the bodies stay).
    void remove();

    // Debug-draw: a line between the anchors with end markers, plus the axis
    // for hinge / slider. Call between your camera begin()/end().
    void drawWire(const tc::Color& color = tc::Color(0.3f, 1.0f, 0.5f)) const;

private:
    bool worldOk() const { return world_ != nullptr && !worldAlive_.expired(); }

    PhysicsWorld* world_ = nullptr;
    uint64_t id_ = 0;
    std::weak_ptr<int> worldAlive_;
};

} // namespace tcx
