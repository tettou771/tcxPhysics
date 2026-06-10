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

enum class JointType { Point, Hinge, Slider, Distance, Fixed };

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

    // --- chainable options ---------------------------------------------------
    // hinge: angle range in radians (min in [-pi,0], max in [0,pi]).
    // slider: travel range in metres (min <= 0 <= max).
    Joint& limits(float min, float max) { limitMin = min; limitMax = max; hasLimits = true; return *this; }
    // distance: keep the points between min and max apart (rigid rod: min == max).
    Joint& range(float min, float max) { minDist = min; maxDist = max; return *this; }
    // distance: make the limits springy instead of hard (frequency Hz, damping 0..1).
    Joint& spring(float hz, float damping) { springHz = hz; springDamping = damping; hasSpring = true; return *this; }
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
    PhysicsBody getBodyA() const;
    PhysicsBody getBodyB() const;        // invalid handle if jointed to the world
    // Current WORLD-space attachment points (they move with the bodies).
    tc::Vec3 getAnchorA() const;
    tc::Vec3 getAnchorB() const;
    tc::Vec3 getAxis() const;            // current world axis (hinge / slider)

    uint64_t getId() const { return id_; }

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
