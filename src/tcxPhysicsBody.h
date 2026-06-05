#pragma once

#include <TrussC.h>
#include <cstdint>

namespace tcx {

class PhysicsWorld;

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

private:
    PhysicsWorld* world_ = nullptr;
    uint32_t id_ = kInvalidId;
};

} // namespace tcx
