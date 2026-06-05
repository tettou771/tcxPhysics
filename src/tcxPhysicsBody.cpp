#include "tcxPhysicsBody.h"
#include "tcxPhysicsWorld.h"

// PhysicsBody is a thin handle: every accessor just forwards to the world, which
// owns the Jolt simulation. No Jolt headers needed here.

namespace tcx {

tc::Vec3 PhysicsBody::getPosition() const {
    if (!isValid()) return tc::Vec3();
    return world_->getBodyPosition(id_);
}

tc::Quaternion PhysicsBody::getRotation() const {
    if (!isValid()) return tc::Quaternion::identity();
    return world_->getBodyRotation(id_);
}

tc::Vec3 PhysicsBody::getSize() const {
    if (!isValid()) return tc::Vec3();
    return world_->getBodySize(id_);
}

} // namespace tcx
