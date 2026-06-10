#include "tcxPhysicsJoint.h"
#include "tcxPhysicsWorld.h"

// PhysicsJoint is a thin handle: every accessor forwards to the world, which
// owns the joint registry and the Jolt constraints. No Jolt headers here.

namespace tcx {

bool PhysicsJoint::isValid() const {
    return worldOk() && world_->hasJoint(id_);
}

JointType PhysicsJoint::getType() const {
    if (!worldOk()) return JointType::Point;
    return world_->getJointType(id_);
}

PhysicsBody PhysicsJoint::getBodyA() const {
    if (!worldOk()) return PhysicsBody();
    uint32_t id = world_->getJointBodyA(id_);
    return (id == PhysicsBody::kInvalidId) ? PhysicsBody() : PhysicsBody(world_, id);
}

PhysicsBody PhysicsJoint::getBodyB() const {
    if (!worldOk()) return PhysicsBody();
    uint32_t id = world_->getJointBodyB(id_);
    return (id == PhysicsBody::kInvalidId) ? PhysicsBody() : PhysicsBody(world_, id);
}

tc::Vec3 PhysicsJoint::getAnchorA() const {
    if (!worldOk()) return tc::Vec3();
    return world_->getJointAnchorA(id_);
}

tc::Vec3 PhysicsJoint::getAnchorB() const {
    if (!worldOk()) return tc::Vec3();
    return world_->getJointAnchorB(id_);
}

tc::Vec3 PhysicsJoint::getAxis() const {
    if (!worldOk()) return tc::Vec3(0.0f, 1.0f, 0.0f);
    return world_->getJointAxis(id_);
}

void PhysicsJoint::remove() {
    if (worldOk()) world_->removeJointById(id_);
}

void PhysicsJoint::drawWire(const tc::Color& color) const {
    if (!isValid()) return;
    tc::Vec3 a = getAnchorA();
    tc::Vec3 b = getAnchorB();
    tc::setColor(color);
    tc::drawLine(a, b);
    tc::drawBox(a, 0.03f);
    tc::drawBox(b, 0.03f);
    JointType t = getType();
    if (t == JointType::Hinge || t == JointType::Slider) {
        tc::Vec3 ax = getAxis();
        tc::drawLine(a - ax * 0.15f, a + ax * 0.15f);
    }
}

} // namespace tcx
