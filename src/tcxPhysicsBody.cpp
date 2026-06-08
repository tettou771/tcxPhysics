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

// --- dynamics ---------------------------------------------------------------
const PhysicsBody& PhysicsBody::applyForce(const tc::Vec3& force) const {
    if (isValid()) world_->applyForceToBody(id_, force);
    return *this;
}

const PhysicsBody& PhysicsBody::applyForce(const tc::Vec3& force, const tc::Vec3& worldPoint) const {
    if (isValid()) world_->applyForceToBody(id_, force, worldPoint);
    return *this;
}

const PhysicsBody& PhysicsBody::applyTorque(const tc::Vec3& torque) const {
    if (isValid()) world_->applyTorqueToBody(id_, torque);
    return *this;
}

const PhysicsBody& PhysicsBody::applyImpulse(const tc::Vec3& impulse) const {
    if (isValid()) world_->applyImpulseToBody(id_, impulse);
    return *this;
}

const PhysicsBody& PhysicsBody::applyImpulse(const tc::Vec3& impulse, const tc::Vec3& worldPoint) const {
    if (isValid()) world_->applyImpulseToBody(id_, impulse, worldPoint);
    return *this;
}

const PhysicsBody& PhysicsBody::applyAngularImpulse(const tc::Vec3& angularImpulse) const {
    if (isValid()) world_->applyAngularImpulseToBody(id_, angularImpulse);
    return *this;
}

const PhysicsBody& PhysicsBody::setLinearVelocity(const tc::Vec3& v) const {
    if (isValid()) world_->setBodyLinearVelocity(id_, v);
    return *this;
}

tc::Vec3 PhysicsBody::getLinearVelocity() const {
    if (!isValid()) return tc::Vec3();
    return world_->getBodyLinearVelocity(id_);
}

const PhysicsBody& PhysicsBody::setAngularVelocity(const tc::Vec3& v) const {
    if (isValid()) world_->setBodyAngularVelocity(id_, v);
    return *this;
}

tc::Vec3 PhysicsBody::getAngularVelocity() const {
    if (!isValid()) return tc::Vec3();
    return world_->getBodyAngularVelocity(id_);
}

const PhysicsBody& PhysicsBody::setPosition(const tc::Vec3& p) const {
    if (isValid()) world_->setBodyPosition(id_, p);
    return *this;
}

const PhysicsBody& PhysicsBody::setRotation(const tc::Quaternion& q) const {
    if (isValid()) world_->setBodyRotation(id_, q);
    return *this;
}

// --- material ---------------------------------------------------------------
const PhysicsBody& PhysicsBody::setFriction(float friction) const {
    if (isValid()) world_->setBodyFriction(id_, friction);
    return *this;
}

float PhysicsBody::getFriction() const {
    if (!isValid()) return 0.0f;
    return world_->getBodyFriction(id_);
}

const PhysicsBody& PhysicsBody::setRestitution(float restitution) const {
    if (isValid()) world_->setBodyRestitution(id_, restitution);
    return *this;
}

float PhysicsBody::getRestitution() const {
    if (!isValid()) return 0.0f;
    return world_->getBodyRestitution(id_);
}

// --- activation -------------------------------------------------------------
const PhysicsBody& PhysicsBody::activate() const {
    if (isValid()) world_->activateBody(id_);
    return *this;
}

bool PhysicsBody::isActive() const {
    if (!isValid()) return false;
    return world_->isBodyActive(id_);
}

} // namespace tcx
