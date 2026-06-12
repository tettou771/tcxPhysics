#include "tcxPhysicsCharacter.h"
#include "tcxPhysicsWorld.h"

// PhysicsCharacter is a thin handle: every accessor forwards to the world,
// which owns the Jolt CharacterVirtual. No Jolt headers here.

namespace tcx {

bool PhysicsCharacter::isValid() const {
    return worldOk() && world_->hasCharacter(id_);
}

const PhysicsCharacter& PhysicsCharacter::setMoveInput(const tc::Vec3& velocity) const {
    if (worldOk()) world_->setCharacterMoveInput(id_, velocity);
    return *this;
}

const PhysicsCharacter& PhysicsCharacter::jump(float speed) const {
    if (worldOk()) world_->characterJump(id_, speed);
    return *this;
}

bool PhysicsCharacter::isGrounded() const {
    return worldOk() && world_->isCharacterGrounded(id_);
}

bool PhysicsCharacter::isOnSteepSlope() const {
    return worldOk() && world_->isCharacterOnSteepSlope(id_);
}

tc::Vec3 PhysicsCharacter::getGroundNormal() const {
    if (!worldOk()) return tc::Vec3(0.0f, 1.0f, 0.0f);
    return world_->getCharacterGroundNormal(id_);
}

tc::Vec3 PhysicsCharacter::getPosition() const {
    if (!worldOk()) return tc::Vec3();
    return world_->getCharacterPosition(id_);
}

const PhysicsCharacter& PhysicsCharacter::setPosition(const tc::Vec3& p) const {
    if (worldOk()) world_->setCharacterPosition(id_, p);
    return *this;
}

tc::Vec3 PhysicsCharacter::getLinearVelocity() const {
    if (!worldOk()) return tc::Vec3();
    return world_->getCharacterLinearVelocity(id_);
}

} // namespace tcx
