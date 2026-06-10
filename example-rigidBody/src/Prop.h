#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace tc;
using namespace tcx;

// Prop - a self-contained physics object.
//
// A plain Node that, in its own setup(), gives ITSELF a RigidBody + a
// ColliderRenderer and listens to its OWN collisions: white flash on impact
// (onCollisionBegan), warm glow while resting (onCollisionStay). All the physics
// wiring and reaction lives here on the object — the app just spawns Props.
//
// No world is passed: RigidBody uses the default physics world (the app sets it
// up and steps it). That's the point of the Mod approach — behaviour travels with
// the object, not piled into the App.
class Prop : public Node {
public:
    Prop(const Color& base, const Vec3& pos) : base_(base), startPos_(pos) {}

    void setup() override {
        setPos(startPos_);                         // place before the body is created
        auto* rb = addMod<RigidBody>(ColliderShape::box(Vec3(0.28f)));
        rb->setRestitution(0.3f);
        renderer_ = addMod<ColliderRenderer>();
        renderer_->setColor(base_);

        // React to our own collisions — pass member functions (this + &Prop::fn).
        // Tokens kept alive as members.
        beganL_ = rb->onCollisionBegan.listen(this, &Prop::onCollisionBegan);
        stayL_  = rb->onCollisionStay.listen (this, &Prop::onCollisionStay);
        endedL_ = rb->onCollisionEnded.listen(this, &Prop::onCollisionEnded);
    }

    void update() override {
        float dt = (float)getDeltaTime();
        flash_ = std::max(0.0f, flash_ - dt * 4.0f);   // impact pop fades fast
        rest_  = std::max(0.0f, rest_  - dt * 6.0f);   // glow drops once Stay stops
        renderer_->setColor(Color(
            clamp(base_.r + (1.0f - base_.r) * flash_ + 0.45f * rest_, 0.0f, 1.0f),  // white + warm
            clamp(base_.g + (1.0f - base_.g) * flash_ + 0.12f * rest_, 0.0f, 1.0f),
            clamp(base_.b + (1.0f - base_.b) * flash_, 0.0f, 1.0f)));
    }

    // Collision handlers (plain methods — Node has no such virtual, so no override).
    void onCollisionBegan(Collision& collision) { flash_ = 1.0f; }
    void onCollisionStay (Collision& collision) { rest_  = 1.0f; }
    void onCollisionEnded(Collision& collision) {}

private:
    Color             base_;
    Vec3              startPos_;
    ColliderRenderer* renderer_ = nullptr;
    EventListener     beganL_, stayL_, endedL_;
    float             flash_ = 0.0f, rest_ = 0.0f;
};
