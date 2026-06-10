#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace tc;
using namespace tcx;

// Faller - a dynamic cube that reacts to passing through a trigger.
//
// It listens to ITS OWN onTrigger* events: while overlapping a trigger it flares
// to white, then fades back to its base color once it exits. Because the gate is
// a sensor, the cube is never deflected — it simply falls through and lights up.
class Faller : public Node {
public:
    Faller(const Color& base, const Vec3& pos) : base_(base) { setPos(pos); }

    void setup() override {
        addMod<RigidBody>(ColliderShape::box(Vec3(0.3f)));
        renderer_ = addMod<ColliderRenderer>();
        renderer_->setColor(base_);

        auto* rb = getMod<RigidBody>();
        enterL_ = rb->onTriggerBegan.listen(this, &Faller::onEnter);
        exitL_  = rb->onTriggerEnded.listen(this, &Faller::onExit);
    }

    void update() override {
        // Ease the flare back down once we've left the trigger.
        if (!inside_) glow_ = std::max(0.0f, glow_ - (float)getDeltaTime() * 3.0f);
        renderer_->setColor(Color(
            clamp(base_.r + (1.0f - base_.r) * glow_, 0.0f, 1.0f),
            clamp(base_.g + (1.0f - base_.g) * glow_, 0.0f, 1.0f),
            clamp(base_.b + (1.0f - base_.b) * glow_, 0.0f, 1.0f)));
    }

private:
    void onEnter(Collision& c) { inside_ = true;  glow_ = 1.0f; }
    void onExit (Collision& c) { inside_ = false; }

    Color             base_;
    ColliderRenderer* renderer_ = nullptr;
    EventListener     enterL_, exitL_;
    bool              inside_ = false;
    float             glow_ = 0.0f;
};
