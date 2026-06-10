#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace tc;
using namespace tcx;

// Pickable - a body the mouse ray can select.
//
// Nothing physics-special here: it's just a RigidBody + ColliderRenderer. The
// raycasting lives in the app (PhysicsWorld::raycast). The app brightens whichever
// Pickable the ray currently hits via setHovered().
class Pickable : public Node {
public:
    Pickable(ColliderShape shape, const Color& base, const Vec3& pos, BodyType type = BodyType::Dynamic)
        : shape_(shape), base_(base), type_(type) { setPos(pos); }

    void setup() override {
        rb_ = addMod<RigidBody>(shape_, type_);
        rb_->setRestitution(0.2f);
        renderer_ = addMod<ColliderRenderer>();
        renderer_->setColor(base_);
    }

    void update() override {
        // Brighten toward white while hovered, ease back when not.
        float target = hovered_ ? 1.0f : 0.0f;
        glow_ += (target - glow_) * std::min(1.0f, (float)getDeltaTime() * 12.0f);
        renderer_->setColor(Color(
            clamp(base_.r + (1.0f - base_.r) * glow_, 0.0f, 1.0f),
            clamp(base_.g + (1.0f - base_.g) * glow_, 0.0f, 1.0f),
            clamp(base_.b + (1.0f - base_.b) * glow_, 0.0f, 1.0f)));
    }

    // rb_ is only set in setup(), which runs a frame after the node is added — so
    // stay null-safe for the brief window before the body exists.
    uint32_t bodyId() const { return rb_ ? rb_->body().getId() : PhysicsBody::kInvalidId; }
    const PhysicsBody& body() const { return rb_->body(); }
    void setHovered(bool h) { hovered_ = h; }

private:
    ColliderShape     shape_;
    Color             base_;
    BodyType          type_;
    RigidBody*        rb_ = nullptr;
    ColliderRenderer* renderer_ = nullptr;
    bool              hovered_ = false;
    float             glow_ = 0.0f;
};
