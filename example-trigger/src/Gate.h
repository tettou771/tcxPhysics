#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace tc;
using namespace tcx;

// Gate - a trigger volume (sensor).
//
// A static, box-shaped RigidBody marked as a trigger: bodies pass straight
// THROUGH it (no collision response) while it reports who is overlapping via the
// onTrigger* events. It keeps a live count of occupants and glows brighter the
// more things are inside. Drawn as a wireframe so you can see cubes pass through.
class Gate : public Node {
public:
    Gate(const Vec3& pos, const Vec3& size) : size_(size) { setPos(pos); }

    void setup() override {
        auto* rb = addMod<RigidBody>(ColliderShape::box(size_), BodyType::Static);
        rb->setTrigger(true);

        enterL_ = rb->onTriggerBegan.listen(this, &Gate::onEnter);
        exitL_  = rb->onTriggerEnded.listen(this, &Gate::onExit);

        box_ = createBox(size_.x, size_.y, size_.z);
    }

    void draw() override {
        // Glow with occupancy: cool blue when empty, hot cyan/white when busy.
        float t = clamp((float)inside_ / 4.0f, 0.0f, 1.0f);
        setColor(Color(0.2f + 0.8f * t, 0.5f + 0.5f * t, 0.9f));
        box_.drawWireframe();
    }

    int occupants() const { return inside_; }

private:
    void onEnter(Collision& c) { inside_++; }
    void onExit (Collision& c) { if (inside_ > 0) inside_--; }

    Vec3          size_;
    Mesh          box_;
    EventListener enterL_, exitL_;
    int           inside_ = 0;
};
