#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace tc;
using namespace tcx;

// Player - a node with a CharacterBody mod and a simple capsule look.
//
// The CharacterBody does all the locomotion (slopes, stairs, walls, moving
// platforms); this class just draws a capsule + a nose so you can see which
// way it faces, and turns toward the last movement direction.
class Player : public Node {
public:
    static constexpr float RADIUS = 0.3f;
    static constexpr float HEIGHT = 0.8f;   // cylinder part; total = HEIGHT + 2*RADIUS

    Player(const Vec3& pos) { setPos(pos); }

    void setup() override {
        body_ = addMod<CharacterBody>(RADIUS, HEIGHT);
        capsule_ = createCapsule(RADIUS, HEIGHT, 16);
        mat_.setBaseColor(0.85f, 0.45f, 0.15f).setMetallic(0.0f).setRoughness(0.5f);
    }

    CharacterBody* body() { return body_; }

    // Steer (called by the app every frame) — also remembers the facing.
    void move(const Vec3& vel) {
        if (body_) body_->setMoveInput(vel);
        if (vel.length() > 0.1f) setEuler(0.0f, atan2f(vel.x, vel.z), 0.0f);
    }

    void draw() override {
        setMaterial(mat_);
        capsule_.draw();
        drawBox(Vec3(0.0f, 0.25f, RADIUS), 0.12f);   // the nose (local +Z)
        clearMaterial();
    }

private:
    CharacterBody* body_ = nullptr;
    Mesh capsule_;
    Material mat_;
};
