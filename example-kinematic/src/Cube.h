#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace tc;
using namespace tcx;

// Cube - a plain dynamic body for the kinematic movers to push around.
class Cube : public Node {
public:
    Cube(const Color& color, const Vec3& pos, float size = 0.3f) : color_(color), size_(size) {
        setPos(pos);
    }

    void setup() override {
        addMod<RigidBody>(ColliderShape::box(Vec3(size_)))->setRestitution(0.2f);
        addMod<ColliderRenderer>()->setColor(color_);
    }

private:
    Color color_;
    float size_;
};
