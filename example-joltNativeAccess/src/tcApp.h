#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace std;
using namespace tc;
using namespace tcx;

// jolt - the advanced escape hatch in action.
//
// tcxPhysics doesn't wrap constraints (yet), but we don't have to wait for it:
// <tcxPhysicsJolt.h> hands us the raw JPH::PhysicsSystem so we can build a
// ball-jointed hanging chain ourselves. Everything else (bodies, drawing,
// stepping) stays in the friendly wrapper.
//
//   click : shove the chain sideways (wrapper applyImpulse)
//   drag  : orbit camera
//
// See CMakeLists.txt for the one build line the hatch needs (link Jolt).
class tcApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;

    void mousePressed(Vec2 pos, int button) override;

private:
    EasyCam cam;
    PhysicsWorld world;

    Mesh unitCube;
    Material linkMat;
    Material anchorMat;
    Light keyLight;
    Light fillLight;

    PhysicsBody anchor;                 // static top point
    std::vector<PhysicsBody> links;     // the swinging chain
    float lastTime = 0.0f;
};
