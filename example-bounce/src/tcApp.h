#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>
#include <vector>

using namespace std;
using namespace tc;
using namespace tcx;

// bounce - material restitution, side by side.
//
// A row of spheres dropped from the same height, each with a higher restitution
// (left = dead, right = super-bouncy). They settle to visibly different bounce
// heights — the whole point of restitution. Colour tracks the value (dark→cyan).
//
//   click / space : drop them again (setPosition reset)
//   drag          : orbit camera
//
// Shows PhysicsBody::setRestitution / setFriction and setPosition (reset).
class tcApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;

    void mousePressed(Vec2 pos, int button) override;
    void keyPressed(int key) override;

private:
    void dropAll();   // teleport every sphere back to the start height, at rest

    EasyCam cam;
    PhysicsWorld world;

    Mesh unitSphere;
    Light keyLight;
    Light fillLight;

    struct Ball {
        PhysicsBody body;
        float restitution;
        Material mat;
        float startX;
    };
    std::vector<Ball> balls;
    float lastTime = 0.0f;
};
