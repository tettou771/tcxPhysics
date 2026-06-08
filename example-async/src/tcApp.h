#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>
#include <vector>

using namespace std;
using namespace tc;
using namespace tcx;

// async - run the simulation on its own fixed-timestep clock.
//
// A tower of stacked boxes. Press 'h' to inject a fake 250 ms frame hitch:
//   - in SYNC mode the next step swallows a huge dt → the tower lurches/explodes
//   - in ASYNC mode the background thread kept stepping at 240 Hz through the
//     hitch, so the tower stays stable and just catches up smoothly
// That difference is the whole reason to step asynchronously.
//
//   a     : toggle sync <-> async (240 Hz)
//   h     : inject a 250 ms hitch (simulate a heavy frame)
//   click : nudge the tower (impulse) / r: rebuild
//   drag  : orbit camera
class tcApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;

    void mousePressed(Vec2 pos, int button) override;
    void keyPressed(int key) override;

private:
    void buildTower();

    EasyCam cam;
    PhysicsWorld world;

    Mesh unitCube;
    Material mat;
    Light keyLight;
    Light fillLight;

    std::vector<PhysicsBody> blocks;
    bool hitchRequested = false;
    float lastTime = 0.0f;
};
