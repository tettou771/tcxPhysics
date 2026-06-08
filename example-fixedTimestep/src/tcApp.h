#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>
#include <vector>

using namespace std;
using namespace tc;
using namespace tcx;

// fixedTimestep - why a physics solver needs a steady step rate, independent of
// the render frame rate ("Fix Your Timestep!").
//
// A tall stack of boxes. In PER-FRAME mode the solver only steps as often as you
// render (~60/s) — too coarse for a tall stack, so it visibly jitters and topples
// on its own. Switch to FIXED-STEP and the sim runs at a steady 240 Hz on its own
// clock (a background thread; on web, the frame loop) — 4x finer stepping — and
// the same stack stands rock-solid.
//
//   F     : toggle per-frame <-> fixed-step (240 Hz)
//   R     : rebuild the stack
//   drag  : orbit camera
class tcApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;

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
    float lastTime = 0.0f;
};
