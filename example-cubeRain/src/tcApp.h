#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace std;
using namespace tc;
using namespace tcx;

// cubeRain - hold the mouse to pour ~100 small cubes per second into a 3D pile.
// Carrot-orange blocks lit by a key + fill light, with a live cube count and FPS
// in the top-left. Direct lighting only (no IBL) so the look is identical on
// every platform — IBL is auto-skipped on iOS Safari anyway (see Environment).
class tcApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;

    void mousePressed(Vec2 pos, int button) override;
    void mouseReleased(Vec2 pos, int button) override;

private:
    EasyCam cam;
    PhysicsWorld world;

    Mesh unitCube;        // a 1x1x1 box, scaled per body when drawn
    Material blockMat;    // matte carrot-orange
    Light keyLight;
    Light fillLight;

    std::vector<PhysicsBody> blocks;

    bool mouseDown = false;
    float spawnAccum = 0.0f;   // fractional carry for the 100/sec spawn rate
    float lastTime = 0.0f;     // for per-frame delta time
};
