#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace std;
using namespace tc;
using namespace tcx;

// joltNativeAccess - the raw-Jolt escape hatch.
//
// tcxPhysics wraps the common 90% (bodies, shapes, joints, motors, filters...).
// For the rest, <tcxPhysicsJolt.h> hands you the raw JPH::PhysicsSystem. This
// example uses it for a PATH CONSTRAINT — a feature the wrapper does NOT
// expose: a bead locked onto a closed Hermite-spline loop, sliding around the
// tilted track under gravity alone, forever.
//
//   click : push the bead along the track
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
    Light keyLight;
    Light fillLight;

    PhysicsWorld world;
    PhysicsBody  bead;
    Mesh         beadMesh;
    Material     beadMat;
    vector<Vec3> trackPoints;   // sampled loop, for drawing
    float lastTime = 0.0f;
};
