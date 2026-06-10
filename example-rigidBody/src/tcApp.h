#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace std;
using namespace tc;
using namespace tcx;

// rigidBody - physics as Node Mods (experimental).
//
// Each falling object is a self-contained `Prop` (see Prop.h): a Node that gives
// itself a RigidBody + ColliderRenderer and reacts to its own collisions. This
// app stays tiny — it just spawns Props and steps the world.
//
//   click : drop a few shapes
//   R     : clear
//   drag  : orbit camera
class tcApp : public App {
public:
    void setup() override;
    void update() override;       // step the world
    void draw() override;         // 3D ground/grid (Props draw themselves)
    void beginDraw() override;    // camera wraps draw() + children
    void endDraw() override;      // end camera, then 2D HUD

    void mousePressed(Vec2 pos, int button) override;
    void keyPressed(int key) override;

private:
    void spawn(int n);

    EasyCam cam;
    Light keyLight;
    Light fillLight;
};
