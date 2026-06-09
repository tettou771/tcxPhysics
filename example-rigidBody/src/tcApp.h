#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>
#include <tcxPhysicsMod.h>   // experimental RigidBody / ColliderRenderer Mods
#include <memory>

using namespace std;
using namespace tc;
using namespace tcx;

// rigidBody - physics as Node Mods (experimental).
//
// Each falling body is a plain Node carrying two Mods:
//   - RigidBody:         owns the Jolt body; its earlyUpdate copies the simulated
//                        transform onto the node every frame (in the Mod dispatch,
//                        so it can never be skipped by a Node::update override).
//   - ColliderRenderer: draws the body's shape with a render material.
// No custom Node subclass needed — pure composition.
//
//   click : drop a few shapes
//   R     : clear
//   drag  : orbit camera
class tcApp : public App {
public:
    void setup() override;
    void update() override;       // step the world
    void draw() override;         // 3D ground/grid (children draw themselves)
    void beginDraw() override;    // camera wraps draw() + children
    void endDraw() override;      // end camera, then 2D HUD

    void mousePressed(Vec2 pos, int button) override;
    void keyPressed(int key) override;

private:
    void spawn(int n);

    EasyCam cam;
    PhysicsWorld world;
    Light keyLight;
    Light fillLight;
    int spawned = 0;
    float lastTime = 0.0f;
};
