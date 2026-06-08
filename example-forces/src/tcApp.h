#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>
#include <vector>

using namespace std;
using namespace tc;
using namespace tcx;

// forces - push rigid bodies around with impulses and forces.
//
//   click  : explosion — radial impulse blasting every box away from the centre
//   space  : levitate  — a continuous upward force while held (fights gravity)
//   r      : freeze    — zero every body's velocity
//   drag   : orbit camera
//
// Shows PhysicsBody::applyImpulse (one-shot kick), applyForce (per-frame push),
// and setLinearVelocity (direct override).
class tcApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;

    void mousePressed(Vec2 pos, int button) override;
    void keyPressed(int key) override;
    void keyReleased(int key) override;

private:
    EasyCam cam;
    PhysicsWorld world;

    Mesh unitCube;
    Material blockMat;
    Light keyLight;
    Light fillLight;

    std::vector<PhysicsBody> blocks;
    std::vector<Vec3> startPos;   // initial stack layout, for reset (r)
    bool levitate = false;        // space held → upward force each frame
    float lastTime = 0.0f;
};
