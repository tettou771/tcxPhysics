#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>
#include "Ragdoll.h"

using namespace std;
using namespace tc;
using namespace tcx;

// ragdoll - swing-twist + cone + hinge joints in their natural habitat.
//
// A small humanoid (Ragdoll.h) wired with the classic ragdoll joints:
// swing-twist shoulders/hips/neck and one-way hinge elbows/knees. It flops
// onto the floor like a puppet with its strings cut — joint limits keep the
// poses anatomical (elbows never bend backwards).
//
//   click : toss the ragdolls in the air
//   J     : toggle joint wireframes
//   R     : reset
//   drag  : orbit camera
class tcApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;
    void beginDraw() override;
    void endDraw() override;

    void mousePressed(Vec2 pos, int button) override;
    void keyPressed(int key) override;

private:
    void buildScene();

    EasyCam cam;
    Light   keyLight;
    Light   fillLight;

    vector<Ragdoll> dolls;
    bool showJoints = true;
};
