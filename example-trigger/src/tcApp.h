#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>
#include "Gate.h"

using namespace std;
using namespace tc;
using namespace tcx;

// trigger - sensor volumes (physics as Node Mods).
//
// A wireframe Gate (see Gate.h) is a static *trigger*: a RigidBody marked with
// setTrigger(true). Dynamic cubes (Faller.h) fall straight THROUGH it — a sensor
// produces no collision response — while it reports overlaps via onTrigger*
// events. The gate counts who's inside and glows; each cube flares white while it
// is passing through.
//
//   click : drop a few cubes
//   R     : clear
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
    void spawn(int n);

    EasyCam cam;
    Light   keyLight;
    Light   fillLight;
    Gate*   gate = nullptr;   // the trigger volume (owned by the node tree)
};
