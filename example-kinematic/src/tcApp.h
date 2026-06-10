#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace std;
using namespace tc;
using namespace tcx;

// kinematic - YOU drive the body, it pushes the dynamics (physics as Node Mods).
//
// Two BodyType::Kinematic movers (Movers.h): a Slider that slides back and forth
// and a Spinner that rotates like a paddle. You animate their NODES in update();
// the RigidBody mod feeds that motion to Jolt as a kinematic move, so they shove
// the dynamic Cubes (Cube.h) around with real momentum while ignoring gravity and
// impacts themselves.
//
//   click : drop more cubes
//   R     : clear cubes
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
    void rain(int n);

    EasyCam cam;
    Light   keyLight;
    Light   fillLight;
    Node*   slider = nullptr;
    Node*   spinner = nullptr;
};
