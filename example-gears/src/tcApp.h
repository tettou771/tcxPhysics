#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace std;
using namespace tc;
using namespace tcx;

// gears - transmissions: gear & rack-and-pinion joints.
//
// One motorized paddle wheel drives everything else through constraints:
//
//   wheel A - hinge + velocity motor (the engine)
//   wheel B - hinge, linked to A with addGearJoint (counter-rotates, 2:1)
//   rack    - a bar on a world slider, linked to A with addRackAndPinionJoint
//             (rotation <-> translation; it shuttles as the wheel turns)
//
// The motor reverses every few seconds, so the whole machine runs back and
// forth. Joint wires drawn with drawWire().
//
//   click : drop balls into the machine
//   SPACE : reverse now
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

    Node* spawnBody(const ColliderShape& shape, const Color& color,
                    const Vec3& pos, BodyType type = BodyType::Dynamic);

    EasyCam cam;
    Light   keyLight;
    Light   fillLight;

    PhysicsJoint engineJoint;     // motorized hinge on wheel A
    float        engineSpeed = 1.6f;
    float        nextReverse = 0.0f;
};
