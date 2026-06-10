#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace std;
using namespace tc;
using namespace tcx;

// motors - joints that DRIVE (physics as Node Mods).
//
// Two motorized constructions:
//
//   windmill - a paddle on a hinge with a velocity motor
//              (Joint::hinge(...).motor(speed)); it spins forever and flings
//              whatever lands on it.
//   elevator - a platform on a slider jointed to the world; a position motor
//              (setMotorTarget) drives it up and down between two stops.
//
//   click : drop balls onto the scene
//   SPACE : reverse the windmill
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
    void rain(int n);

    Node* spawnBody(const ColliderShape& shape, const Color& color,
                    const Vec3& pos, BodyType type = BodyType::Dynamic);

    EasyCam cam;
    Light   keyLight;
    Light   fillLight;

    PhysicsJoint windmillJoint;       // velocity motor (reversible)
    PhysicsJoint elevatorJoint;       // position motor (up / down)
    float        windmillSpeed = 3.0f;
    float        nextElevatorMove = 0.0f;
    bool         elevatorUp = false;
    vector<Node*> balls;
};
