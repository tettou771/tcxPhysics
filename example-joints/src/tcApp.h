#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace std;
using namespace tc;
using namespace tcx;

// joints - constrain bodies to each other (physics as Node Mods).
//
// Three classic constructions, all wired with RigidBody::jointTo / jointToWorld
// and the Joint:: factories:
//
//   chain    - an anchor + links pinned with Joint::point (ball joints)
//   door     - a panel on Joint::hinge with angle limits, on a static frame
//   pendulum - a ball hung from the air with Joint::distance + spring
//
// The world owns every joint. drawWire() visualizes them each frame via
// defaultWorld().getJoints(). Destroying a node removes its joints
// automatically (R rebuilds the scene to show that).
//
//   click : shove the door and the pendulum
//   R     : rebuild the scene
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

    // Spawn a node with a RigidBody (+ColliderRenderer) attached IMMEDIATELY
    // (addMod from app code creates the body right away, so joints can be
    // wired in the same breath).
    Node* spawnBody(const ColliderShape& shape, const Color& color,
                    const Vec3& pos, BodyType type = BodyType::Dynamic);

    EasyCam cam;
    Light   keyLight;
    Light   fillLight;

    Node* door = nullptr;       // hinge demo
    Node* pendulum = nullptr;   // distance-spring demo
    Node* chainTail = nullptr;  // bottom link of the point-joint chain
    Node* sign = nullptr;       // sixDof loose-mount demo
};
