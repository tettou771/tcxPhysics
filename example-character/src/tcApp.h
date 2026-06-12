#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>
#include "Player.h"

using namespace std;
using namespace tc;
using namespace tcx;

// character - a walking-character controller (Jolt CharacterVirtual).
//
// WASD walks (camera-relative), SPACE jumps. The playground shows everything
// the controller handles for free:
//
//   slope     - a 20 deg ramp, walkable
//   cliff     - a 60 deg wall, beyond the slope limit: the character slides off
//   stairs    - 0.25 m steps, climbed without jumping (stair stepping)
//   platform  - a kinematic mover you can ride
//   crates    - light dynamic boxes you can shove through
//
//   WASD  : walk (camera-relative)   SPACE : jump
//   R     : reset                    drag  : orbit camera
class tcApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;
    void beginDraw() override;
    void endDraw() override;

    void keyPressed(int key) override;
    void keyReleased(int key) override;

private:
    void buildScene();
    Node* spawnBody(const ColliderShape& shape, const Color& color,
                    const Vec3& pos, BodyType type = BodyType::Dynamic,
                    float density = 1000.0f, const Vec3& euler = Vec3());

    EasyCam cam;
    Light   keyLight;
    Light   fillLight;

    Player* player = nullptr;
    Node*   platform = nullptr;
    bool wDown = false, aDown = false, sDown = false, dDown = false;
};
