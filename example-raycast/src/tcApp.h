#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>
#include "Pickable.h"

using namespace std;
using namespace tc;
using namespace tcx;

// raycast - mouse picking with PhysicsWorld::raycast().
//
// Every frame a ray is built from the camera through the mouse cursor (origin =
// camera position, direction toward the point under the cursor) and cast into the
// world. The body it hits is highlighted; the ray, the hit point and its surface
// normal are drawn. SPACE shoots the hovered body with an impulse along the ray.
//
//   move  : hover-highlight the body under the cursor
//   SPACE : shoot the hovered body (impulse along the ray)
//   click : drop more bodies
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
    void populate();
    void updatePick();   // build the ray, cast it, flag the hovered body

    EasyCam cam;
    Light   keyLight;
    Light   fillLight;

    vector<shared_ptr<Pickable>> picks;

    // Current frame's pick state (filled by updatePick, drawn in draw()).
    Vec3       rayOrigin;
    Vec3       rayDir;
    RaycastHit hover;
    Pickable*  hovered = nullptr;
};
