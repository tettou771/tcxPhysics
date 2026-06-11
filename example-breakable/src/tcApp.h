#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace std;
using namespace tc;
using namespace tcx;

// breakable - joints that SNAP under load.
//
// A plank bridge hangs between two pillars, every junction a point joint with
// Joint::point(...).breakForce(N). Drop weights on it: the tension climbs,
// and when one junction transmits more than its threshold it snaps —
// PhysicsWorld::jointBroke fires (we mark the spot red) and the bridge tears
// apart from its weakest link.
//
//   click : drop a heavy ball onto the bridge
//   R     : rebuild the bridge
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
    void buildBridge();

    Node* spawnBody(const ColliderShape& shape, const Color& color,
                    const Vec3& pos, BodyType type = BodyType::Dynamic,
                    float density = 1000.0f);

    EasyCam cam;
    Light   keyLight;
    Light   fillLight;

    struct BreakMark { Vec3 pos; float age = 0.0f; };
    vector<BreakMark> marks;          // where joints snapped (drawn fading red)
    EventListener breakL;
    int   breakCount = 0;
    float lastBreakForce = 0.0f;
};
