#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>
#include <vector>
#include <unordered_map>

using namespace std;
using namespace tc;
using namespace tcx;

// collision - react to contact events.
//
// Boxes rain onto a floor; every time two bodies start touching we get a
// contactBegan event. We flash the bodies on impact (brighter the harder the
// hit, using ContactEventArgs::speed) and pop a short-lived spark at the contact
// point. A running counter shows how many contacts have fired.
//
//   click : drop a fresh burst of boxes
//   drag  : orbit camera
//
// Shows PhysicsWorld::contactBegan (tc::Event) with main-thread-safe handlers,
// and using the contact point / approach speed.
class tcApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;

    void mousePressed(Vec2 pos, int button) override;

private:
    void onContact(ContactEventArgs& c);   // listener body
    void spawnBurst(int n);

    EasyCam cam;
    PhysicsWorld world;
    EventListener contactListener;          // RAII — disconnects on destroy

    Mesh unitCube;
    Light keyLight;
    Light fillLight;

    struct Block {
        PhysicsBody body;
        float flash = 0.0f;                 // 0..1, decays each frame
    };
    std::vector<Block> blocks;
    std::unordered_map<uint32_t, int> idToIndex;   // Jolt id -> blocks index

    struct Spark { Vec3 pos; float life; float strength; };
    std::vector<Spark> sparks;

    long contactCount = 0;
    float lastTime = 0.0f;
};
