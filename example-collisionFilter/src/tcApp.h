#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace std;
using namespace tc;
using namespace tcx;

// collisionFilter - layers, masks and per-body user data.
//
// Two teams of balls share one arena:
//   red  - collision layer 1, mask = ground | red   (ignores blue)
//   blue - collision layer 2, mask = ground | blue  (ignores red)
// They pile up on the same floor but pass straight through each other —
// until SPACE flips every mask to 0xff and the teams suddenly collide.
//
// Every ball also carries setUserData(serial); a contactBegan listener reads
// the two tags back to caption the latest contact ("red #3 x red #7").
//
//   click : drop more balls
//   SPACE : toggle the filter (teams ignore / hit each other)
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
    void rain(int n);
    void applyMasks();

    EasyCam cam;
    Light   keyLight;
    Light   fillLight;

    vector<Node*> balls;       // even serial = red, odd = blue
    bool   filtered = true;    // true: teams ignore each other
    int    nextSerial = 1;
    string lastContact = "(none yet)";
    EventListener contactL;
};
