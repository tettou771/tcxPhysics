#include "tcApp.h"

#include <thread>
#include <chrono>

static constexpr int   TOWER_H = 14;     // boxes tall
static constexpr float BOX     = 26.0f;

void tcApp::setup() {
    setWindowTitle("tcxPhysics - async  (a: sync/async  h: hitch  click: nudge)");

    cam.setDistance(620.0f);
    cam.setTarget(0.0f, 160.0f, 0.0f);
    cam.enableMouseInput();

    unitCube = createBox(1.0f);
    mat.setBaseColor(0.85f, 0.16f, 0.01f).setMetallic(0.0f).setRoughness(0.5f);

    keyLight.setDirectional(Vec3(-0.4f, -1.0f, -0.6f));
    keyLight.setDiffuse(1.0f, 0.95f, 0.88f);
    keyLight.setIntensity(2.6f);
    addLight(keyLight);

    fillLight.setDirectional(Vec3(0.6f, -0.3f, 0.5f));
    fillLight.setDiffuse(0.6f, 0.6f, 0.65f);
    fillLight.setIntensity(1.1f);
    addLight(fillLight);

    world.setup();
    world.setGravity(Vec3(0.0f, -600.0f, 0.0f));
    world.addGroundPlane(0.0f);
    buildTower();

    lastTime = getElapsedTimef();
}

void tcApp::buildTower() {
    world.clearDynamicBodies();
    blocks.clear();
    // Slight alternating offset so it's a believable, faintly unstable stack.
    for (int i = 0; i < TOWER_H; ++i) {
        float jitter = (i % 2 == 0) ? 1.5f : -1.5f;
        Vec3 pos(jitter, BOX * 0.5f + i * (BOX + 0.5f), 0.0f);
        blocks.push_back(world.addBox(pos, Vec3(BOX, BOX, BOX)));
    }
}

void tcApp::update() {
    float t = getElapsedTimef();
    float dt = std::min(0.05f, std::max(0.0f, t - lastTime));
    lastTime = t;

    // Simulate a heavy frame: block the main thread for 250 ms. In async mode
    // the physics worker keeps stepping during this sleep; in sync mode the sim
    // is frozen and the resume produces one oversized (clamped) step.
    if (hitchRequested) {
        hitchRequested = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        // Re-read the clock so this frame's dt reflects the hitch.
        t = getElapsedTimef();
        dt = std::min(0.05f, std::max(0.0f, t - lastTime));
        lastTime = t;
    }

    // In async mode update() is a no-op (the worker drives the sim); calling it
    // would warn, so we only step when synchronous.
    if (!world.isAsync()) world.update(dt);
}

void tcApp::draw() {
    clear(0.08f, 0.09f, 0.11f);

    cam.begin();
    setCameraPosition(cam.getPosition());

    setMaterial(mat);
    for (const PhysicsBody& b : blocks) {
        if (!b.isValid()) continue;
        Vec3 s = b.getSize();
        pushMatrix();
        translate(b.getPosition());
        rotate(b.getRotation());
        scale(s.x, s.y, s.z);
        unitCube.draw();
        popMatrix();
    }
    clearMaterial();

    cam.end();

    string mode = world.isAsync() ? "ASYNC (240 Hz background thread)" : "SYNC (per-frame)";
    setColor(1.0f);
    drawBitmapString(
        "mode: " + mode + "\n" +
        "fps:  " + std::to_string((int)(getFrameRate() + 0.5)) + "\n" +
        "\n" +
        "a:     toggle sync / async\n" +
        "h:     inject 250 ms hitch\n" +
        "click: nudge tower   r: rebuild\n" +
        "drag:  orbit camera",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    (void)pos; (void)button;
    // Shove the upper half of the tower sideways.
    for (size_t i = blocks.size() / 2; i < blocks.size(); ++i)
        if (blocks[i].isValid()) blocks[i].applyImpulse(Vec3(45000.0f, 0.0f, 0.0f));
}

void tcApp::keyPressed(int key) {
    if (key == 'A' || key == 'a') {
        if (world.isAsync()) world.updateAsyncStop();
        else                 world.updateAsyncStart(240.0f);
    }
    if (key == 'H' || key == 'h') hitchRequested = true;
    if (key == 'R' || key == 'r') buildTower();
}
