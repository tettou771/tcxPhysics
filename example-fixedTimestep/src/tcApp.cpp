#include "tcApp.h"

// Metre-scale: a tower of ~0.25 m boxes under default gravity (-9.81).
static constexpr int   TOWER_H = 14;     // boxes tall
static constexpr float BOX     = 0.25f;

// A faint reference grid on the ground plane (y = 0).
static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - fixedTimestep  (F: per-frame/fixed  R: rebuild)");

    // Cap the whole loop to 30 fps so the contrast is obvious on ANY display
    // (a 120 Hz monitor would otherwise step per-frame physics fairly finely on
    // its own). At 30 fps, PER-FRAME physics steps only 30x/s — too coarse for a
    // tall stack — while FIXED-STEP keeps running at 240 Hz regardless.
    setFps(30);

    cam.setTarget(0.0f, 1.6f, 0.0f);
    cam.setDistance(7.0f);
    cam.setAzimuth(0.5f);
    cam.setElevation(0.18f);
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

    world.setup();                 // default gravity -9.81
    world.addGroundPlane(0.0f);
    buildTower();

    lastTime = getElapsedTimef();
}

void tcApp::buildTower() {
    world.clearDynamicBodies();
    blocks.clear();
    // Slight alternating offset so it's a believable, faintly unstable stack.
    for (int i = 0; i < TOWER_H; ++i) {
        float jitter = (i % 2 == 0) ? 0.015f : -0.015f;
        Vec3 pos(jitter, BOX * 0.5f + i * (BOX + 0.005f), 0.0f);
        blocks.push_back(world.addBox(pos, Vec3(BOX, BOX, BOX)));
    }
}

void tcApp::update() {
    float t = getElapsedTimef();
    float dt = std::min(0.05f, std::max(0.0f, t - lastTime));
    lastTime = t;

    // PER-FRAME: one step per rendered frame (30/s here). FIXED-STEP: the worker
    // drives the sim at 240 Hz, so update() would be a no-op (and warn) — skip it.
    if (!world.isAsync()) world.update(dt);
}

void tcApp::draw() {
    clear(0.08f, 0.09f, 0.11f);

    cam.begin();
    setCameraPosition(cam.getPosition());

    drawFloorGrid(2.5f, 0.5f);

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

    string mode = world.isAsync() ? "FIXED-STEP (240 Hz, background)"
                                   : "PER-FRAME (30 fps -> 30 steps/s)";
    setColor(1.0f);
    drawBitmapString(
        "mode: " + mode + "\n" +
        "fps:  " + std::to_string((int)(getFrameRate() + 0.5)) + "\n" +
        "\n" +
        "The loop is capped to 30 fps. A solver is only as stable\n" +
        "as its STEP RATE:\n" +
        "  PER-FRAME steps once per frame (30/s) -> too coarse for\n" +
        "  a tall stack: it jitters and topples on its own.\n" +
        "  FIXED-STEP runs at a steady 240 Hz, no matter the fps\n" +
        "  -> the same stack stands solid.\n" +
        "\n" +
        "Try: watch it wobble in PER-FRAME, press F (it firms up),\n" +
        "     then R to rebuild.\n" +
        "\n" +
        "F:    toggle per-frame / fixed-step\n" +
        "R:    rebuild stack\n" +
        "drag: orbit camera",
        20.0f, 20.0f);
}

void tcApp::keyPressed(int key) {
    // Letter keycodes are uppercase ASCII (SAPP_KEYCODE_A == 'A'); compare upper.
    if (key == 'F') {
        if (world.isAsync()) world.updateAsyncStop();
        else                 world.updateAsyncStart(240.0f);
    }
    if (key == 'R') buildTower();
}
