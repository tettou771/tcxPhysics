#include "tcApp.h"
#include "Faller.h"

static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - trigger  (sensor volume)");

    cam.setTarget(0.0f, 1.0f, 0.0f);
    cam.setDistance(7.0f);
    cam.setAzimuth(0.6f);
    cam.setElevation(0.3f);
    cam.enableMouseInput();

    keyLight.setDirectional(Vec3(-0.4f, -1.0f, -0.6f));
    keyLight.setDiffuse(1.0f, 0.95f, 0.88f);
    keyLight.setIntensity(2.6f);
    addLight(keyLight);

    fillLight.setDirectional(Vec3(0.6f, -0.3f, 0.5f));
    fillLight.setDiffuse(0.6f, 0.6f, 0.65f);
    fillLight.setIntensity(1.1f);
    addLight(fillLight);

    defaultWorld().setup();
    defaultWorld().addGroundPlane(0.0f);

    // A wide, flat sensor slab floating at mid-height — cubes fall through it.
    auto g = make_shared<Gate>(Vec3(0.0f, 1.4f, 0.0f), Vec3(2.6f, 0.5f, 2.6f));
    gate = g.get();
    addChild(g);

    spawn(8);
}

void tcApp::spawn(int n) {
    for (int i = 0; i < n; i++) {
        Color base(random(0.15f, 0.6f), random(0.3f, 0.7f), random(0.5f, 0.95f));
        Vec3  pos(random(-0.9f, 0.9f), random(3.5f, 4.5f), random(-0.9f, 0.9f));
        addChild(make_shared<Faller>(base, pos));
    }
}

void tcApp::update() {
    float dt = std::min(0.05f, std::max(0.0f, (float)getDeltaTime()));
    defaultWorld().update(dt);
}

void tcApp::beginDraw() {
    clear(0.08f, 0.09f, 0.11f);
    cam.begin();
    setCameraPosition(cam.getPosition());
}

void tcApp::draw() {
    drawFloorGrid(2.5f, 0.5f);
}

void tcApp::endDraw() {
    cam.end();
    setColor(1.0f);
    drawBitmapString(
        "inside gate: " + std::to_string(gate ? gate->occupants() : 0) + "\n" +
        "\n" +
        "The blue box is a TRIGGER (sensor): a static\n" +
        "RigidBody with setTrigger(true). Cubes fall\n" +
        "straight THROUGH it (no collision response) and\n" +
        "flare white while overlapping. The gate counts\n" +
        "occupants via onTrigger Began/Ended and glows.\n" +
        "\n" +
        "click: drop cubes   R: clear   drag: orbit",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    spawn(5);
}

void tcApp::keyPressed(int key) {
    if (key == 'R') {
        // Clear the fallers but keep the gate.
        for (auto& child : getChildren())
            if (child.get() != (Node*)gate) child->destroy();
    }
}
