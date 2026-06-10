#include "tcApp.h"

static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - ragdoll  (swing-twist / cone / hinge limits)");

    cam.setTarget(0.0f, 1.0f, 0.0f);
    cam.setDistance(7.0f);
    cam.setAzimuth(0.4f);
    cam.setElevation(0.25f);
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

    buildScene();
}

void tcApp::buildScene() {
    dolls.clear();
    dolls.resize(3);
    dolls[0].build(*this, Vec3(-1.1f, 0.6f, -0.3f), Color(0.85f, 0.45f, 0.30f));
    dolls[1].build(*this, Vec3( 0.0f, 1.2f,  0.2f), Color(0.35f, 0.65f, 0.85f));
    dolls[2].build(*this, Vec3( 1.1f, 1.8f, -0.1f), Color(0.55f, 0.80f, 0.40f));
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
    drawFloorGrid(3.0f, 0.5f);
    if (showJoints)
        for (auto& j : defaultWorld().getJoints()) j.drawWire();
}

void tcApp::endDraw() {
    cam.end();
    setColor(1.0f);
    drawBitmapString(
        "joints: " + std::to_string((int)defaultWorld().getJoints().size()) + "\n" +
        "\n" +
        "Each doll: swing-twist neck/shoulders/hips +\n" +
        "one-way hinge elbows/knees (Ragdoll.h). The\n" +
        "limits keep poses anatomical as they flop.\n" +
        "\n" +
        "click: toss   J: joint wires   R: reset\n" +
        "drag: orbit",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    for (auto& d : dolls)
        d.kick(Vec3(random(-1.5f, 1.5f), random(3.5f, 5.0f), random(-1.5f, 1.5f)));
}

void tcApp::keyPressed(int key) {
    if (key == 'J') showJoints = !showJoints;
    if (key == 'R') {
        for (auto& d : dolls) d.clear();
        buildScene();
    }
}
