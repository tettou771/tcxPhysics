#include "tcApp.h"
#include "Movers.h"
#include "Cube.h"

static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - kinematic  (driven movers push dynamics)");

    cam.setTarget(0.0f, 0.5f, 0.0f);
    cam.setDistance(8.0f);
    cam.setAzimuth(0.7f);
    cam.setElevation(0.4f);
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

    // A low slab that slides side to side, and a paddle bar that spins.
    auto sl = make_shared<Slider>(Vec3(3.0f, 0.3f, 3.0f), /*amp*/1.6f, /*speed*/1.8f, /*y*/0.3f);
    slider = sl.get();
    addChild(sl);

    auto sp = make_shared<Spinner>(Vec3(3.0f, 0.4f, 0.3f), /*speed*/2.2f, Vec3(0.0f, 0.9f, 0.0f));
    spinner = sp.get();
    addChild(sp);

    rain(10);
}

void tcApp::rain(int n) {
    for (int i = 0; i < n; i++) {
        Color c = Color::fromHSB(random(0.0f, 1.0f), 0.5f, 0.75f);
        Vec3  p(random(-1.8f, 1.8f), random(2.5f, 4.0f), random(-1.8f, 1.8f));
        addChild(make_shared<Cube>(c, p));
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
    drawFloorGrid(3.0f, 0.5f);
}

void tcApp::endDraw() {
    cam.end();
    setColor(1.0f);
    drawBitmapString(
        "Orange slab = Slider, blue bar = Spinner.\n" +
        std::string("Both are BodyType::Kinematic: the app moves\n") +
        "their NODES, the bodies follow and push the\n" +
        "dynamic cubes with real momentum (but ignore\n" +
        "gravity and impacts themselves).\n" +
        "\n" +
        "click: drop cubes   R: clear   drag: orbit",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    rain(6);
}

void tcApp::keyPressed(int key) {
    if (key == 'R') {
        // Clear cubes but keep the movers.
        for (auto& child : getChildren()) {
            Node* c = child.get();
            if (c != slider && c != spinner) c->destroy();
        }
    }
}
