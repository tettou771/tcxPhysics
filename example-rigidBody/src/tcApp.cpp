#include "tcApp.h"
#include "Prop.h"

static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - rigidBody  (physics as Node Mods)");

    cam.setTarget(0.0f, 0.5f, 0.0f);
    cam.setDistance(6.0f);
    cam.setAzimuth(0.6f);
    cam.setElevation(0.35f);
    cam.enableMouseInput();

    keyLight.setDirectional(Vec3(-0.4f, -1.0f, -0.6f));
    keyLight.setDiffuse(1.0f, 0.95f, 0.88f);
    keyLight.setIntensity(2.6f);
    addLight(keyLight);

    fillLight.setDirectional(Vec3(0.6f, -0.3f, 0.5f));
    fillLight.setDiffuse(0.6f, 0.6f, 0.65f);
    fillLight.setIntensity(1.1f);
    addLight(fillLight);

    defaultWorld().setup();              // the singleton world Props attach to
    defaultWorld().addGroundPlane(0.0f);

    spawn(12);
}

void tcApp::spawn(int n) {
    for (int i = 0; i < n; i++) {
        Color base(random(0.15f, 0.95f), random(0.15f, 0.95f), random(0.15f, 0.95f));
        Vec3  pos(random(-1.2f, 1.2f), random(2.5f, 3.5f), random(-1.2f, 1.2f));
        // One self-contained object — no world passed; it uses the default world.
        addChild(make_shared<Prop>(base, pos));
    }
}

void tcApp::update() {
    float dt = std::min(0.05f, std::max(0.0f, (float)getDeltaTime()));
    defaultWorld().update(dt);   // Props' RigidBody mods sync after this

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
        "bodies: " + std::to_string((int)getChildCount()) + "\n" +
        "\n" +
        "Each body is a self-contained Prop (Prop.h):\n" +
        "a Node + RigidBody + ColliderRenderer that\n" +
        "handles its own collisions (flash = impact,\n" +
        "warm glow = resting).\n" +
        "\n" +
        "click: drop shapes   R: clear\n" +
        "drag:  orbit camera",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    (void)pos; (void)button;
    spawn(6);
}

void tcApp::keyPressed(int key) {
    if (key == 'R') {
        for (auto& child : getChildren()) child->destroy();
    }
}
