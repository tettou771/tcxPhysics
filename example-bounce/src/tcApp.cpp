#include "tcApp.h"

// Metre-scale: spheres ~0.15 m radius dropped from 3 m under default gravity.
static constexpr int   COUNT   = 7;
static constexpr float RADIUS  = 0.15f;
static constexpr float SPACING = 0.5f;
static constexpr float DROP_Y  = 3.0f;

// A faint reference grid on the ground plane (y = 0).
static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - bounce  (restitution 0 .. 0.9, click to re-drop)");

    // Fairly low, slightly-raised view so the different bounce heights line up.
    cam.setTarget(0.0f, 0.9f, 0.0f);
    cam.setDistance(5.5f);
    cam.setAzimuth(0.5f);
    cam.setElevation(0.22f);
    cam.enableMouseInput();

    unitSphere = createSphere(1.0f, 20);

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

    float off = (COUNT - 1) * SPACING * 0.5f;
    for (int i = 0; i < COUNT; ++i) {
        Ball b;
        b.restitution = (float)i / (COUNT - 1) * 0.9f;  // 0 .. 0.9
        b.startX = i * SPACING - off;

        b.body = world.addSphere(Vec3(b.startX, DROP_Y, 0.0f), RADIUS);
        b.body.setRestitution(b.restitution);
        b.body.setFriction(0.2f);

        // Dark (dead) → cyan (bouncy). Linear baseColor.
        float t = b.restitution / 0.9f;
        b.mat.setBaseColor(0.02f + 0.02f * (1 - t), 0.10f + 0.55f * t, 0.14f + 0.70f * t)
             .setMetallic(0.0f)
             .setRoughness(0.35f);
        balls.push_back(b);
    }

    lastTime = getElapsedTimef();
}

void tcApp::dropAll() {
    for (Ball& b : balls) {
        b.body.setPosition(Vec3(b.startX, DROP_Y, 0.0f));   // teleport to start
        b.body.setLinearVelocity(Vec3(0.0f, 0.0f, 0.0f));   // at rest
        b.body.setAngularVelocity(Vec3(0.0f, 0.0f, 0.0f));
    }
}

void tcApp::update() {
    float t = getElapsedTimef();
    float dt = std::min(0.05f, std::max(0.0f, t - lastTime));
    lastTime = t;
    world.update(dt);
}

void tcApp::draw() {
    clear(0.08f, 0.09f, 0.11f);

    cam.begin();
    setCameraPosition(cam.getPosition());

    drawFloorGrid(2.5f, 0.5f);

    for (Ball& b : balls) {
        if (!b.body.isValid()) continue;
        setMaterial(b.mat);
        Vec3 s = b.body.getSize();
        pushMatrix();
        translate(b.body.getPosition());
        rotate(b.body.getRotation());
        scale(s.x * 0.5f, s.y * 0.5f, s.z * 0.5f);  // getSize is full extent; mesh radius 1
        unitSphere.draw();
        popMatrix();
    }
    clearMaterial();

    cam.end();

    setColor(1.0f);
    string line = "restitution (left -> right): ";
    for (int i = 0; i < COUNT; ++i) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%.2f", balls[i].restitution);
        line += buf;
        if (i < COUNT - 1) line += "  ";
    }
    drawBitmapString(
        line + "\n\n" +
        "click / space: re-drop\n" +
        "drag:          orbit camera",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    (void)pos; (void)button;
    dropAll();
}

void tcApp::keyPressed(int key) {
    if (key == KEY_SPACE) dropAll();
}
