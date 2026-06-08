#include "tcApp.h"

static constexpr int   COUNT   = 7;       // number of spheres in the row
static constexpr float RADIUS  = 18.0f;
static constexpr float SPACING = 60.0f;
static constexpr float DROP_Y  = 320.0f;  // start height

void tcApp::setup() {
    setWindowTitle("tcxPhysics - bounce  (restitution 0 .. 0.9, click to re-drop)");

    cam.setDistance(560.0f);
    cam.setTarget(0.0f, 120.0f, 0.0f);
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

    world.setup();
    world.setGravity(Vec3(0.0f, -600.0f, 0.0f));
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

    // Legend: restitution value under each column would need projection; keep it
    // simple with a 2D readout.
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
