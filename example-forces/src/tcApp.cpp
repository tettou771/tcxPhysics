#include "tcApp.h"

// Metre-scale scene: cubes ~0.3 m, default gravity (-9.81), masses come out in
// kg via the default density (1000). That's why the force/impulse numbers below
// read like real physics instead of magic constants.
static constexpr int   GRID   = 6;       // GRID x GRID footprint
static constexpr int   LAYERS = 3;       // stacked this many high
static constexpr float CELL   = 0.55f;   // spacing (m)

// A faint reference grid on the ground plane (y = 0).
static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - forces  (click: explode  space: levitate  v: jump  r: freeze)");

    // Oblique 3/4 view (not the default head-on side view).
    cam.setTarget(0.0f, 0.7f, 0.0f);
    cam.setDistance(6.0f);
    cam.setAzimuth(0.7f);      // ~40° around
    cam.setElevation(0.5f);    // ~28° looking down
    cam.enableMouseInput();

    unitCube = createBox(1.0f);
    blockMat.setBaseColor(0.85f, 0.16f, 0.01f).setMetallic(0.0f).setRoughness(0.5f);

    keyLight.setDirectional(Vec3(-0.4f, -1.0f, -0.6f));
    keyLight.setDiffuse(1.0f, 0.95f, 0.88f);
    keyLight.setIntensity(2.6f);
    addLight(keyLight);

    fillLight.setDirectional(Vec3(0.6f, -0.3f, 0.5f));
    fillLight.setDiffuse(0.6f, 0.6f, 0.65f);
    fillLight.setIntensity(1.1f);
    addLight(fillLight);

    world.setup();                 // default gravity is -9.81 — no override needed
    world.addGroundPlane(0.0f);

    // A stack of slightly varied cubes, so mass differs body to body — which lets
    // us see getMass() doing real work below.
    float off = (GRID - 1) * CELL * 0.5f;
    for (int y = 0; y < LAYERS; ++y)
        for (int gx = 0; gx < GRID; ++gx)
            for (int gz = 0; gz < GRID; ++gz) {
                float s = random(0.22f, 0.42f);
                Vec3 pos(gx * CELL - off, 0.3f + y * CELL, gz * CELL - off);
                blocks.push_back(world.addBox(pos, Vec3(s)));
                startPos.push_back(pos);
            }

    lastTime = getElapsedTimef();
}

void tcApp::update() {
    float t = getElapsedTimef();
    float dt = std::min(0.05f, std::max(0.0f, t - lastTime));
    lastTime = t;

    // Levitate while space is held: force = mass * acceleration. Using getMass()
    // gives every cube the SAME upward acceleration (~25 m/s², well above gravity)
    // regardless of its size — exactly how real anti-gravity would feel.
    if (levitate)
        for (const PhysicsBody& b : blocks)
            b.applyForce(Vec3(0.0f, b.getMass() * 25.0f, 0.0f));

    world.update(dt);
}

void tcApp::draw() {
    clear(0.08f, 0.09f, 0.11f);

    cam.begin();
    setCameraPosition(cam.getPosition());

    drawFloorGrid(2.5f, 0.5f);

    setMaterial(blockMat);
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

    setColor(1.0f);
    drawBitmapString(
        "boxes: " + std::to_string((int)blocks.size()) + "\n" +
        "fps:   " + std::to_string((int)(getFrameRate() + 0.5)) + "\n" +
        "\n" +
        "click: explosion (impulse = mass x dv)\n" +
        "space: levitate  (force = mass x accel)\n" +
        "v:     jump      (addVelocity, mass-independent)\n" +
        "r:     reset the stack\n" +
        "drag:  orbit camera",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    (void)pos; (void)button;
    // Explosion: give each cube the same outward SPEED near the blast, falling
    // off with distance. impulse = mass * speed, so getMass() makes heavy and
    // light cubes leave at the same velocity.
    const Vec3 centre(0.0f, 0.3f, 0.0f);
    for (const PhysicsBody& b : blocks) {
        if (!b.isValid()) continue;
        Vec3 dir = b.getPosition() - centre;
        float dist = std::max(0.15f, dir.length());
        Vec3 n = dir / dist;
        n.y += 0.5f;                                 // bias upward so it bursts up
        float speed = std::min(6.0f, 2.5f / dist);   // m/s, stronger near centre
        b.applyImpulse(n * (b.getMass() * speed));
    }
}

void tcApp::keyPressed(int key) {
    if (key == KEY_SPACE) levitate = true;
    if (key == 'V') {
        // Mass-independent kick: every cube jumps to the same upward velocity,
        // big or small. No mass in sight — you just say "4 m/s up".
        for (const PhysicsBody& b : blocks)
            b.addVelocity(Vec3(0.0f, 4.0f, 0.0f));
    }
    if (key == 'R') {
        // Reset: teleport every cube back to its starting slot, at rest. Shows
        // setPosition/setRotation (teleport) + zeroing velocity.
        for (size_t i = 0; i < blocks.size(); ++i) {
            blocks[i].setPosition(startPos[i]);
            blocks[i].setRotation(Quaternion::identity());
            blocks[i].setLinearVelocity(Vec3(0.0f, 0.0f, 0.0f));
            blocks[i].setAngularVelocity(Vec3(0.0f, 0.0f, 0.0f));
        }
    }
}

void tcApp::keyReleased(int key) {
    if (key == KEY_SPACE) levitate = false;
}
