#include "tcApp.h"

// A small stack of boxes we then shove around.
static constexpr int GRID = 6;        // GRID x GRID footprint
static constexpr int LAYERS = 3;      // stacked this many high
static constexpr float CELL = 26.0f;  // spacing
static constexpr float BOX = 20.0f;   // box size

void tcApp::setup() {
    setWindowTitle("tcxPhysics - forces  (click: explode  space: levitate  r: freeze)");

    cam.setDistance(520.0f);
    cam.setTarget(0.0f, 60.0f, 0.0f);
    cam.enableMouseInput();

    unitCube = createBox(1.0f);

    // Linear baseColor — green channel lifts the red into orange.
    blockMat.setBaseColor(0.85f, 0.16f, 0.01f).setMetallic(0.0f).setRoughness(0.5f);

    // Direct lighting only (IBL-free → identical on every platform).
    keyLight.setDirectional(Vec3(-0.4f, -1.0f, -0.6f));
    keyLight.setDiffuse(1.0f, 0.95f, 0.88f);
    keyLight.setIntensity(2.6f);
    addLight(keyLight);

    fillLight.setDirectional(Vec3(0.6f, -0.3f, 0.5f));
    fillLight.setDiffuse(0.6f, 0.6f, 0.65f);
    fillLight.setIntensity(1.1f);
    addLight(fillLight);

    world.setup();
    world.setGravity(Vec3(0.0f, -400.0f, 0.0f));
    world.addGroundPlane(0.0f);

    // Build the stack centred on the origin.
    float off = (GRID - 1) * CELL * 0.5f;
    for (int y = 0; y < LAYERS; ++y)
        for (int gx = 0; gx < GRID; ++gx)
            for (int gz = 0; gz < GRID; ++gz) {
                Vec3 pos(gx * CELL - off, BOX * 0.5f + y * (BOX + 1.0f), gz * CELL - off);
                blocks.push_back(world.addBox(pos, Vec3(BOX, BOX, BOX)));
            }

    lastTime = getElapsedTimef();
}

void tcApp::update() {
    float t = getElapsedTimef();
    float dt = std::min(0.05f, std::max(0.0f, t - lastTime));
    lastTime = t;

    // Continuous upward force while space is held. applyForce accumulates over
    // the step (unlike an impulse), so holding it gradually lifts the pile.
    if (levitate) {
        for (const PhysicsBody& b : blocks)
            b.applyForce(Vec3(0.0f, 9000.0f, 0.0f));
    }

    world.update(dt);
}

void tcApp::draw() {
    clear(0.08f, 0.09f, 0.11f);

    cam.begin();
    setCameraPosition(cam.getPosition());

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
        "click: explosion\n" +
        "space: levitate (hold)\n" +
        "r:     freeze\n" +
        "drag:  orbit camera",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    (void)pos; (void)button;
    // Explosion: impulse each body away from the blast centre, stronger when
    // closer, plus a little lift so the pile bursts upward.
    const Vec3 centre(0.0f, 40.0f, 0.0f);
    const float power = 90000.0f;
    for (const PhysicsBody& b : blocks) {
        if (!b.isValid()) continue;
        Vec3 dir = b.getPosition() - centre;
        float dist = std::max(20.0f, dir.length());
        Vec3 n = dir / dist;
        n.y += 0.6f;                                  // bias upward
        b.applyImpulse(n * (power / dist));           // 1/d falloff
    }
}

void tcApp::keyPressed(int key) {
    if (key == KEY_SPACE) levitate = true;
    if (key == 'R' || key == 'r') {
        // Freeze everything — direct velocity override.
        for (const PhysicsBody& b : blocks) {
            b.setLinearVelocity(Vec3(0.0f, 0.0f, 0.0f));
            b.setAngularVelocity(Vec3(0.0f, 0.0f, 0.0f));
        }
    }
}

void tcApp::keyReleased(int key) {
    if (key == KEY_SPACE) levitate = false;
}
