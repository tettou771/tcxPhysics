#include "tcApp.h"

// Metre-scale scene: cubes ~0.2 m under default gravity (-9.81), so masses are a
// few kg and everything falls at a natural pace. Hold the mouse to pour cubes.
static constexpr float SPAWN_PER_SEC = 100.0f;
static constexpr int   MAX_BLOCKS    = 1500;

// A faint reference grid on the ground plane (y = 0).
static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - cubeRain  (hold mouse to pour cubes)");

    // Build stamp — printed to the JS console so we can tell which build a
    // browser actually loaded (cache busting / deploy verification).
    logNotice() << "tcxPhysics cubeRain | build " __DATE__ " " __TIME__;

    // On touch screens (and the web build on iPad/phones), deliver the first
    // touch as mouse events so tap-and-hold pours cubes and drag orbits. This is
    // the default in current TrussC; kept here so the example is version-robust.
    setTouchAsMouse(true);

    // Oblique 3/4 view of the pile.
    cam.setTarget(0.0f, 0.5f, 0.0f);
    cam.setDistance(5.0f);
    cam.setAzimuth(0.6f);
    cam.setElevation(0.4f);
    cam.enableMouseInput();

    // A unit cube reused for every block (scaled per-body at draw time).
    unitCube = createBox(1.0f);

    // Vivid carrot-orange: dielectric, fairly rough so the gloss stays subtle.
    // baseColor is LINEAR — a green channel well above zero turns the red into
    // orange (roughly sRGB #ED7014).
    blockMat.setBaseColor(0.85f, 0.16f, 0.01f)
            .setMetallic(0.0f)
            .setRoughness(0.5f);

    // Direct lighting only — no IBL/Environment, so the look is identical on
    // every platform (IBL is auto-skipped on iOS Safari regardless). A strong
    // fill stands in for the missing image-based ambient.
    keyLight.setDirectional(Vec3(-0.4f, -1.0f, -0.6f));
    keyLight.setDiffuse(1.0f, 0.95f, 0.88f);
    keyLight.setIntensity(2.6f);
    addLight(keyLight);

    fillLight.setDirectional(Vec3(0.6f, -0.3f, 0.5f));
    fillLight.setDiffuse(0.6f, 0.6f, 0.65f);
    fillLight.setIntensity(1.1f);
    addLight(fillLight);

    world.setup(MAX_BLOCKS + 16);  // default gravity -9.81 — natural at this scale
    world.addGroundPlane(0.0f);

    lastTime = getElapsedTimef();
}

void tcApp::update() {
    // Per-frame delta time, clamped so a hitch (or the first frame) can't blow up
    // the simulation.
    float t = getElapsedTimef();
    float dt = std::min(0.05f, std::max(0.0f, t - lastTime));
    lastTime = t;

    // Pour cubes while the mouse is held.
    if (mouseDown && (int)blocks.size() < MAX_BLOCKS) {
        spawnAccum += SPAWN_PER_SEC * dt;
        while (spawnAccum >= 1.0f && (int)blocks.size() < MAX_BLOCKS) {
            spawnAccum -= 1.0f;
            float s = random(0.14f, 0.30f);
            Vec3 pos(random(-0.7f, 0.7f), random(2.6f, 3.2f), random(-0.7f, 0.7f));
            blocks.push_back(world.addBox(pos, Vec3(s, s, s)));
        }
    }

    world.update(dt);
}

void tcApp::draw() {
    clear(0.08f, 0.09f, 0.11f);

    cam.begin();
    setCameraPosition(cam.getPosition());  // needed for correct PBR specular

    drawFloorGrid(2.5f, 0.5f);

    // The blocks.
    setMaterial(blockMat);
    for (const PhysicsBody& b : blocks) {
        if (!b.isValid()) continue;
        Vec3 size = b.getSize();
        pushMatrix();
        translate(b.getPosition());
        rotate(b.getRotation());
        scale(size.x, size.y, size.z);
        unitCube.draw();
        popMatrix();
    }
    clearMaterial();

    cam.end();

    // 2D overlay: live count + FPS.
    setColor(1.0f);
    drawBitmapString(
        "cubes: " + std::to_string((int)blocks.size()) + "\n" +
        "fps:   " + std::to_string((int)(getFrameRate() + 0.5)) + "\n" +
        "\n" +
        "hold mouse: pour cubes\n" +
        "drag:       orbit camera",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    (void)pos; (void)button;
    mouseDown = true;
}

void tcApp::mouseReleased(Vec2 pos, int button) {
    (void)pos; (void)button;
    mouseDown = false;
}
