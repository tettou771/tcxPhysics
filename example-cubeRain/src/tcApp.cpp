#include "tcApp.h"

// Spawn ~100 cubes per second while the mouse is held, capped so the frame rate
// stays comfortable.
static constexpr float SPAWN_PER_SEC = 100.0f;
static constexpr int   MAX_BLOCKS    = 4000;

void tcApp::setup() {
    setWindowTitle("tcxPhysics - cubeRain  (hold mouse to pour cubes)");

    // Build stamp — printed to the JS console so we can tell which build a
    // browser actually loaded (cache busting / deploy verification).
    logNotice() << "tcxPhysics cubeRain | build " __DATE__ " " __TIME__
                << " | devicePixelRatio-sensitive HUD";

    // On touch screens (and the web build on iPad/phones), deliver the first
    // touch as mouse events so tap-and-hold pours cubes and drag orbits — this
    // is OFF by default on desktop/web. Without it, taps do nothing.
    setTouchAsMouse(true);

    // Camera looking slightly down at the pile.
    cam.setDistance(440.0f);
    cam.setTarget(0.0f, 50.0f, 0.0f);
    cam.enableMouseInput();

    // A unit cube reused for every block (scaled per-body at draw time).
    unitCube = createBox(1.0f);

    // Vivid carrot-orange: dielectric, fairly rough so the gloss stays subtle.
    // baseColor is LINEAR — a green channel well above zero is what turns the
    // red into orange (roughly sRGB #ED7014).
    blockMat.setBaseColor(0.85f, 0.16f, 0.01f)
            .setMetallic(0.0f)
            .setRoughness(0.5f);

    // Procedural environment gives soft image-based ambient so shadowed faces
    // don't go pure black — the "CG look".
    env.loadProcedural();
    setEnvironment(env);

    // Key light: warm, from front-above. Fill: cool, from the other side.
    keyLight.setDirectional(Vec3(-0.4f, -1.0f, -0.6f));
    keyLight.setDiffuse(1.0f, 0.95f, 0.88f);
    keyLight.setIntensity(2.6f);
    addLight(keyLight);

    fillLight.setDirectional(Vec3(0.6f, -0.3f, 0.5f));
    fillLight.setDiffuse(0.5f, 0.5f, 0.55f);  // near-neutral, faint cool tint
    fillLight.setIntensity(0.45f);
    addLight(fillLight);

    // Physics: punchy gravity so blocks fall at a lively pace at this scale.
    world.setup(MAX_BLOCKS + 16);
    world.setGravity(Vec3(0.0f, -400.0f, 0.0f));
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
            float s = random(6.0f, 14.0f);
            Vec3 pos(random(-40.0f, 40.0f), 220.0f, random(-40.0f, 40.0f));
            blocks.push_back(world.addBox(pos, Vec3(s, s, s)));
        }
    }

    world.update(dt);
}

void tcApp::draw() {
    clear(0.08f, 0.09f, 0.11f);

    cam.begin();
    setCameraPosition(cam.getPosition());  // needed for correct PBR specular

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
