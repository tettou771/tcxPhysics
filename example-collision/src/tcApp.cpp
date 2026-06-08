#include "tcApp.h"

// Metre-scale: cubes ~0.2 m raining onto the floor under default gravity.
static constexpr int   MAX_BLOCKS = 120;
static constexpr float BOX        = 0.2f;
static constexpr float SPEED_REF  = 6.0f;   // approach speed (m/s) that maps to a full flash

static float clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

// A faint reference grid on the ground plane (y = 0).
static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - collision  (contact events: flash + spark + count)");

    cam.setTarget(0.0f, 0.5f, 0.0f);
    cam.setDistance(6.0f);
    cam.setAzimuth(0.6f);
    cam.setElevation(0.4f);
    cam.enableMouseInput();

    unitCube = createBox(1.0f);

    keyLight.setDirectional(Vec3(-0.4f, -1.0f, -0.6f));
    keyLight.setDiffuse(1.0f, 0.95f, 0.88f);
    keyLight.setIntensity(2.6f);
    addLight(keyLight);

    fillLight.setDirectional(Vec3(0.6f, -0.3f, 0.5f));
    fillLight.setDiffuse(0.6f, 0.6f, 0.65f);
    fillLight.setIntensity(1.1f);
    addLight(fillLight);

    world.setup(MAX_BLOCKS + 16);   // default gravity -9.81
    world.addGroundPlane(0.0f);

    // Subscribe to contacts. The handler runs on the MAIN thread (tcxPhysics
    // drains worker-thread contacts here), so touching app state is safe.
    contactListener = world.contactBegan.listen(this, &tcApp::onContact);

    spawnBurst(40);
    lastTime = getElapsedTimef();
}

void tcApp::spawnBurst(int n) {
    if ((int)blocks.size() + n > MAX_BLOCKS) {
        world.clearDynamicBodies();
        blocks.clear();
        idToIndex.clear();
    }
    for (int i = 0; i < n; ++i) {
        Vec3 pos(random(-0.9f, 0.9f), random(2.5f, 3.5f), random(-0.9f, 0.9f));
        Block blk;
        blk.body = world.addBox(pos, Vec3(BOX, BOX, BOX));
        blk.body.setRestitution(0.3f);
        idToIndex[blk.body.getId()] = (int)blocks.size();
        blocks.push_back(blk);
    }
}

void tcApp::onContact(ContactEventArgs& c) {
    ++contactCount;
    float strength = clamp01(c.speed / SPEED_REF);

    // Flash whichever of the two bodies are tracked dynamic blocks.
    for (uint32_t id : {c.a.getId(), c.b.getId()}) {
        auto it = idToIndex.find(id);
        if (it != idToIndex.end())
            blocks[it->second].flash = std::max(blocks[it->second].flash, strength);
    }

    // A spark at the contact point, brighter/bigger for a harder hit.
    if (strength > 0.05f)
        sparks.push_back({c.point, 1.0f, strength});
}

void tcApp::update() {
    float t = getElapsedTimef();
    float dt = std::min(0.05f, std::max(0.0f, t - lastTime));
    lastTime = t;

    world.update(dt);   // contactBegan fires (on this thread) inside here

    // Decay flashes and sparks.
    for (Block& b : blocks) b.flash = std::max(0.0f, b.flash - dt * 3.0f);
    for (Spark& s : sparks) s.life -= dt * 2.5f;
    sparks.erase(std::remove_if(sparks.begin(), sparks.end(),
                                [](const Spark& s) { return s.life <= 0.0f; }),
                 sparks.end());
}

void tcApp::draw() {
    clear(0.07f, 0.08f, 0.10f);

    cam.begin();
    setCameraPosition(cam.getPosition());

    drawFloorGrid(2.5f, 0.5f);

    Material mat;
    mat.setMetallic(0.0f).setRoughness(0.5f);
    for (const Block& b : blocks) {
        if (!b.body.isValid()) continue;
        // Base teal, lerped toward hot white by the current flash.
        float f = b.flash;
        mat.setBaseColor(0.05f + 0.95f * f, 0.30f + 0.70f * f, 0.40f + 0.60f * f);
        setMaterial(mat);
        Vec3 s = b.body.getSize();
        pushMatrix();
        translate(b.body.getPosition());
        rotate(b.body.getRotation());
        scale(s.x, s.y, s.z);
        unitCube.draw();
        popMatrix();
    }

    // Sparks — bright little cubes that shrink as they fade.
    Material sparkMat;
    sparkMat.setMetallic(0.0f).setRoughness(0.2f);
    for (const Spark& s : sparks) {
        sparkMat.setBaseColor(1.0f, 0.9f, 0.5f);
        setMaterial(sparkMat);
        float sz = 0.03f + 0.12f * s.strength * s.life;
        pushMatrix();
        translate(s.pos);
        scale(sz, sz, sz);
        unitCube.draw();
        popMatrix();
    }
    clearMaterial();

    cam.end();

    setColor(1.0f);
    drawBitmapString(
        "contacts: " + std::to_string(contactCount) + "\n" +
        "boxes:    " + std::to_string((int)blocks.size()) + "\n" +
        "fps:      " + std::to_string((int)(getFrameRate() + 0.5)) + "\n" +
        "\n" +
        "click: drop a fresh burst\n" +
        "drag:  orbit camera",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    (void)pos; (void)button;
    spawnBurst(40);
}
