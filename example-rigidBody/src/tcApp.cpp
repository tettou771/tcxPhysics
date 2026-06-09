#include "tcApp.h"

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

    world.setup();                  // default gravity -9.81
    world.addGroundPlane(0.0f);     // world-level static floor

    spawn(12);
    lastTime = getElapsedTimef();
}

void tcApp::spawn(int n) {
    for (int i = 0; i < n; i++) {
        auto node = make_shared<Node>();              // a PLAIN node — no subclass
        Vec3 p(random(-1.2f, 1.2f), random(2.5f, 3.5f), random(-1.2f, 1.2f));
        node->setPos(p);
        addChild(node);                                // shared_ptr ready in setup()

        // Pick a shape.
        ColliderShape shape;
        int kind = (int)random(0.0f, 3.0f);
        if (kind == 0)      shape = ColliderShape::box(Vec3(0.26f));
        else if (kind == 1) shape = ColliderShape::sphere(0.15f);
        else                shape = ColliderShape::capsule(0.1f, 0.24f);

        // Two mods: physics + its renderer. The renderer reads the RigidBody's
        // shape, so add RigidBody first.
        node->addMod<RigidBody>(world, shape)->setRestitution(0.3f);
        node->addMod<ColliderRenderer>()->setColor(
            Color(random(0.15f, 0.95f), random(0.15f, 0.95f), random(0.15f, 0.95f)));
        spawned++;
    }
}

void tcApp::update() {
    float t = getElapsedTimef();
    float dt = std::min(0.05f, std::max(0.0f, t - lastTime));
    lastTime = t;
    world.update(dt);   // children's RigidBody.earlyUpdate sync after this
}

void tcApp::beginDraw() {
    clear(0.08f, 0.09f, 0.11f);
    cam.begin();
    setCameraPosition(cam.getPosition());
}

void tcApp::draw() {
    drawFloorGrid(2.5f, 0.5f);   // inside the camera; children draw after this
}

void tcApp::endDraw() {
    cam.end();
    setColor(1.0f);
    drawBitmapString(
        "bodies: " + std::to_string(spawned) + "\n" +
        "\n" +
        "Each body = plain Node + RigidBody Mod + ColliderRenderer Mod.\n" +
        "The RigidBody Mod syncs the simulated transform onto the node\n" +
        "(in the Mod dispatch — a Node::update override can't skip it).\n" +
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
        for (auto& child : getChildren()) child->destroy();   // RigidBody.onDestroy frees bodies
        spawned = 0;
    }
}
