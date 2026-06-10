#include "tcApp.h"

static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - joints  (point / hinge / distance)");

    cam.setTarget(0.0f, 1.2f, 0.0f);
    cam.setDistance(7.5f);
    cam.setAzimuth(0.5f);
    cam.setElevation(0.3f);
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

    buildScene();
}

Node* tcApp::spawnBody(const ColliderShape& shape, const Color& color,
                       const Vec3& pos, BodyType type) {
    auto n = make_shared<Node>();
    n->setPos(pos);
    addChild(n);
    // addMod from app code runs the mod's setup() immediately — the Jolt body
    // exists right after this line, so a joint can be wired straight away.
    n->addMod<RigidBody>(shape, type);
    n->addMod<ColliderRenderer>()->setColor(color);
    return n.get();
}

void tcApp::buildScene() {
    // --- chain: anchor + ball-jointed links (the Mod take on the raw-Jolt
    // chain in example-joltNativeAccess) -------------------------------------
    const float BOX = 0.18f, GAP = 0.26f, TOP = 2.6f;
    Node* prev = spawnBody(ColliderShape::box(Vec3(BOX)), Color(0.10f, 0.30f, 0.45f),
                           Vec3(-1.6f, TOP, 0.0f), BodyType::Static);
    float prevY = TOP;
    for (int i = 0; i < 7; i++) {
        float y = TOP - GAP * (i + 1);
        Node* link = spawnBody(ColliderShape::box(Vec3(BOX)), Color(0.85f, 0.16f, 0.01f),
                               Vec3(-1.6f, y, 0.0f));
        link->getMod<RigidBody>()->jointTo(prev,
            Joint::point(Vec3(-1.6f, (prevY + y) * 0.5f, 0.0f)));
        prev = link;
        prevY = y;
    }
    chainTail = prev;

    // --- door: a panel hinged to a static post, swing limited ---------------
    Node* post = spawnBody(ColliderShape::box(Vec3(0.12f, 1.6f, 0.12f)),
                           Color(0.35f, 0.37f, 0.40f), Vec3(0.0f, 0.8f, 0.0f),
                           BodyType::Static);
    door = spawnBody(ColliderShape::box(Vec3(0.9f, 1.4f, 0.06f)),
                     Color(0.30f, 0.65f, 0.85f), Vec3(0.51f, 0.8f, 0.0f));
    // Hinge on the post edge, swinging around Y, limited to ±100°.
    door->getMod<RigidBody>()->jointTo(post,
        Joint::hinge(Vec3(0.06f, 0.8f, 0.0f), Vec3(0.0f, 1.0f, 0.0f))
            .limits(-TAU * 100.0f / 360.0f, TAU * 100.0f / 360.0f));

    // --- pendulum: a ball hung from the air on a springy distance joint -----
    Vec3 ballPos(1.8f, 1.2f, 0.0f);
    Vec3 hook(1.8f, 2.8f, 0.0f);
    pendulum = spawnBody(ColliderShape::sphere(0.22f), Color(0.55f, 0.80f, 0.40f), ballPos);
    pendulum->getMod<RigidBody>()->jointToWorld(
        Joint::distance(ballPos, hook).spring(2.0f, 0.2f));
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

    // Visualize every live joint (anchor line + markers, axis for hinges).
    for (auto& j : defaultWorld().getJoints()) j.drawWire();
}

void tcApp::endDraw() {
    cam.end();
    setColor(1.0f);
    drawBitmapString(
        "joints: " + std::to_string((int)defaultWorld().getJoints().size()) + "\n" +
        "\n" +
        "chain    : Joint::point ball joints\n" +
        "door     : Joint::hinge with +/-100 deg limits\n" +
        "pendulum : Joint::distance to the air, springy\n" +
        "\n" +
        "Wired with rb->jointTo(other, def); the world owns\n" +
        "the joints (drawWire shows them); destroying a node\n" +
        "removes its joints automatically.\n" +
        "\n" +
        "click: shove   R: rebuild   drag: orbit",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    // Kick the demos so the constraints show their character.
    if (chainTail) chainTail->getMod<RigidBody>()->body().addVelocity(Vec3(2.5f, 0.0f, 0.8f));
    if (door)      door->getMod<RigidBody>()->body().addVelocity(Vec3(0.0f, 0.0f, 2.5f));
    if (pendulum)  pendulum->getMod<RigidBody>()->body().addVelocity(Vec3(1.8f, 0.0f, 0.6f));
}

void tcApp::keyPressed(int key) {
    if (key == 'R') {
        // Destroying the nodes removes their bodies AND (automatically) every
        // joint touching them — then build a fresh scene.
        for (auto& child : getChildren()) child->destroy();
        door = nullptr;
        pendulum = nullptr;
        chainTail = nullptr;
        buildScene();
    }
}
