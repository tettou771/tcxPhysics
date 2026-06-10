#include "tcApp.h"

static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - gears  (gear & rack-and-pinion)");

    cam.setTarget(0.0f, 1.0f, 0.0f);
    cam.setDistance(7.0f);
    cam.setAzimuth(0.3f);
    cam.setElevation(0.2f);
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
    n->addMod<RigidBody>(shape, type);
    n->addMod<ColliderRenderer>()->setColor(color);
    return n.get();
}

void tcApp::buildScene() {
    // Two posts carrying two paddle wheels (hinges around Z, facing the camera).
    Node* postA = spawnBody(ColliderShape::box(Vec3(0.12f, 1.2f, 0.12f)),
                            Color(0.35f, 0.37f, 0.40f), Vec3(-0.8f, 0.6f, -0.3f),
                            BodyType::Static);
    Node* postB = spawnBody(ColliderShape::box(Vec3(0.12f, 1.2f, 0.12f)),
                            Color(0.35f, 0.37f, 0.40f), Vec3(0.8f, 0.6f, -0.3f),
                            BodyType::Static);

    Vec3 hubA(-0.8f, 1.4f, -0.15f), hubB(0.8f, 1.4f, -0.15f);
    Node* wheelA = spawnBody(ColliderShape::box(Vec3(1.1f, 0.18f, 0.07f)),
                             Color(0.85f, 0.55f, 0.15f), hubA);
    Node* wheelB = spawnBody(ColliderShape::box(Vec3(1.1f, 0.18f, 0.07f)),
                             Color(0.30f, 0.65f, 0.85f), hubB);

    // Wheel A is the engine: hinge + velocity motor.
    engineJoint = wheelA->getMod<RigidBody>()->jointTo(postA,
        Joint::hinge(hubA, Vec3(0.0f, 0.0f, 1.0f)).motor(engineSpeed, 300.0f));
    // Wheel B: a free hinge...
    PhysicsJoint hingeB = wheelB->getMod<RigidBody>()->jointTo(postB,
        Joint::hinge(hubB, Vec3(0.0f, 0.0f, 1.0f)));
    // ...locked to wheel A through a GEAR (2:1 — B turns half as fast).
    defaultWorld().addGearJoint(engineJoint, hingeB, 2.0f);

    // The rack: a bar on a world slider along X, driven by wheel A through a
    // rack-and-pinion (one wheel turn = ~0.5 m of travel).
    Node* rackBar = spawnBody(ColliderShape::box(Vec3(1.6f, 0.12f, 0.12f)),
                              Color(0.55f, 0.80f, 0.40f), Vec3(-0.8f, 0.25f, 0.6f));
    PhysicsJoint rackJoint = rackBar->getMod<RigidBody>()->jointToWorld(
        Joint::slider(Vec3(1.0f, 0.0f, 0.0f)).limits(-1.2f, 1.2f));
    defaultWorld().addRackAndPinionJoint(engineJoint, rackJoint, TAU / 0.5f);

    nextReverse = getElapsedTimef() + 4.0f;
}

void tcApp::update() {
    float dt = std::min(0.05f, std::max(0.0f, (float)getDeltaTime()));
    defaultWorld().update(dt);

    if (getElapsedTimef() >= nextReverse) {
        engineSpeed = -engineSpeed;
        engineJoint.setMotorVelocity(engineSpeed);
        nextReverse = getElapsedTimef() + 4.0f;
    }
}

void tcApp::beginDraw() {
    clear(0.08f, 0.09f, 0.11f);
    cam.begin();
    setCameraPosition(cam.getPosition());
}

void tcApp::draw() {
    drawFloorGrid(3.0f, 0.5f);
    for (auto& j : defaultWorld().getJoints()) j.drawWire();
}

void tcApp::endDraw() {
    cam.end();
    setColor(1.0f);
    drawBitmapString(
        "engine " + std::string(engineSpeed > 0 ? "+" : "-") + " | one motor drives everything:\n" +
        "\n" +
        "wheel A : hinge + motor (the engine)\n" +
        "wheel B : addGearJoint(A, B, 2)  - counter-rotates at half speed\n" +
        "rack    : addRackAndPinionJoint(A, rack) - shuttles on its slider\n" +
        "\n" +
        "click: drop balls   SPACE: reverse now   R: reset\n" +
        "drag: orbit",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    for (int i = 0; i < 4; i++) {
        Color c = Color::fromHSB(random(0.0f, 1.0f), 0.5f, 0.8f);
        spawnBody(ColliderShape::sphere(0.12f), c,
                  Vec3(random(-1.2f, 1.2f), random(2.5f, 3.5f), random(-0.3f, 0.6f)));
    }
}

void tcApp::keyPressed(int key) {
    if (key == ' ') {
        engineSpeed = -engineSpeed;
        engineJoint.setMotorVelocity(engineSpeed);
        nextReverse = getElapsedTimef() + 4.0f;
    }
    if (key == 'R') {
        for (auto& child : getChildren()) child->destroy();
        engineJoint = PhysicsJoint();
        buildScene();
    }
}
