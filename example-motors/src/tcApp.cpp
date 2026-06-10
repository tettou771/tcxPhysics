#include "tcApp.h"

static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - motors  (driven hinge & slider)");

    cam.setTarget(0.0f, 1.2f, 0.0f);
    cam.setDistance(8.0f);
    cam.setAzimuth(0.4f);
    cam.setElevation(0.25f);
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
    // --- windmill: a paddle on a velocity-motor hinge ------------------------
    Node* post = spawnBody(ColliderShape::box(Vec3(0.15f, 1.6f, 0.15f)),
                           Color(0.35f, 0.37f, 0.40f), Vec3(-1.5f, 0.8f, -0.4f),
                           BodyType::Static);
    // The paddle spins around Z (facing the camera), centred on the post top.
    Vec3 hub(-1.5f, 1.7f, -0.25f);
    Node* paddle = spawnBody(ColliderShape::box(Vec3(2.0f, 0.25f, 0.08f)),
                             Color(0.85f, 0.55f, 0.15f), hub);
    windmillJoint = paddle->getMod<RigidBody>()->jointTo(post,
        Joint::hinge(hub, Vec3(0.0f, 0.0f, 1.0f)).motor(windmillSpeed, 200.0f));

    // --- elevator: a platform on a position-motor slider ---------------------
    // Slider to the world along Y: position 0 = where it spawns; the motor then
    // drives between the two stops (see update()).
    Node* platform = spawnBody(ColliderShape::box(Vec3(1.2f, 0.1f, 1.2f)),
                               Color(0.30f, 0.65f, 0.85f), Vec3(1.6f, 0.3f, 0.0f));
    elevatorJoint = platform->getMod<RigidBody>()->jointToWorld(
        Joint::slider(Vec3(0.0f, 1.0f, 0.0f)).limits(0.0f, 1.8f));
    elevatorJoint.setMotorTarget(0.0f);
    nextElevatorMove = getElapsedTimef() + 2.0f;
    elevatorUp = false;

    rain(6);
}

void tcApp::rain(int n) {
    for (int i = 0; i < n; i++) {
        Color c = Color::fromHSB(random(0.0f, 1.0f), 0.5f, 0.8f);
        // Drop over the windmill and the elevator alternately.
        Vec3 p = (i % 2 == 0)
            ? Vec3(random(-1.9f, -1.1f), random(3.0f, 4.0f), random(-0.5f, 0.0f))
            : Vec3(random(1.2f, 2.0f), random(3.0f, 4.0f), random(-0.4f, 0.4f));
        balls.push_back(spawnBody(ColliderShape::sphere(0.16f), c, p));
    }
}

void tcApp::update() {
    float dt = std::min(0.05f, std::max(0.0f, (float)getDeltaTime()));
    defaultWorld().update(dt);

    // Elevator: bounce between the slider stops every few seconds.
    if (elevatorJoint.isValid() && getElapsedTimef() >= nextElevatorMove) {
        elevatorUp = !elevatorUp;
        elevatorJoint.setMotorTarget(elevatorUp ? 1.8f : 0.0f);
        nextElevatorMove = getElapsedTimef() + 4.0f;
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
        "windmill : hinge + velocity motor (" + std::to_string(windmillSpeed).substr(0, 4) + " rad/s)\n" +
        "elevator : slider + position motor (" + std::string(elevatorUp ? "up" : "down") + ")\n" +
        "\n" +
        "Joint::hinge(hub, axis).motor(speed) spins the paddle;\n" +
        "joint.setMotorTarget(h) drives the platform between\n" +
        "its slider stops. Balls get flung / lifted.\n" +
        "\n" +
        "click: drop balls   SPACE: reverse windmill\n" +
        "R: reset   drag: orbit",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    rain(4);
}

void tcApp::keyPressed(int key) {
    if (key == ' ') {
        windmillSpeed = -windmillSpeed;
        windmillJoint.setMotorVelocity(windmillSpeed);
    }
    if (key == 'R') {
        for (auto& child : getChildren()) child->destroy();
        balls.clear();
        windmillJoint = PhysicsJoint();
        elevatorJoint = PhysicsJoint();
        buildScene();
    }
}
