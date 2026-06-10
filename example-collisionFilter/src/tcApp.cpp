#include "tcApp.h"

// Layer assignments. The ground keeps the default layer 0.
static constexpr int      LAYER_RED  = 1;
static constexpr int      LAYER_BLUE = 2;
static constexpr uint32_t BIT_GROUND = 1u << 0;
static constexpr uint32_t BIT_RED    = 1u << LAYER_RED;
static constexpr uint32_t BIT_BLUE   = 1u << LAYER_BLUE;

static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - collisionFilter  (layers / masks / user data)");

    cam.setTarget(0.0f, 0.8f, 0.0f);
    cam.setDistance(6.5f);
    cam.setAzimuth(0.5f);
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

    defaultWorld().setup();
    defaultWorld().addGroundPlane(0.0f);

    // Caption the latest ball-vs-ball contact via the user-data tags.
    contactL = defaultWorld().contactBegan.listen([this](ContactEventArgs& c) {
        uint64_t ua = c.a.getUserData(), ub = c.b.getUserData();
        if (ua == 0 || ub == 0) return;   // ground has no tag
        auto name = [](uint64_t s) {
            return std::string(s % 2 == 0 ? "red #" : "blue #") + std::to_string((int)s);
        };
        lastContact = name(ua) + "  x  " + name(ub);
    });

    rain(16);
}

void tcApp::rain(int n) {
    for (int i = 0; i < n; i++) {
        int  serial = nextSerial++;
        bool red    = (serial % 2 == 0);
        auto node   = make_shared<Node>();
        node->setPos(random(-1.2f, 1.2f), random(2.0f, 4.0f), random(-1.2f, 1.2f));
        addChild(node);
        auto* rb = node->addMod<RigidBody>(ColliderShape::sphere(0.18f));
        rb->setCollisionLayer(red ? LAYER_RED : LAYER_BLUE);
        rb->body().setUserData((uint64_t)serial);
        node->addMod<ColliderRenderer>()->setColor(
            red ? Color(0.85f, 0.25f, 0.20f) : Color(0.25f, 0.45f, 0.90f));
        balls.push_back(node.get());
    }
    applyMasks();
}

void tcApp::applyMasks() {
    for (size_t i = 0; i < balls.size(); i++) {
        auto* rb = balls[i]->getMod<RigidBody>();
        if (!rb) continue;
        bool red = (rb->body().getUserData() % 2 == 0);
        uint32_t mask = filtered ? (BIT_GROUND | (red ? BIT_RED : BIT_BLUE)) : 0xffu;
        rb->setCollisionMask(mask);
        rb->body().activate();   // wake everyone so the new filter takes effect
    }
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
    drawFloorGrid(2.5f, 0.5f);
}

void tcApp::endDraw() {
    cam.end();
    setColor(1.0f);
    drawBitmapString(
        std::string("filter: ") + (filtered ? "ON  (red and blue pass through each other)"
                                            : "OFF (everything collides)") + "\n" +
        "last contact: " + lastContact + "\n" +
        "\n" +
        "Each ball: setCollisionLayer(1 or 2) + a mask of\n" +
        "layers it hits; both sides must agree to collide.\n" +
        "setUserData(serial) tags bodies so contact events\n" +
        "can name who touched whom.\n" +
        "\n" +
        "click: drop balls   SPACE: toggle filter   R: reset\n" +
        "drag: orbit",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    rain(8);
}

void tcApp::keyPressed(int key) {
    if (key == ' ') {
        filtered = !filtered;
        applyMasks();
    }
    if (key == 'R') {
        for (auto& child : getChildren()) child->destroy();
        balls.clear();
        nextSerial = 1;
        lastContact = "(none yet)";
        rain(16);
    }
}
