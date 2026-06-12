#include "tcApp.h"

// Bridge layout: PLANKS planks spanning between two pillars at x = ±SPAN/2.
static constexpr int   PLANKS = 9;
static constexpr float SPAN   = 4.0f;
static constexpr float DECK_Y = 1.6f;
static constexpr float BREAK_FORCE = 4000.0f;   // N — holds the deck, not a falling safe
static constexpr float SAG = 0.18f;              // initial dip at the centre (a flat
                                                 // chain would need near-infinite tension)

static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - breakable  (joints that snap under load)");

    cam.setTarget(0.0f, 1.0f, 0.0f);
    cam.setDistance(7.5f);
    cam.setAzimuth(0.35f);
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

    // Mark every snap (and count them) — the joint is already gone here.
    breakL = defaultWorld().jointBroke.listen([this](JointBreakEventArgs& e) {
        marks.push_back({e.point, 0.0f});
        breakCount++;
        lastBreakForce = e.force;
    });

    buildBridge();
}

Node* tcApp::spawnBody(const ColliderShape& shape, const Color& color,
                       const Vec3& pos, BodyType type, float density) {
    auto n = make_shared<Node>();
    n->setPos(pos);
    addChild(n);
    n->addMod<RigidBody>(shape, type, density);
    n->addMod<ColliderRenderer>()->setColor(color);
    return n.get();
}

// Deck height along the span: dips by SAG at the centre (shallow parabola).
static float deckY(float x) {
    float t = x / (SPAN * 0.5f);
    return DECK_Y - SAG * (1.0f - t * t);
}

void tcApp::buildBridge() {
    float half = SPAN * 0.5f;
    Node* pillarL = spawnBody(ColliderShape::box(Vec3(0.2f, DECK_Y, 0.7f)),
                              Color(0.35f, 0.37f, 0.40f), Vec3(-half - 0.1f, DECK_Y * 0.5f, 0.0f),
                              BodyType::Static);
    Node* pillarR = spawnBody(ColliderShape::box(Vec3(0.2f, DECK_Y, 0.7f)),
                              Color(0.35f, 0.37f, 0.40f), Vec3( half + 0.1f, DECK_Y * 0.5f, 0.0f),
                              BodyType::Static);

    // Wooden planks (density 300) joined end-to-end with BREAKABLE point
    // joints, hung with an initial sag — a perfectly flat chain would need
    // near-infinite tension just to hold itself.
    float pitch = SPAN / PLANKS;                  // centre-to-centre spacing
    float plankW = pitch * 0.88f;                 // small gap between planks
    Node* prev = pillarL;
    for (int i = 0; i < PLANKS; i++) {
        float x = -half + pitch * (i + 0.5f);
        Node* plank = spawnBody(ColliderShape::box(Vec3(plankW, 0.07f, 0.6f)),
                                Color(0.62f, 0.45f, 0.25f), Vec3(x, deckY(x), 0.0f),
                                BodyType::Dynamic, 300.0f);
        // Junction at the shared edge, snapping above BREAK_FORCE newtons.
        float jx = x - pitch * 0.5f;
        plank->getMod<RigidBody>()->jointTo(prev,
            Joint::point(Vec3(jx, deckY(jx), 0.0f)).breakForce(BREAK_FORCE));
        prev = plank;
    }
    // Last plank to the right pillar.
    prev->getMod<RigidBody>()->jointTo(pillarR,
        Joint::point(Vec3(half, deckY(half), 0.0f)).breakForce(BREAK_FORCE));
}

void tcApp::update() {
    float dt = std::min(0.05f, std::max(0.0f, (float)getDeltaTime()));
    defaultWorld().update(dt);

    for (auto& m : marks) m.age += dt;
    while (!marks.empty() && marks.front().age > 3.0f) marks.erase(marks.begin());

}

void tcApp::beginDraw() {
    clear(0.08f, 0.09f, 0.11f);
    cam.begin();
    setCameraPosition(cam.getPosition());
}

void tcApp::draw() {
    drawFloorGrid(3.0f, 0.5f);
    for (auto& j : defaultWorld().getJoints()) j.drawWire();

    // Snap markers: red boxes fading out where joints broke.
    for (auto& m : marks) {
        float a = 1.0f - m.age / 3.0f;
        setColor(1.0f, 0.2f, 0.15f, a);
        drawBox(m.pos, 0.1f);
    }
}

void tcApp::endDraw() {
    cam.end();
    setColor(1.0f);
    drawBitmapString(
        "breaks: " + std::to_string(breakCount) +
        (breakCount > 0 ? "   last: " + std::to_string((int)lastBreakForce) + " N" : "") + "\n" +
        "\n" +
        "Every junction: Joint::point(p).breakForce(" + std::to_string((int)BREAK_FORCE) + ")\n" +
        "Drop weights until the tension exceeds the\n" +
        "threshold - jointBroke fires (red marks) and\n" +
        "the bridge tears at its weakest link.\n" +
        "\n" +
        "click: drop a heavy ball   R: rebuild\n" +
        "drag: orbit",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    // A dense ball (~140 kg) somewhere over the deck.
    spawnBody(ColliderShape::sphere(0.32f),
              Color(0.30f, 0.65f, 0.85f),
              Vec3(random(-SPAN * 0.4f, SPAN * 0.4f), 4.0f, random(-0.2f, 0.2f)));
}

void tcApp::keyPressed(int key) {
    if (key == 'R') {
        for (auto& child : getChildren()) child->destroy();
        marks.clear();
        breakCount = 0;
        lastBreakForce = 0.0f;
        buildBridge();
    }
}
