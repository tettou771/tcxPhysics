#include "tcApp.h"

static constexpr float WALK_SPEED = 4.0f;   // m/s
static constexpr float JUMP_SPEED = 5.0f;   // m/s

static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - character  (WASD walk, SPACE jump)");

    cam.setDistance(7.0f);
    cam.setAzimuth(0.0f);
    cam.setElevation(0.45f);
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
                       const Vec3& pos, BodyType type, float density, const Vec3& euler) {
    auto n = make_shared<Node>();
    n->setPos(pos);
    n->setEuler(euler);
    addChild(n);
    n->addMod<RigidBody>(shape, type, density);
    n->addMod<ColliderRenderer>()->setColor(color);
    return n.get();
}

void tcApp::buildScene() {
    // The player.
    auto p = make_shared<Player>(Vec3(0.0f, 1.0f, 0.0f));
    player = p.get();
    addChild(p);

    // A walkable 20 deg ramp (+X) and a 60 deg cliff behind it (too steep).
    spawnBody(ColliderShape::box(Vec3(3.0f, 0.15f, 2.2f)), Color(0.45f, 0.55f, 0.40f),
              Vec3(3.4f, 0.50f, 0.0f), BodyType::Static, 1000.0f, Vec3(0.0f, 0.0f, 0.349f));
    spawnBody(ColliderShape::box(Vec3(2.4f, 0.15f, 2.2f)), Color(0.55f, 0.40f, 0.40f),
              Vec3(6.2f, 1.95f, 0.0f), BodyType::Static, 1000.0f, Vec3(0.0f, 0.0f, 1.047f));

    // Stairs (-X): five 0.25 m risers — stepped up without jumping.
    for (int i = 0; i < 5; i++) {
        float h = 0.25f * (i + 1);
        spawnBody(ColliderShape::box(Vec3(0.6f, h, 1.6f)), Color(0.45f, 0.47f, 0.52f),
                  Vec3(-2.2f - 0.6f * i, h * 0.5f, 0.0f), BodyType::Static);
    }

    // A kinematic platform sliding side to side (+Z side) — step on and ride.
    platform = spawnBody(ColliderShape::box(Vec3(1.4f, 0.15f, 1.4f)),
                         Color(0.30f, 0.65f, 0.85f), Vec3(0.0f, 0.35f, 3.0f),
                         BodyType::Kinematic);

    // Light crates to shove through (-Z side).
    for (int i = 0; i < 4; i++)
        spawnBody(ColliderShape::box(Vec3(0.45f)), Color::fromHSB(0.08f + i * 0.04f, 0.5f, 0.8f),
                  Vec3(-0.8f + 0.55f * i, 0.3f, -2.5f), BodyType::Dynamic, 200.0f);
}

void tcApp::update() {
    // WASD -> camera-relative horizontal velocity.
    if (player && player->body()) {
        float az = cam.getAzimuth();
        Vec3 fwd(-sinf(az), 0.0f, -cosf(az));            // where the camera looks
        Vec3 right(cosf(az), 0.0f, -sinf(az));
        float ix = (dDown ? 1.0f : 0.0f) - (aDown ? 1.0f : 0.0f);
        float iz = (wDown ? 1.0f : 0.0f) - (sDown ? 1.0f : 0.0f);
        Vec3 dir = fwd * iz + right * ix;
        if (dir.length() > 0.001f) dir = dir.normalized();
        player->move(dir * WALK_SPEED);
    }

    // Slide the platform back and forth.
    if (platform) platform->setPos(sinf(getElapsedTimef() * 0.8f) * 1.8f, 0.35f, 3.0f);

    float dt = std::min(0.05f, std::max(0.0f, (float)getDeltaTime()));
    defaultWorld().update(dt);

}

void tcApp::beginDraw() {
    // Aim the camera HERE, not in update(): by draw time the player node has
    // been synced to the character's post-step position, so the camera never
    // lags a frame behind (visible as a 1-frame pop when stepping up stairs).
    if (player) cam.setTarget(player->getPos() + Vec3(0.0f, 0.6f, 0.0f));

    clear(0.08f, 0.09f, 0.11f);
    cam.begin();
    setCameraPosition(cam.getPosition());
}

void tcApp::draw() {
    drawFloorGrid(8.0f, 1.0f);
}

void tcApp::endDraw() {
    cam.end();
    setColor(1.0f);
    bool grounded = player && player->body() && player->body()->isGrounded();
    drawBitmapString(
        std::string("grounded: ") + (grounded ? "yes" : "no") + "\n" +
        "\n" +
        "ramp (20 deg): walk up   cliff (60 deg): slides off\n" +
        "stairs: stepped without jumping   blue platform: ride it\n" +
        "crates: shove through\n" +
        "\n" +
        "WASD: walk (camera-relative)   SPACE: jump\n" +
        "R: reset   drag: orbit",
        20.0f, 20.0f);
}

void tcApp::keyPressed(int key) {
    if (key == 'W') wDown = true;
    if (key == 'A') aDown = true;
    if (key == 'S') sDown = true;
    if (key == 'D') dDown = true;
    if (key == ' ' && player && player->body()) player->body()->jump(JUMP_SPEED);
    if (key == 'R') {
        for (auto& child : getChildren()) child->destroy();
        player = nullptr;
        platform = nullptr;
        buildScene();
    }
}

void tcApp::keyReleased(int key) {
    if (key == 'W') wDown = false;
    if (key == 'A') aDown = false;
    if (key == 'S') sDown = false;
    if (key == 'D') dDown = false;
}
