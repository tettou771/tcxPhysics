#include "tcApp.h"

static void drawFloorGrid(float halfExtent, float step) {
    setColor(0.25f, 0.27f, 0.30f);
    for (float a = -halfExtent; a <= halfExtent + 0.001f; a += step) {
        drawLine(Vec3(a, 0.0f, -halfExtent), Vec3(a, 0.0f, halfExtent));
        drawLine(Vec3(-halfExtent, 0.0f, a), Vec3(halfExtent, 0.0f, a));
    }
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - raycast  (mouse picking)");

    cam.setTarget(0.0f, 0.4f, 0.0f);
    cam.setDistance(7.0f);
    cam.setAzimuth(0.6f);
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

    populate();
}

void tcApp::populate() {
    picks.clear();
    // A tidy grid of mixed shapes resting on the ground — easy targets to pick.
    const int n = 3;
    for (int ix = 0; ix < n; ix++) {
        for (int iz = 0; iz < n; iz++) {
            float x = (ix - (n - 1) * 0.5f) * 0.9f;
            float z = (iz - (n - 1) * 0.5f) * 0.9f;
            int kind = (ix * n + iz) % 4;
            ColliderShape shape = ColliderShape::box(Vec3(0.36f));
            if (kind == 1) shape = ColliderShape::sphere(0.22f);
            if (kind == 2) shape = ColliderShape::capsule(0.16f, 0.28f);
            if (kind == 3) shape = ColliderShape::cylinder(0.2f, 0.4f);
            Color base = Color::fromHSB((ix * n + iz) / (float)(n * n), 0.55f, 0.7f);
            auto p = make_shared<Pickable>(shape, base, Vec3(x, 0.6f, z));
            picks.push_back(p);
            addChild(p);
        }
    }
}

void tcApp::updatePick() {
    hovered = nullptr;
    hover = RaycastHit();
    if (picks.empty()) return;

    // The pickables are drawn under the EasyCam, so each carries that camera's
    // immutable context (stamped at draw time) — exactly the matrices needed to
    // unproject the cursor, and valid even here in update(). This is the same path
    // the node inspector uses to drag objects, so it's right for any camera.
    auto ctx = picks.front()->getCameraContext();
    if (!ctx) return;   // nothing drawn yet (first frame)

    Ray ray = ctx->screenPointToRay(getMouseX(), getMouseY());
    rayOrigin = ray.origin;
    rayDir    = ray.direction;

    hover = defaultWorld().raycast(ray, 100.0f);
    if (hover) {
        for (auto& p : picks)
            if (p->bodyId() == hover.body.getId()) { hovered = p.get(); break; }
    }
    for (auto& p : picks) p->setHovered(p.get() == hovered);
}

void tcApp::update() {
    updatePick();
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

    // Visualize the hit: a marker at the impact point and its surface normal.
    if (hover) {
        setColor(1.0f, 0.85f, 0.2f);
        drawLine(hover.point, hover.point + hover.normal * 0.6f);   // surface normal
        drawBox(hover.point, 0.12f);                                // hit marker
    }
}

void tcApp::endDraw() {
    cam.end();
    setColor(1.0f);
    std::string target = hovered ? "body" : (hover ? "ground" : "(nothing)");
    drawBitmapString(
        "ray hit: " + target + "\n" +
        "\n" +
        "A ray is cast from the camera through the mouse\n" +
        "each frame (PhysicsWorld::raycast). The body under\n" +
        "the cursor lights up; the yellow marker + line show\n" +
        "the hit point and surface normal.\n" +
        "\n" +
        "move: hover   SPACE: shoot hovered body\n" +
        "click: drop bodies   R: reset   drag: orbit",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    // Drop a fresh body above the cursor's ground position. Same camera-context
    // ray as picking, intersected with the floor plane (Y = 0).
    Vec3 spawn(0.0f, 3.0f, 0.0f);
    auto ctx = picks.empty() ? nullptr : picks.front()->getCameraContext();
    if (ctx) {
        Ray r = ctx->screenPointToRay(pos.x, pos.y);
        float t; Vec3 hitPt;
        if (r.intersectPlane(Vec3(0.0f, 1.0f, 0.0f), 0.0f, t, hitPt))
            spawn = Vec3(hitPt.x, 3.0f, hitPt.z);
    }
    Color base = Color::fromHSB(random(0.0f, 1.0f), 0.55f, 0.7f);
    auto p = make_shared<Pickable>(ColliderShape::box(Vec3(0.34f)), base, spawn);
    picks.push_back(p);
    addChild(p);
}

void tcApp::keyPressed(int key) {
    if (key == ' ' && hovered) {
        // Shove the hovered body away along the ray (mass-independent kick).
        hovered->body().addVelocity(rayDir * 6.0f);
    }
    if (key == 'R') {
        for (auto& p : picks) p->destroy();
        populate();
    }
}
