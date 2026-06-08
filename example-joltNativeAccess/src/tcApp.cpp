#include "tcApp.h"

// The escape hatch: raw Jolt access for the constraint API the wrapper doesn't
// expose. Including this pulls in Jolt headers (see CMakeLists.txt — we link the
// addon's Jolt target so headers + JPH_* defines match).
#include <tcxPhysicsJolt.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Body/BodyLock.h>

static constexpr int   LINKS = 9;
static constexpr float BOX   = 18.0f;
static constexpr float GAP   = 30.0f;   // centre-to-centre spacing of links
static constexpr float TOP_Y = 360.0f;

// Pin two bodies together with a ball joint at a world-space point, using the
// raw Jolt PhysicsSystem reached via the escape hatch.
static void ballJoint(PhysicsWorld& world, const PhysicsBody& a, const PhysicsBody& b,
                      const Vec3& worldPoint) {
    JPH::PhysicsSystem& sys = joltSystem(world);
    const JPH::BodyLockInterface& lockIf = sys.GetBodyLockInterface();

    JPH::BodyLockWrite la(lockIf, joltBodyId(a));
    JPH::BodyLockWrite lb(lockIf, joltBodyId(b));
    if (!la.Succeeded() || !lb.Succeeded()) return;

    JPH::PointConstraintSettings s;
    s.mSpace  = JPH::EConstraintSpace::WorldSpace;
    s.mPoint1 = s.mPoint2 = JPH::RVec3(worldPoint.x, worldPoint.y, worldPoint.z);

    JPH::Constraint* c = s.Create(la.GetBody(), lb.GetBody());
    sys.AddConstraint(c);   // Jolt ref-counts and owns it from here
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - jolt escape hatch  (ball-jointed chain via raw Jolt)");

    cam.setDistance(640.0f);
    cam.setTarget(0.0f, 180.0f, 0.0f);
    cam.enableMouseInput();

    unitCube = createBox(1.0f);
    linkMat.setBaseColor(0.85f, 0.16f, 0.01f).setMetallic(0.0f).setRoughness(0.45f);
    anchorMat.setBaseColor(0.10f, 0.30f, 0.45f).setMetallic(0.0f).setRoughness(0.6f);

    keyLight.setDirectional(Vec3(-0.4f, -1.0f, -0.6f));
    keyLight.setDiffuse(1.0f, 0.95f, 0.88f);
    keyLight.setIntensity(2.6f);
    addLight(keyLight);

    fillLight.setDirectional(Vec3(0.6f, -0.3f, 0.5f));
    fillLight.setDiffuse(0.6f, 0.6f, 0.65f);
    fillLight.setIntensity(1.1f);
    addLight(fillLight);

    world.setup();
    world.setGravity(Vec3(0.0f, -500.0f, 0.0f));

    // Static anchor at the top (dynamic = false).
    anchor = world.addBox(Vec3(0.0f, TOP_Y, 0.0f), Vec3(BOX, BOX, BOX), false);

    // Dynamic links hanging below, each ball-jointed to the one above.
    PhysicsBody prev = anchor;
    float prevY = TOP_Y;
    for (int i = 0; i < LINKS; ++i) {
        float y = TOP_Y - GAP * (i + 1);
        PhysicsBody link = world.addBox(Vec3(0.0f, y, 0.0f), Vec3(BOX, BOX, BOX));
        ballJoint(world, prev, link, Vec3(0.0f, (prevY + y) * 0.5f, 0.0f));
        links.push_back(link);
        prev = link;
        prevY = y;
    }

    lastTime = getElapsedTimef();
}

void tcApp::update() {
    float t = getElapsedTimef();
    float dt = std::min(0.05f, std::max(0.0f, t - lastTime));
    lastTime = t;
    world.update(dt);
}

void tcApp::draw() {
    clear(0.08f, 0.09f, 0.11f);

    cam.begin();
    setCameraPosition(cam.getPosition());

    auto drawBody = [&](const PhysicsBody& b) {
        if (!b.isValid()) return;
        Vec3 s = b.getSize();
        pushMatrix();
        translate(b.getPosition());
        rotate(b.getRotation());
        scale(s.x, s.y, s.z);
        unitCube.draw();
        popMatrix();
    };

    setMaterial(anchorMat);
    drawBody(anchor);
    clearMaterial();

    setMaterial(linkMat);
    for (const PhysicsBody& b : links) drawBody(b);
    clearMaterial();

    cam.end();

    setColor(1.0f);
    drawBitmapString(
        "ball-jointed chain via <tcxPhysicsJolt.h>\n"
        "links: " + std::to_string((int)links.size()) + "\n" +
        "\n" +
        "click: shove the chain\n" +
        "drag:  orbit camera",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    (void)pos; (void)button;
    // Wrapper-side impulse on the bottom link to set it swinging.
    if (!links.empty() && links.back().isValid())
        links.back().applyImpulse(Vec3(60000.0f, 0.0f, 0.0f));
}
