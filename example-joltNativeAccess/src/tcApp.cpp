#include "tcApp.h"

// The escape hatch: raw Jolt access for features the wrapper does not expose —
// here, a path constraint. Including this pulls in Jolt headers (see
// local.cmake — we link the addon's Jolt target so headers + JPH_* defines match).
#include <tcxPhysicsJolt.h>
#include <Jolt/Physics/Constraints/PathConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>
#include <Jolt/Physics/Body/BodyLock.h>

// The track: a closed loop of radius R, tilted so gravity keeps the bead moving.
static constexpr int   TRACK_SEGMENTS = 24;
static constexpr float TRACK_RADIUS   = 1.4f;
static constexpr float TRACK_TILT     = 0.45f;   // radians around X
static const     Vec3  TRACK_CENTER(0.0f, 1.6f, 0.0f);

// Point on the loop at angle a (world space).
static Vec3 trackPoint(float a) {
    // Circle in XZ, then tilt around X: y' = -z sin(t), z' = z cos(t).
    float x = cosf(a) * TRACK_RADIUS;
    float z = sinf(a) * TRACK_RADIUS;
    float y = -z * sinf(TRACK_TILT);
    z       =  z * cosf(TRACK_TILT);
    return TRACK_CENTER + Vec3(x, y, z);
}

// Attach `bead` to a closed Hermite loop with a raw Jolt PathConstraint
// (body 1 = the world, body 2 = the bead).
static void pathConstraint(PhysicsWorld& world, const PhysicsBody& bead) {
    JPH::PhysicsSystem& sys = joltSystem(world);

    auto* path = new JPH::PathConstraintPathHermite();
    for (int i = 0; i < TRACK_SEGMENTS; i++) {
        float a  = TAU * i / TRACK_SEGMENTS;
        Vec3 p   = trackPoint(a);
        // Tangent = d(point)/d(angle) * the angle step (Hermite segment scale).
        float da = TAU / TRACK_SEGMENTS;
        Vec3 t   = (trackPoint(a + 0.001f) - trackPoint(a - 0.001f)) * (da / 0.002f);
        JPH::Vec3 normal(0.0f, cosf(TRACK_TILT), sinf(TRACK_TILT));   // loop plane normal
        path->AddPoint(JPH::Vec3(p.x, p.y, p.z), JPH::Vec3(t.x, t.y, t.z), normal);
    }
    path->SetIsLooping(true);

    const JPH::BodyLockInterface& lockIf = sys.GetBodyLockInterface();
    JPH::BodyLockWrite lb(lockIf, joltBodyId(bead));
    if (!lb.Succeeded()) return;

    JPH::PathConstraintSettings s;
    s.mPath = path;                                  // ref-counted from here
    s.mPathPosition = JPH::Vec3::sZero();            // path points are world-space
    s.mRotationConstraintType = JPH::EPathRotationConstraintType::ConstrainToPath;
    // Start fraction = wherever the bead is. Read the position THROUGH the held
    // lock — calling the wrapper here (bead.getPosition()) would try to re-lock
    // the same body and deadlock.
    JPH::Vec3 beadPos(lb.GetBody().GetPosition());
    s.mPathFraction = path->GetClosestPoint(beadPos, 0.0f);

    JPH::Constraint* c = s.Create(JPH::Body::sFixedToWorld, lb.GetBody());
    sys.AddConstraint(c);   // Jolt ref-counts and owns it from here
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - joltNativeAccess  (raw Jolt: path constraint)");

    cam.setTarget(TRACK_CENTER);
    cam.setDistance(5.5f);
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

    world.setup();

    // The bead starts on the track (the wrapper makes the body; raw Jolt rails it).
    bead = world.addBox(trackPoint(0.0f), Vec3(0.22f, 0.14f, 0.14f));
    pathConstraint(world, bead);

    beadMesh = createBox(1.0f);
    beadMat.setBaseColor(0.85f, 0.16f, 0.01f).setMetallic(0.0f).setRoughness(0.45f);

    // Sample the loop once for drawing.
    for (int i = 0; i <= TRACK_SEGMENTS * 4; i++)
        trackPoints.push_back(trackPoint(TAU * i / (TRACK_SEGMENTS * 4)));

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

    // The track.
    setColor(0.35f, 0.55f, 0.75f);
    for (size_t i = 1; i < trackPoints.size(); i++)
        drawLine(trackPoints[i - 1], trackPoints[i]);

    // The bead (oriented to the path by the constraint).
    if (bead.isValid()) {
        Vec3 s = bead.getSize();
        pushMatrix();
        translate(bead.getPosition());
        rotate(bead.getRotation());
        scale(s.x, s.y, s.z);
        setMaterial(beadMat);
        beadMesh.draw();
        clearMaterial();
        popMatrix();
    }

    cam.end();

    setColor(1.0f);
    drawBitmapString(
        "raw Jolt via <tcxPhysicsJolt.h>: a PATH CONSTRAINT\n"
        "(not wrapped) rails the bead onto a closed Hermite\n"
        "loop. Gravity alone drives it around the tilted track.\n"
        "\n"
        "click: push the bead\n"
        "drag:  orbit camera",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    // Mass-independent shove along the world X axis — the constraint converts
    // whatever component is tangent to the track into speed along it.
    if (bead.isValid()) bead.addVelocity(Vec3(3.0f, 0.0f, 0.0f));
}
