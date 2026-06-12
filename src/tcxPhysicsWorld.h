#pragma once

#include <TrussC.h>
#include <memory>
#include <cstdint>
#include <vector>
#include "tcxPhysicsBody.h"
#include "tcxPhysicsJoint.h"
#include "tcxPhysicsCharacter.h"

namespace tcx {

// =============================================================================
// ContactEventArgs - payload for PhysicsWorld::contactBegan / contactEnded.
//
// A value snapshot built on the MAIN thread right after the step (Jolt detects
// contacts on worker threads; we buffer them and replay them safely here). The
// two bodies are copyable handles, so a listener may freely read transforms,
// apply impulses, queue removals, etc.
// =============================================================================
struct ContactEventArgs {
    PhysicsBody a;        // the two bodies in contact (handles; always safe to copy)
    PhysicsBody b;
    tc::Vec3 point;       // world-space contact point (set on contactBegan; zero on ended)
    tc::Vec3 normal;      // world-space contact normal, from a toward b (began only)
    float speed = 0.0f;   // relative approach speed along the normal at impact
                          // (began only) — handy for impact sound / vfx intensity
};

// How a body moves under the simulation.
//   Static    - never moves (floor, walls).
//   Kinematic - moved by YOU (setPosition / moveKinematic); pushes dynamics but
//               ignores forces, gravity and collisions back on itself.
//   Dynamic   - fully simulated (falls, collides, responds to forces).
enum class MotionType { Static, Kinematic, Dynamic };

// =============================================================================
// RaycastHit - result of PhysicsWorld::raycast().
//
// `hit` is false when the ray reached maxDistance without touching anything; the
// struct is also contextually convertible to bool, so `if (auto h = world.raycast(...))`
// reads cleanly. On a hit, `body` is the body struck, `point` the world-space
// impact, `normal` its outward surface normal, `distance` how far along the ray.
// =============================================================================
struct RaycastHit {
    bool        hit = false;
    PhysicsBody body;
    tc::Vec3    point;
    tc::Vec3    normal;
    float       distance = 0.0f;

    explicit operator bool() const { return hit; }
};

// =============================================================================
// PhysicsWorld - owns the Jolt simulation and acts as a factory for bodies.
//
// Lifecycle:
//   setup()  once (e.g. in App::setup)
//   update() once per frame (e.g. in App::update)
//   addBox / addSphere / addGroundPlane to populate it
//
// Units are whatever your scene uses — Jolt itself is unit-agnostic. The default
// gravity is the physical -9.81 on Y; for small scenes you'll usually want a
// punchier value via setGravity() so things fall at a lively pace.
// =============================================================================
class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();
    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    // Initialize the simulation. Call once before adding bodies.
    //
    //   maxBodies              - cap on rigid bodies alive at once.
    //   maxBodyPairs           - broadphase collision-pair buffer (0 = auto).
    //   maxContactConstraints  - contact-constraint buffer (0 = auto).
    //
    // The two buffers default to generous values scaled from maxBodies. They
    // matter for dense piles: if either fills up, Jolt drops contacts and bodies
    // tunnel through each other and the floor. Raise them if you see that with
    // very large/dense scenes.
    void setup(int maxBodies = 10240, int maxBodyPairs = 0, int maxContactConstraints = 0);

    PhysicsWorld& setGravity(const tc::Vec3& gravity);
    tc::Vec3 getGravity() const;

    // Advance the simulation by dt seconds. collisionSteps subdivides the step
    // for stability at large dt (1 is fine for ~60fps).
    //
    // Use this OR the async stepping below — not both. While async is running,
    // update() is ignored (with a warning).
    void update(float dt = 1.0f / 60.0f, int collisionSteps = 1);

    // --- stepping mode -------------------------------------------------------
    // By default you drive the sim yourself with update(dt) once per frame.
    //
    // Alternatively, run the simulation on its own fixed-timestep clock so the
    // physics stays stable independent of frame rate / hitches:
    //   world.updateAsyncStart(120);   // step at 120 Hz on a background thread
    //   ...
    //   world.updateAsyncStop();
    // Body reads and force/velocity calls stay safe while async runs — they are
    // serialized against the step. Collision events still fire on the MAIN
    // thread (drained on the frame loop), so handlers can touch app state.
    //
    // Web (wasm) has no background threads: async there transparently falls back
    // to fixed-timestep stepping driven by the frame loop (events().update),
    // logged once as a warning. Same API, no code change needed.
    void updateAsyncStart(float hz = 120.0f);
    void updateAsyncStop();
    bool isAsync() const;

    // --- collision events ----------------------------------------------------
    // Fired on the MAIN thread (inside update(), or — in async mode — on the
    // frame's events().update) so handlers can safely touch app / render state
    // even though Jolt detects contacts on worker threads.
    //
    //   listener = world.contactBegan.listen([](ContactEventArgs& c){ ... });
    tc::Event<ContactEventArgs> contactBegan;      // two bodies started touching
    tc::Event<ContactEventArgs> contactPersisted;  // still touching (fires every step — can be heavy)
    tc::Event<ContactEventArgs> contactEnded;      // two bodies stopped touching

    // --- body factory --------------------------------------------------------
    // dynamic = true  -> falls and collides (a thrown block)
    // dynamic = false -> never moves (floor, walls, static scenery)
    // size is the FULL box size (w, h, d); position is its center.
    //
    // mass = density * volume. The default density is 1000 (water, kg/m^3) — so
    // build at roughly METRE scale and the numbers feel real: a 0.3 m cube weighs
    // ~27 kg, default gravity -9.81 looks natural. (Jolt is unit-agnostic; 1000
    // is its native default. See the README on scale.)
    PhysicsBody addBox(const tc::Vec3& position, const tc::Vec3& size, bool dynamic = true, float density = 1000.0f);
    PhysicsBody addSphere(const tc::Vec3& position, float radius, bool dynamic = true, float density = 1000.0f);

    // A capsule aligned to the Y axis: a cylinder of `cylinderHeight` capped by
    // two hemispheres of `radius` (total height = cylinderHeight + 2*radius).
    // Draw it with tc::createCapsule(radius, cylinderHeight). Great for characters.
    PhysicsBody addCapsule(const tc::Vec3& position, float radius, float cylinderHeight, bool dynamic = true, float density = 1000.0f);
    // A cylinder aligned to the Y axis. Draw with tc::createCylinder(radius, height).
    PhysicsBody addCylinder(const tc::Vec3& position, float radius, float height, bool dynamic = true, float density = 1000.0f);

    // A convex body built as the convex hull of a mesh's vertices. Use for
    // arbitrary *convex* dynamic shapes (gems, dice, rocks). The mesh is only
    // sampled for its points — keep your own copy to draw. Concave detail is lost
    // (it's the hull); for concave geometry use addMesh (static).
    PhysicsBody addConvexHull(const tc::Vec3& position, const tc::Mesh& mesh, bool dynamic = true, float density = 1000.0f);

    // A triangle-mesh body for arbitrary (incl. concave) geometry — terrain,
    // level scenery. STATIC ONLY: Jolt can't make a mesh shape dynamic (no mass
    // properties), so `dynamic` is ignored with a warning if true. Draw the same
    // mesh yourself at the body's transform.
    PhysicsBody addMesh(const tc::Vec3& position, const tc::Mesh& mesh, bool dynamic = false);

    // A large flat static box acting as the ground, centered on (0, y, 0).
    PhysicsBody addGroundPlane(float y = 0.0f, float size = 100000.0f);

    // --- joints ----------------------------------------------------------------
    // Constrain two bodies to each other (or one body to the world). The world
    // owns every joint; the returned PhysicsJoint is a lightweight handle (copy
    // freely, query any time). Describe the joint with the Joint factories:
    //
    //   auto j = world.addJoint(frame, door, Joint::hinge(edge, {0,1,0}).limits(-1.2f, 1.2f));
    //   auto p = world.addJoint(ball, Joint::distance(ballPos, ceiling).spring(2, 0.2f));
    //
    // ORDER MATTERS for hinge/slider signs: `a` is the BASE, `b` is the side
    // that moves positively (limits, angles, motor velocity/target are measured
    // of b relative to a). The one-body overload joints `a` to the world with
    // the world as the base, so positive values move `a`.
    //
    // A joint is removed explicitly (removeJoint) or AUTOMATICALLY when either
    // of its bodies is removed — it can never dangle.
    PhysicsJoint addJoint(const PhysicsBody& a, const PhysicsBody& b, const Joint& def);
    PhysicsJoint addJoint(const PhysicsBody& a, const Joint& def);   // a <-> the world

    // Link two EXISTING joints into a transmission:
    //   gear          - two hinges spin in lockstep. ratio = teethB / teethA
    //                   (wheel A turns ratio times for one turn of wheel B).
    //   rackAndPinion - a hinge (pinion) drives a slider (rack) and vice versa.
    //                   ratio = radians of pinion rotation per metre of rack travel.
    PhysicsJoint addGearJoint(const PhysicsJoint& hingeA, const PhysicsJoint& hingeB, float ratio = 1.0f);
    PhysicsJoint addRackAndPinionJoint(const PhysicsJoint& pinionHinge, const PhysicsJoint& rackSlider, float ratio);

    void removeJoint(const PhysicsJoint& joint);
    std::vector<PhysicsJoint> getJoints() const;                     // all live joints
    std::vector<PhysicsJoint> getJointsForBody(uint32_t bodyId) const;

    // Fired (main thread) when a breakable joint snaps — see Joint::breakForce.
    // The joint is already removed when this fires.
    tc::Event<JointBreakEventArgs> jointBroke;

    // --- characters ------------------------------------------------------------
    // A walking-character controller (capsule centered on `position`): climbs
    // slopes up to maxSlopeAngle (rad), steps up stairs, slides along walls,
    // rides moving platforms and pushes dynamic bodies. It also carries an inner
    // kinematic body, so raycasts hit it and sensors see it. Steer it through
    // the returned PhysicsCharacter handle; the world advances all characters
    // inside update() — nothing extra to call.
    PhysicsCharacter addCharacter(const tc::Vec3& position, float radius,
                                  float cylinderHeight,
                                  float maxSlopeAngle = 0.8727f /* 50 deg */);
    void removeCharacter(const PhysicsCharacter& character);

    // --- queried / driven by PhysicsCharacter (you rarely call these directly) -
    bool hasCharacter(uint64_t id) const;
    void setCharacterMoveInput(uint64_t id, const tc::Vec3& velocity);
    void characterJump(uint64_t id, float speed);
    bool isCharacterGrounded(uint64_t id) const;
    bool isCharacterOnSteepSlope(uint64_t id) const;
    tc::Vec3 getCharacterGroundNormal(uint64_t id) const;
    tc::Vec3 getCharacterPosition(uint64_t id) const;
    void setCharacterPosition(uint64_t id, const tc::Vec3& p);
    tc::Vec3 getCharacterLinearVelocity(uint64_t id) const;

    // --- queried / driven by PhysicsJoint (you rarely call these directly) ---
    bool hasJoint(uint64_t id) const;
    JointType getJointType(uint64_t id) const;
    uint32_t getJointBodyA(uint64_t id) const;   // kInvalidId = jointed to the world
    uint32_t getJointBodyB(uint64_t id) const;
    tc::Vec3 getJointAnchorA(uint64_t id) const; // current world-space attachment
    tc::Vec3 getJointAnchorB(uint64_t id) const;
    tc::Vec3 getJointAxis(uint64_t id) const;
    void removeJointById(uint64_t id);
    // Motors (hinge: rad/s + N·m torque cap; slider: m/s + N force cap; -1 = unlimited).
    void setJointMotorVelocity(uint64_t id, float velocity, float maxForce);
    void setJointMotorTarget(uint64_t id, float target, float maxForce);
    void setJointMotorOff(uint64_t id);

    // --- queries -------------------------------------------------------------
    // Cast a ray from `origin` along `direction` (need not be normalized) up to
    // maxDistance, returning the CLOSEST body hit. Use for mouse picking (build
    // the ray from the camera), line-of-sight, ground probes, shooting, etc.
    // Static, kinematic and dynamic bodies are all hit; sensors are ignored.
    RaycastHit raycast(const tc::Vec3& origin, const tc::Vec3& direction,
                       float maxDistance = 1.0e6f) const;
    // Convenience overload: cast a tc::Ray (e.g. from a camera's
    // CameraContext::screenPointToRay for mouse picking).
    RaycastHit raycast(const tc::Ray& ray, float maxDistance = 1.0e6f) const {
        return raycast(ray.origin, ray.direction, maxDistance);
    }

    void removeBody(const PhysicsBody& body);
    // Remove every dynamic body but keep static scenery (floor/walls).
    void clearDynamicBodies();
    int getNumBodies() const;

    // --- queried / driven by PhysicsBody (you rarely call these directly) ----
    // PhysicsBody forwards to these; all are serialized against the step so they
    // stay safe while async stepping runs.
    tc::Vec3 getBodyPosition(uint32_t id) const;
    tc::Quaternion getBodyRotation(uint32_t id) const;
    tc::Vec3 getBodySize(uint32_t id) const;

    void applyForceToBody(uint32_t id, const tc::Vec3& force);
    void applyForceToBody(uint32_t id, const tc::Vec3& force, const tc::Vec3& worldPoint);
    void applyTorqueToBody(uint32_t id, const tc::Vec3& torque);
    void applyImpulseToBody(uint32_t id, const tc::Vec3& impulse);
    void applyImpulseToBody(uint32_t id, const tc::Vec3& impulse, const tc::Vec3& worldPoint);
    void applyAngularImpulseToBody(uint32_t id, const tc::Vec3& angularImpulse);
    void addVelocityToBody(uint32_t id, const tc::Vec3& dv);

    float getBodyMass(uint32_t id) const;

    void setBodyLinearVelocity(uint32_t id, const tc::Vec3& v);
    tc::Vec3 getBodyLinearVelocity(uint32_t id) const;
    void setBodyAngularVelocity(uint32_t id, const tc::Vec3& v);
    tc::Vec3 getBodyAngularVelocity(uint32_t id) const;

    void setBodyPosition(uint32_t id, const tc::Vec3& p);
    void setBodyRotation(uint32_t id, const tc::Quaternion& q);

    // Switch a body between static / kinematic / dynamic after creation.
    void setBodyMotionType(uint32_t id, MotionType type);
    // Drive a KINEMATIC body toward (pos, rot) over dt: Jolt derives the velocity
    // so the body smoothly pushes the dynamic bodies it meets (unlike setPosition,
    // which teleports and imparts no momentum). Call every frame with the frame dt.
    void moveBodyKinematic(uint32_t id, const tc::Vec3& pos, const tc::Quaternion& rot, float dt);

    // A sensor (trigger volume) reports overlaps via the normal contact events but
    // produces NO collision response — bodies pass straight through it.
    void setBodyIsSensor(uint32_t id, bool sensor);
    bool isBodySensor(uint32_t id) const;

    // Free 64-bit tag carried by the body (an id, an index, a packed pointer...).
    void setBodyUserData(uint32_t id, uint64_t data);
    uint64_t getBodyUserData(uint32_t id) const;

    // Degrees of freedom (Unity's "constraints"): which WORLD axes the body may
    // move along / rotate around. Bit set = allowed:
    //   1,2,4 = move X,Y,Z   8,16,32 = rotate X,Y,Z   0x3f = all (default)
    // Runtime-changeable; see PhysicsBody::lockRotation/lockTranslation/lock2D.
    void setBodyAllowedDofs(uint32_t id, uint32_t allowedBits);
    uint32_t getBodyAllowedDofs(uint32_t id) const;

    // Collision filtering: each body belongs to ONE layer (0..7) and carries a
    // MASK of layers it collides with (bit n = layer n). Two bodies collide iff
    // each one's mask contains the other's layer. Defaults: layer 0, mask 0xff
    // (everything collides). Sensors report only the overlaps their mask allows.
    void setBodyCollisionLayer(uint32_t id, int layer);
    int getBodyCollisionLayer(uint32_t id) const;
    void setBodyCollisionMask(uint32_t id, uint32_t mask);
    uint32_t getBodyCollisionMask(uint32_t id) const;

    void setBodyFriction(uint32_t id, float friction);
    float getBodyFriction(uint32_t id) const;
    void setBodyRestitution(uint32_t id, float restitution);
    float getBodyRestitution(uint32_t id) const;

    void activateBody(uint32_t id);
    bool isBodyActive(uint32_t id) const;

    // --- escape hatch (advanced) ---------------------------------------------
    // Raw pointers to the underlying Jolt objects, as void* so this header stays
    // Jolt-free. Don't cast these by hand — include <tcxPhysicsJolt.h> for typed
    // accessors (joltSystem(world) / joltBodyInterface(world)). Both are null
    // before setup(). Direct Jolt calls BYPASS the step lock: in async mode,
    // updateAsyncStop() first (or do your own locking).
    void* nativeSystem() const;          // -> JPH::PhysicsSystem*
    void* nativeBodyInterface() const;   // -> JPH::BodyInterface*

    // Lifetime token: holders (e.g. RigidBody nodes) keep a weak_ptr and check it
    // before calling removeBody() in their destructor. At shutdown the world may be
    // destroyed before its bodies' owners; when the token has expired, the world
    // (and all its bodies) is already gone, so skip the removeBody.
    std::weak_ptr<int> aliveToken() const { return alive_; }

private:
    std::shared_ptr<int> alive_ = std::make_shared<int>(0);
    // Shared by both addJoint overloads (idB == kInvalidId -> jointed to the world).
    PhysicsJoint addJointInternal(uint32_t idA, uint32_t idB, const Joint& def);
    // Shared by the setJointMotor* methods (mode: 0 velocity, 1 target, 2 off).
    void jointMotorCommand(uint64_t id, int mode, float value, float maxForce);
    // Snap breakable joints whose transmitted force/torque exceeded their
    // threshold during the last step of duration dt (fires jointBroke).
    void checkJointBreaks(float dt);
    // Advance every character controller by dt (steering + ExtendedUpdate).
    void updateCharacters(float dt);
    // Drain worker-collected contacts and fire contactBegan/Ended (main thread).
    void dispatchContacts();
#ifdef __EMSCRIPTEN__
    void webAsyncTick();   // fixed-timestep step driven by the frame loop
#else
    void asyncLoop();      // background-thread fixed-timestep loop
#endif

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace tcx
