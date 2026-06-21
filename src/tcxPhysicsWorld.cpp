#include "tcxPhysicsWorld.h"

// Jolt headers are confined to this translation unit (and tcxPhysicsBody.cpp).
// <Jolt/Jolt.h> must come first.
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterMask.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceMask.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterMask.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/SpringSettings.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>

// Compile the opt-in escape-hatch header here too, so it can never silently rot
// (it's inline-only — no ODR/runtime cost). Jolt is already on the path in this TU.
#include "tcxPhysicsJolt.h"

#include <thread>
#include <atomic>
#include <chrono>
#include <optional>
#include <functional>
#include <algorithm>
#include <vector>
#include <cstdarg>
#include <cstdio>
#include <cmath>

JPH_SUPPRESS_WARNINGS

using namespace std;
using namespace tc;

namespace tcx {

// ---------------------------------------------------------------------------
// Jolt boilerplate (object/broadphase layers, global init) — all internal.
// ---------------------------------------------------------------------------
namespace {

// Collision filtering uses Jolt's group/mask scheme: each body's ObjectLayer
// packs an 8-bit GROUP (which layers it belongs to) and an 8-bit MASK (which
// groups it collides with). Two bodies collide iff
// (groupA & maskB) && (groupB & maskA). The wrapper exposes this as
// setCollisionLayer(0..7) (single membership bit) + setCollisionMask(bits).
// One broadphase layer holds everything — fine at creative-coding body counts.
inline JPH::ObjectLayer defaultObjectLayer() {
    return JPH::ObjectLayerPairFilterMask::sGetObjectLayer(0x01, 0xff);
}

// Route Jolt's trace output to the TrussC log.
void traceImpl(const char* fmt, ...) {
    char buf[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    logNotice() << "[Jolt] " << buf;
}

#ifdef JPH_ENABLE_ASSERTS
bool assertFailedImpl(const char* expr, const char* msg, const char* file, JPH::uint line) {
    logError() << "[Jolt] assert failed: " << file << ":" << line << " (" << expr << ") "
               << (msg ? msg : "");
    return true; // trigger a breakpoint in a debugger
}
#endif

// Jolt has process-global state (Factory + registered types). Reference-count it
// so multiple PhysicsWorlds (or repeated setup/teardown) stay correct.
int g_refCount = 0;

void globalInit() {
    if (g_refCount++ == 0) {
        JPH::RegisterDefaultAllocator();
        JPH::Trace = traceImpl;
        JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = assertFailedImpl;)
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }
}

void globalShutdown() {
    if (g_refCount > 0 && --g_refCount == 0) {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
}

inline JPH::RVec3 toJolt(const Vec3& v) { return JPH::RVec3(v.x, v.y, v.z); }
inline Vec3       toTc(const JPH::Vec3& v)  { return Vec3(v.GetX(), v.GetY(), v.GetZ()); }
#ifdef JPH_DOUBLE_PRECISION
// In double precision RVec3 is a distinct type; single precision aliases Vec3.
inline Vec3       toTc(const JPH::RVec3& v) { return Vec3((float)v.GetX(), (float)v.GetY(), (float)v.GetZ()); }
#endif
// Jolt Quat is (x, y, z, w); tc::Quaternion is (w, x, y, z).
inline Quaternion toTc(const JPH::Quat& q) { return Quaternion(q.GetW(), q.GetX(), q.GetY(), q.GetZ()); }
inline JPH::Quat  toJolt(const Quaternion& q) { return JPH::Quat(q.x, q.y, q.z, q.w); }

constexpr JPH::uint cMaxPhysicsJobs     = 2048;
constexpr JPH::uint cMaxPhysicsBarriers = 8;

// ---------------------------------------------------------------------------
// ContactListener — runs on Jolt worker threads. It must do almost nothing:
// it just pushes a minimal record through `sink` (a thread-safe enqueue set up
// by the world). The records are replayed as events on the main thread later,
// so user handlers never run on a physics worker thread.
// ---------------------------------------------------------------------------
// Contact phase passed through the sink: 0 = began, 1 = persisted (still
// touching), 2 = ended. Matches PhysicsWorld's three contact events.
class ContactListenerImpl final : public JPH::ContactListener {
public:
    // (bodyA, bodyB, worldPoint, worldNormal, approachSpeed, phase)
    std::function<void(uint32_t, uint32_t, const Vec3&, const Vec3&, float, int)> sink;

    void OnContactAdded(const JPH::Body& b1, const JPH::Body& b2,
                        const JPH::ContactManifold& manifold,
                        JPH::ContactSettings&) override {
        fromManifold(b1, b2, manifold, 0);
    }

    // Fires every step while a pair keeps touching — can be a lot of events.
    void OnContactPersisted(const JPH::Body& b1, const JPH::Body& b2,
                            const JPH::ContactManifold& manifold,
                            JPH::ContactSettings&) override {
        fromManifold(b1, b2, manifold, 1);
    }

    void OnContactRemoved(const JPH::SubShapeIDPair& pair) override {
        if (!sink) return;
        // No manifold on removal — point/normal/speed are unknown.
        sink(pair.GetBody1ID().GetIndexAndSequenceNumber(),
             pair.GetBody2ID().GetIndexAndSequenceNumber(),
             Vec3(), Vec3(), 0.0f, 2);
    }

private:
    void fromManifold(const JPH::Body& b1, const JPH::Body& b2,
                      const JPH::ContactManifold& manifold, int phase) {
        if (!sink) return;
        Vec3 normal = toTc(manifold.mWorldSpaceNormal);
        Vec3 point  = toTc(manifold.GetWorldSpaceContactPointOn1(0));
        // Approach speed along the contact normal — a cheap, useful proxy for
        // "how hard" the contact is.
        float speed = std::abs((b1.GetLinearVelocity() - b2.GetLinearVelocity())
                                   .Dot(manifold.mWorldSpaceNormal));
        sink(b1.GetID().GetIndexAndSequenceNumber(),
             b2.GetID().GetIndexAndSequenceNumber(), point, normal, speed, phase);
    }
};

} // anonymous namespace

// ---------------------------------------------------------------------------
// PhysicsWorld::Impl
// ---------------------------------------------------------------------------
struct PhysicsWorld::Impl {
    bool initialized = false;

    unique_ptr<JPH::TempAllocatorImpl> tempAllocator;
    unique_ptr<JPH::JobSystem>         jobSystem;

    // BroadPhaseLayerInterfaceMask allocates in its constructor, which runs
    // when Impl is built (PhysicsWorld's constructor — BEFORE setup()/globalInit
    // had a chance to register Jolt's allocator). Register it first via this
    // leading member; the call is idempotent.
    struct JoltAllocatorInit {
        JoltAllocatorInit() { JPH::RegisterDefaultAllocator(); }
    } allocatorInit;

    // Filters must outlive physicsSystem (it keeps references to them), so they
    // are declared first. Group/mask scheme; a single broadphase layer.
    JPH::BroadPhaseLayerInterfaceMask      bpLayer{1};
    JPH::ObjectVsBroadPhaseLayerFilterMask objVsBpFilter{bpLayer};
    JPH::ObjectLayerPairFilterMask         objPairFilter;

    JPH::PhysicsSystem physicsSystem;
    ContactListenerImpl contactListener;

    vector<JPH::BodyID> dynamicBodies; // tracked so clearDynamicBodies() works

    // --- joint registry ------------------------------------------------------
    // The world owns every joint: the Jolt constraint (ref-counted) plus what a
    // PhysicsJoint handle can ask for. Anchors / axis are stored LOCAL to their
    // body at creation, so the world-space accessors track the moving bodies.
    // bodyB == kInvalidId means "jointed to the world" (JPH::Body::sFixedToWorld).
    struct JointRec {
        uint64_t id = 0;
        JPH::Ref<JPH::Constraint> constraint;
        uint32_t bodyA = 0xffffffffu;
        uint32_t bodyB = 0xffffffffu;
        JointType type = JointType::Point;
        JPH::Vec3 localAnchorA = JPH::Vec3::sZero();  // local to bodyA
        JPH::Vec3 localAnchorB = JPH::Vec3::sZero();  // local to bodyB (world if none)
        JPH::Vec3 localAxisA = JPH::Vec3::sAxisY();   // local to bodyA
        float breakForce = -1.0f;                     // N   (-1 = unbreakable)
        float breakTorque = -1.0f;                    // N·m (-1 = unbreakable)
    };
    vector<JointRec> joints;
    uint64_t nextJointId = 1;

    // Remove every joint touching `bodyId` (called BEFORE that body is
    // destroyed — a Jolt constraint must never outlive its bodies).
    void removeJointsTouching(uint32_t bodyId) {
        for (auto it = joints.begin(); it != joints.end();) {
            if (it->bodyA == bodyId || it->bodyB == bodyId) {
                physicsSystem.RemoveConstraint(it->constraint.GetPtr());
                it = joints.erase(it);
            } else {
                ++it;
            }
        }
    }

    JointRec* find(uint64_t id) {
        for (auto& r : joints) if (r.id == id) return &r;
        return nullptr;   // linear scan — joint counts are small
    }

    // --- character registry --------------------------------------------------
    // CharacterVirtual is not tracked by the PhysicsSystem; the world owns and
    // advances them (updateCharacters). moveInput/jump are buffered steering.
    struct CharRec {
        uint64_t id = 0;
        JPH::Ref<JPH::CharacterVirtual> ch;
        JPH::Vec3 moveInput = JPH::Vec3::sZero();   // desired horizontal velocity
        float jumpSpeed = 0.0f;                     // pending jump (0 = none)
    };
    vector<CharRec> characters;
    uint64_t nextCharId = 1;

    CharRec* findChar(uint64_t id) {
        for (auto& r : characters) if (r.id == id) return &r;
        return nullptr;
    }

    // Local anchor -> current world position (world-fixed side stores world).
    JPH::Vec3 anchorWorld(uint32_t bodyId, const JPH::Vec3& local) {
        if (bodyId == 0xffffffffu) return local;
        JPH::BodyID bid(bodyId);
        return JPH::Vec3(bodies().GetPosition(bid)) + bodies().GetRotation(bid) * local;
    }

    // A joint's current world-space axis (stored local to its base body A).
    JPH::Vec3 axisWorld(const JointRec& r) {
        if (r.bodyA == 0xffffffffu) return r.localAxisA;
        return bodies().GetRotation(JPH::BodyID(r.bodyA)) * r.localAxisA;
    }

    JPH::BodyInterface& bodies() { return physicsSystem.GetBodyInterface(); }
    const JPH::BodyInterface& bodies() const { return physicsSystem.GetBodyInterface(); }

    // --- threading / contact buffering --------------------------------------
    // simMutex serializes physicsSystem.Update() against body reads/mutations so
    // those stay safe while the async thread steps. On single-threaded web it is
    // a no-op (TC_MUTEX = NullMutex). contactsMutex guards `pending`.
    TC_MUTEX simMutex;
    TC_MUTEX contactsMutex;

    struct PendingContact {
        uint32_t a = 0xffffffffu;
        uint32_t b = 0xffffffffu;
        Vec3  point;
        Vec3  normal;
        float speed = 0.0f;
        int   phase = 0;   // 0 = began, 1 = persisted, 2 = ended
    };
    vector<PendingContact> pending;

    // Async stepping state.
    bool async = false;
    bool asyncWarnedWeb = false;
    float asyncHz = 120.0f;
    EventListener frameListener;  // events().update hook (dispatch on web/native)
#ifndef __EMSCRIPTEN__
    std::atomic<bool> asyncRunning{false};
    std::thread asyncThread;
#endif
    // Web fixed-timestep accumulator (frame-loop driven).
    float  webPrevTime = 0.0f;
    double webAccum = 0.0;

    // Sim time multiplier for the accumulator-based modes (fixed / async). 1 =
    // real time, 2 = double speed, 0 = paused. Plain float: a benign cross-thread
    // read in async (set rarely, torn reads harmless for a debug/slow-mo scale).
    float  timeScale = 1.0f;

    // Synchronous fixed-timestep accumulator (updateFixed / updateFixedStart).
    double fixedAccum = 0.0;
    bool   fixedStepping = false;   // updateFixedStart() is driving the frame loop
    float  fixedHz = 60.0f;
    int    fixedMaxSteps = 8;
    EventListener fixedFrameListener;  // events().update hook for auto fixed stepping
};

// ---------------------------------------------------------------------------
// PhysicsWorld
// ---------------------------------------------------------------------------
PhysicsWorld::PhysicsWorld() : impl_(make_unique<Impl>()) {}

PhysicsWorld::~PhysicsWorld() {
    if (impl_ && impl_->initialized) {
        if (impl_->async) updateAsyncStop();
        globalShutdown();
    }
}

void PhysicsWorld::setup(int maxBodies, int maxBodyPairs, int maxContactConstraints) {
    if (impl_->initialized) {
        logWarning() << "tcxPhysics: PhysicsWorld::setup() called twice — ignoring.";
        return;
    }
    globalInit();

#ifdef __EMSCRIPTEN__
    // Web build: assume a single thread (TrussC's web backend is single-threaded).
    impl_->jobSystem = make_unique<JPH::JobSystemSingleThreaded>(cMaxPhysicsJobs);
#else
    int numThreads = std::max(1u, std::thread::hardware_concurrency() - 1);
    impl_->jobSystem = make_unique<JPH::JobSystemThreadPool>(
        cMaxPhysicsJobs, cMaxPhysicsBarriers, numThreads);
#endif

    const JPH::uint maxB = (JPH::uint)std::max(1, maxBodies);
    // Broadphase pair / contact-constraint buffers. These must be generous: when
    // many bodies pile up, the number of simultaneous contacts can far exceed the
    // body count, and if the buffer fills, Jolt silently drops contacts and the
    // bodies tunnel through each other (and the floor). Scale well above maxB.
    const JPH::uint maxPairs = maxBodyPairs > 0
        ? (JPH::uint)maxBodyPairs : std::max<JPH::uint>(65536, maxB * 8);
    const JPH::uint maxCC = maxContactConstraints > 0
        ? (JPH::uint)maxContactConstraints : std::max<JPH::uint>(20480, maxB * 4);

    // The per-step temp allocator must scale with the buffers above — a dense
    // pile needs scratch memory proportional to the number of contacts/pairs it
    // resolves. Too small and Jolt logs "TempAllocator: Out of memory" and the
    // step degrades. Derive it from the buffer sizes with a 16MB floor.
    size_t tempSize = std::max<size_t>(16u * 1024 * 1024,
                                       ((size_t)maxPairs + (size_t)maxCC) * 512);
    impl_->tempAllocator = make_unique<JPH::TempAllocatorImpl>((JPH::uint)tempSize);

    // Single broadphase layer holding every group (group/mask filtering happens
    // at the object-layer level).
    impl_->bpLayer.ConfigureLayer(JPH::BroadPhaseLayer(0), 0xff, 0);

    impl_->physicsSystem.Init(
        maxB, /*numBodyMutexes*/ 0, maxPairs, maxCC,
        impl_->bpLayer, impl_->objVsBpFilter, impl_->objPairFilter);

    impl_->physicsSystem.SetGravity(JPH::Vec3(0.0f, -9.81f, 0.0f));

    // Route Jolt's (worker-thread) contact callbacks into a thread-safe buffer;
    // they are replayed as events on the main thread in dispatchContacts().
    impl_->contactListener.sink =
        [imp = impl_.get()](uint32_t a, uint32_t b, const Vec3& pt,
                            const Vec3& n, float s, int phase) {
            TC_LOCK_GUARD(imp->contactsMutex);
            imp->pending.push_back({a, b, pt, n, s, phase});
        };
    impl_->physicsSystem.SetContactListener(&impl_->contactListener);

    impl_->initialized = true;
}

PhysicsWorld& PhysicsWorld::setGravity(const Vec3& g) {
    if (impl_->initialized) {
        TC_LOCK_GUARD(impl_->simMutex);
        impl_->physicsSystem.SetGravity(JPH::Vec3(g.x, g.y, g.z));
    }
    return *this;
}

Vec3 PhysicsWorld::getGravity() const {
    if (!impl_->initialized) return Vec3(0, -9.81f, 0);
    return toTc(impl_->physicsSystem.GetGravity());
}

PhysicsWorld& PhysicsWorld::setTimeScale(float s) {
    impl_->timeScale = std::max(0.0f, s);
    return *this;
}

float PhysicsWorld::getTimeScale() const { return impl_->timeScale; }

void PhysicsWorld::update(float dt, int collisionSteps) {
    if (!impl_->initialized) return;
    if (impl_->async) {
        logWarning() << "tcxPhysics: update() ignored while async stepping runs "
                        "(call updateAsyncStop() first).";
        return;
    }
    if (impl_->fixedStepping) {
        logWarning() << "tcxPhysics: update() ignored while updateFixedStart() runs "
                        "(call updateFixedStop() first).";
        return;
    }
    // Pre-step inputs (kinematic movers, forces) run in lock-step with the sim —
    // here that's one step per frame, so physicsUpdate fires once with dt.
    physicsUpdate.notify(dt);
    // Characters steer and collide against the current world state first, then
    // the system step resolves the dynamics (incl. their inner bodies).
    updateCharacters(dt);
    {
        TC_LOCK_GUARD(impl_->simMutex);
        impl_->physicsSystem.Update(dt, std::max(1, collisionSteps),
                                    impl_->tempAllocator.get(), impl_->jobSystem.get());
    }
    checkJointBreaks(dt / std::max(1, collisionSteps));
    dispatchContacts();
}

// One fixed step: fire physicsUpdate (so listeners set kinematic targets / forces
// for THIS step), then advance characters + dynamics by exactly `step`.
void PhysicsWorld::fixedStepOnce(float step) {
    physicsUpdate.notify(step);   // pre-step inputs, in lock-step with the sim
    updateCharacters(step);
    {
        TC_LOCK_GUARD(impl_->simMutex);
        impl_->physicsSystem.Update(step, 1, impl_->tempAllocator.get(),
                                    impl_->jobSystem.get());
    }
}

// Shared accumulator loop for both the manual updateFixed() and the auto
// updateFixedStart() listener.
float PhysicsWorld::fixedTick(float realDt, float hz, int maxSteps) {
    const float step = 1.0f / std::max(1.0f, hz);
    const int cap = std::max(1, maxSteps);
    double scaled = (double)realDt * std::max(0.0f, impl_->timeScale);
    if (scaled > 0.0) impl_->fixedAccum += scaled;
    // Cap the catch-up budget at maxSteps worth of time (spiral-of-death guard):
    // maxSteps is the per-frame step budget, so a big timeScale fast-forward just
    // needs a big maxSteps — no separate fixed-seconds ceiling fighting it.
    double maxAcc = (double)cap * step;
    if (impl_->fixedAccum > maxAcc) impl_->fixedAccum = maxAcc;

    int steps = 0;
    while (impl_->fixedAccum >= step && steps < cap) {
        fixedStepOnce(step);
        impl_->fixedAccum -= step;
        ++steps;
    }
    // Drop any leftover we couldn't consume within the cap so the next frame
    // starts fresh instead of trying to catch up forever.
    if (impl_->fixedAccum >= step) impl_->fixedAccum = 0.0;

    if (steps > 0) {
        checkJointBreaks(step);
        dispatchContacts();
    }
    // Interpolation factor in [0,1): how far into the next step we already are.
    return (float)(impl_->fixedAccum / step);
}

float PhysicsWorld::updateFixed(float realDt, float hz, int maxSteps) {
    if (!impl_->initialized) return 0.0f;
    if (impl_->async) {
        logWarning() << "tcxPhysics: updateFixed() ignored while async stepping runs "
                        "(call updateAsyncStop() first).";
        return 0.0f;
    }
    if (impl_->fixedStepping) {
        logWarning() << "tcxPhysics: updateFixed() ignored while updateFixedStart() drives "
                        "the loop (don't call it manually too).";
        return 0.0f;
    }
    return fixedTick(realDt, hz, maxSteps);
}

void PhysicsWorld::updateFixedStart(float hz, int maxSteps) {
    if (!impl_->initialized) {
        logWarning() << "tcxPhysics: updateFixedStart() before setup() — ignored.";
        return;
    }
    if (impl_->async) {
        logWarning() << "tcxPhysics: updateFixedStart() ignored while async stepping runs "
                        "(call updateAsyncStop() first).";
        return;
    }
    if (impl_->fixedStepping) return;
    impl_->fixedStepping  = true;
    impl_->fixedHz        = std::max(1.0f, hz);
    impl_->fixedMaxSteps  = std::max(1, maxSteps);
    impl_->fixedAccum     = 0.0;
    // Drive the accumulator from the frame loop — synchronous, main-thread, so
    // physicsUpdate listeners and body writes are always ordered against the sim.
    impl_->fixedFrameListener = events().update.listen([this] {
        fixedTick((float)getDeltaTime(), impl_->fixedHz, impl_->fixedMaxSteps);
    });
}

void PhysicsWorld::updateFixedStop() {
    if (!impl_->fixedStepping) return;
    impl_->fixedStepping = false;
    impl_->fixedFrameListener = EventListener();  // disconnect from events().update
    dispatchContacts();                            // flush anything the last step queued
}

bool PhysicsWorld::isFixedStepping() const { return impl_->fixedStepping; }

// ---------------------------------------------------------------------------
// Async stepping
// ---------------------------------------------------------------------------
void PhysicsWorld::updateAsyncStart(float hz) {
    if (!impl_->initialized) {
        logWarning() << "tcxPhysics: updateAsyncStart() before setup() — ignored.";
        return;
    }
    if (impl_->fixedStepping) {
        logWarning() << "tcxPhysics: updateAsyncStart() ignored while updateFixedStart() runs "
                        "(call updateFixedStop() first).";
        return;
    }
    if (impl_->async) return;
    impl_->asyncHz = std::max(1.0f, hz);
    impl_->async = true;

#ifdef __EMSCRIPTEN__
    // No background threads on web — step from the frame loop instead.
    if (!impl_->asyncWarnedWeb) {
        impl_->asyncWarnedWeb = true;
        logWarning() << "tcxPhysics: web has no background threads — async stepping "
                        "falls back to fixed-timestep stepping driven by the frame loop.";
    }
    impl_->webPrevTime = getElapsedTimef();
    impl_->webAccum = 0.0;
    impl_->frameListener = events().update.listen([this] { webAsyncTick(); });
#else
    impl_->asyncRunning = true;
    impl_->asyncThread = std::thread([this] { asyncLoop(); });
    // The worker only steps; events are still delivered on the main thread.
    impl_->frameListener = events().update.listen([this] {
        // Steps run off-thread, so the best a main-thread mover can do is set
        // kinematic targets / forces once per frame with the frame delta.
        float fdt = (float)getDeltaTime();
        physicsUpdate.notify(fdt);
        // Characters advance on the frame loop (per-frame steering reads).
        updateCharacters(fdt);
        checkJointBreaks(1.0f / impl_->asyncHz);
        dispatchContacts();
    });
#endif
}

void PhysicsWorld::updateAsyncStop() {
    if (!impl_->async) return;
#ifndef __EMSCRIPTEN__
    impl_->asyncRunning = false;
    if (impl_->asyncThread.joinable()) impl_->asyncThread.join();
#endif
    impl_->frameListener = EventListener();  // disconnect from events().update
    impl_->async = false;
    dispatchContacts();  // flush anything the final step queued
}

bool PhysicsWorld::isAsync() const { return impl_->async; }

#ifndef __EMSCRIPTEN__
void PhysicsWorld::asyncLoop() {
    using clock = std::chrono::steady_clock;
    const float step = 1.0f / impl_->asyncHz;
    auto prev = clock::now();
    double accum = 0.0;
    while (impl_->asyncRunning) {
        auto now = clock::now();
        accum += std::chrono::duration<double>(now - prev).count() * std::max(0.0f, impl_->timeScale);
        prev = now;
        if (accum > 0.25) accum = 0.25;  // clamp catch-up (avoid spiral of death)
        int steps = 0;
        while (accum >= step && steps < 8) {
            {
                TC_LOCK_GUARD(impl_->simMutex);
                impl_->physicsSystem.Update(step, 1, impl_->tempAllocator.get(),
                                            impl_->jobSystem.get());
            }
            accum -= step;
            ++steps;
        }
        double sleep = step - accum;
        if (sleep < 0.0005) sleep = 0.0005;
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep));
    }
}
#else
void PhysicsWorld::webAsyncTick() {
    const float step = 1.0f / impl_->asyncHz;
    float t = getElapsedTimef();
    double dt = (double)t - impl_->webPrevTime;
    impl_->webPrevTime = t;
    if (dt < 0.0) dt = 0.0;
    impl_->webAccum += dt * std::max(0.0f, impl_->timeScale);
    if (impl_->webAccum > 0.25) impl_->webAccum = 0.25;
    int steps = 0;
    float h = step;   // mutable lvalue for notify(float&)
    while (impl_->webAccum >= step && steps < 8) {
        physicsUpdate.notify(h);   // pre-step inputs, per fixed step (main thread)
        updateCharacters(step);
        impl_->physicsSystem.Update(step, 1, impl_->tempAllocator.get(),
                                    impl_->jobSystem.get());
        impl_->webAccum -= step;
        ++steps;
    }
    if (steps > 0) checkJointBreaks(step);
    dispatchContacts();
}
#endif

// ---------------------------------------------------------------------------
// Contact event dispatch (main thread)
// ---------------------------------------------------------------------------
void PhysicsWorld::dispatchContacts() {
    vector<Impl::PendingContact> local;
    {
        TC_LOCK_GUARD(impl_->contactsMutex);
        if (impl_->pending.empty()) return;
        local.swap(impl_->pending);
    }
    for (const auto& pc : local) {
        ContactEventArgs args;
        args.a      = PhysicsBody(this, pc.a);
        args.b      = PhysicsBody(this, pc.b);
        args.point  = pc.point;
        args.normal = pc.normal;
        args.speed  = pc.speed;
        switch (pc.phase) {
            case 0:  contactBegan.notify(args);     break;
            case 1:  contactPersisted.notify(args); break;
            default: contactEnded.notify(args);     break;
        }
    }
}

PhysicsBody PhysicsWorld::addBox(const Vec3& position, const Vec3& size, bool dynamic, float density) {
    if (!impl_->initialized) return PhysicsBody();

    JPH::Vec3 half(std::max(0.001f, size.x * 0.5f),
                   std::max(0.001f, size.y * 0.5f),
                   std::max(0.001f, size.z * 0.5f));
    // Convex radius must be <= the smallest half extent.
    float minHalf = std::min({half.GetX(), half.GetY(), half.GetZ()});
    float convexRadius = std::min(JPH::cDefaultConvexRadius, minHalf * 0.5f);

    JPH::BoxShapeSettings shapeSettings(half, convexRadius);
    shapeSettings.SetDensity(std::max(0.0001f, density));
    JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) {
        logError() << "tcxPhysics: box shape error: " << result.GetError().c_str();
        return PhysicsBody();
    }

    JPH::BodyCreationSettings bcs(
        result.Get(), toJolt(position), JPH::Quat::sIdentity(),
        dynamic ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static,
        defaultObjectLayer());
    bcs.mAllowDynamicOrKinematic = true;   // allow setMotionType later

    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID id = impl_->bodies().CreateAndAddBody(
        bcs, dynamic ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
    if (id.IsInvalid()) return PhysicsBody();
    if (dynamic) impl_->dynamicBodies.push_back(id);
    return PhysicsBody(this, id.GetIndexAndSequenceNumber());
}

PhysicsBody PhysicsWorld::addSphere(const Vec3& position, float radius, bool dynamic, float density) {
    if (!impl_->initialized) return PhysicsBody();

    JPH::SphereShapeSettings shapeSettings(std::max(0.001f, radius));
    shapeSettings.SetDensity(std::max(0.0001f, density));
    JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) {
        logError() << "tcxPhysics: sphere shape error: " << result.GetError().c_str();
        return PhysicsBody();
    }

    JPH::BodyCreationSettings bcs(
        result.Get(), toJolt(position), JPH::Quat::sIdentity(),
        dynamic ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static,
        defaultObjectLayer());
    bcs.mAllowDynamicOrKinematic = true;   // allow setMotionType later

    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID id = impl_->bodies().CreateAndAddBody(
        bcs, dynamic ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
    if (id.IsInvalid()) return PhysicsBody();
    if (dynamic) impl_->dynamicBodies.push_back(id);
    return PhysicsBody(this, id.GetIndexAndSequenceNumber());
}

PhysicsBody PhysicsWorld::addCapsule(const Vec3& position, float radius, float cylinderHeight,
                                     bool dynamic, float density) {
    if (!impl_->initialized) return PhysicsBody();

    JPH::CapsuleShapeSettings shapeSettings(std::max(0.001f, cylinderHeight * 0.5f),
                                            std::max(0.001f, radius));
    shapeSettings.SetDensity(std::max(0.0001f, density));
    JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) {
        logError() << "tcxPhysics: capsule shape error: " << result.GetError().c_str();
        return PhysicsBody();
    }

    JPH::BodyCreationSettings bcs(
        result.Get(), toJolt(position), JPH::Quat::sIdentity(),
        dynamic ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static,
        defaultObjectLayer());
    bcs.mAllowDynamicOrKinematic = true;   // allow setMotionType later

    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID id = impl_->bodies().CreateAndAddBody(
        bcs, dynamic ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
    if (id.IsInvalid()) return PhysicsBody();
    if (dynamic) impl_->dynamicBodies.push_back(id);
    return PhysicsBody(this, id.GetIndexAndSequenceNumber());
}

PhysicsBody PhysicsWorld::addCylinder(const Vec3& position, float radius, float height,
                                      bool dynamic, float density) {
    if (!impl_->initialized) return PhysicsBody();

    float halfH = std::max(0.001f, height * 0.5f);
    float r = std::max(0.001f, radius);
    // Convex radius must be <= the smaller of half-height and radius.
    float convexRadius = std::min(JPH::cDefaultConvexRadius, std::min(halfH, r) * 0.5f);

    JPH::CylinderShapeSettings shapeSettings(halfH, r, convexRadius);
    shapeSettings.SetDensity(std::max(0.0001f, density));
    JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) {
        logError() << "tcxPhysics: cylinder shape error: " << result.GetError().c_str();
        return PhysicsBody();
    }

    JPH::BodyCreationSettings bcs(
        result.Get(), toJolt(position), JPH::Quat::sIdentity(),
        dynamic ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static,
        defaultObjectLayer());
    bcs.mAllowDynamicOrKinematic = true;   // allow setMotionType later

    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID id = impl_->bodies().CreateAndAddBody(
        bcs, dynamic ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
    if (id.IsInvalid()) return PhysicsBody();
    if (dynamic) impl_->dynamicBodies.push_back(id);
    return PhysicsBody(this, id.GetIndexAndSequenceNumber());
}

PhysicsBody PhysicsWorld::addConvexHull(const Vec3& position, const Mesh& mesh,
                                        bool dynamic, float density) {
    if (!impl_->initialized) return PhysicsBody();

    const std::vector<Vec3>& verts = mesh.getVertices();
    if (verts.size() < 4) {
        logError() << "tcxPhysics: addConvexHull needs at least 4 points.";
        return PhysicsBody();
    }
    JPH::Array<JPH::Vec3> points;
    points.reserve(verts.size());
    for (const Vec3& v : verts) points.push_back(JPH::Vec3(v.x, v.y, v.z));

    JPH::ConvexHullShapeSettings shapeSettings(points);
    shapeSettings.SetDensity(std::max(0.0001f, density));
    JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) {
        logError() << "tcxPhysics: convex hull error: " << result.GetError().c_str();
        return PhysicsBody();
    }

    JPH::BodyCreationSettings bcs(
        result.Get(), toJolt(position), JPH::Quat::sIdentity(),
        dynamic ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static,
        defaultObjectLayer());
    bcs.mAllowDynamicOrKinematic = true;   // allow setMotionType later

    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID id = impl_->bodies().CreateAndAddBody(
        bcs, dynamic ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
    if (id.IsInvalid()) return PhysicsBody();
    if (dynamic) impl_->dynamicBodies.push_back(id);
    return PhysicsBody(this, id.GetIndexAndSequenceNumber());
}

PhysicsBody PhysicsWorld::addMesh(const Vec3& position, const Mesh& mesh, bool dynamic) {
    if (!impl_->initialized) return PhysicsBody();
    if (dynamic) {
        logWarning() << "tcxPhysics: addMesh is static-only (Jolt mesh shapes carry no "
                        "mass) — creating it static.";
    }

    const std::vector<Vec3>& verts = mesh.getVertices();
    const std::vector<unsigned int>& inds = mesh.getIndices();

    JPH::VertexList vlist;
    vlist.reserve(verts.size());
    for (const Vec3& v : verts) vlist.push_back(JPH::Float3(v.x, v.y, v.z));

    JPH::IndexedTriangleList tris;
    if (!inds.empty()) {
        tris.reserve(inds.size() / 3);
        for (size_t i = 0; i + 2 < inds.size(); i += 3)
            tris.push_back(JPH::IndexedTriangle((JPH::uint32)inds[i], (JPH::uint32)inds[i + 1],
                                                (JPH::uint32)inds[i + 2], 0));
    } else {
        // Non-indexed mesh: every 3 vertices form a triangle.
        for (JPH::uint32 i = 0; i + 2 < (JPH::uint32)verts.size(); i += 3)
            tris.push_back(JPH::IndexedTriangle(i, i + 1, i + 2, 0));
    }
    if (vlist.empty() || tris.empty()) {
        logError() << "tcxPhysics: addMesh got an empty / triangle-less mesh.";
        return PhysicsBody();
    }

    JPH::MeshShapeSettings shapeSettings(vlist, tris);
    JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) {
        logError() << "tcxPhysics: mesh shape error: " << result.GetError().c_str();
        return PhysicsBody();
    }

    // Mesh shapes are always static.
    JPH::BodyCreationSettings bcs(
        result.Get(), toJolt(position), JPH::Quat::sIdentity(),
        JPH::EMotionType::Static, defaultObjectLayer());

    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID id = impl_->bodies().CreateAndAddBody(bcs, JPH::EActivation::DontActivate);
    if (id.IsInvalid()) return PhysicsBody();
    return PhysicsBody(this, id.GetIndexAndSequenceNumber());
}

PhysicsBody PhysicsWorld::addGroundPlane(float y, float size) {
    PhysicsBody ground = addBox(Vec3(0, y - 5.0f, 0), Vec3(size, 10.0f, size), false);
    if (impl_->initialized) {
        TC_LOCK_GUARD(impl_->simMutex);
        impl_->physicsSystem.OptimizeBroadPhase();
    }
    return ground;
}

RaycastHit PhysicsWorld::raycast(const Vec3& origin, const Vec3& direction, float maxDistance) const {
    RaycastHit out;
    if (!impl_->initialized) return out;

    float len = direction.length();
    // Reject degenerate / non-finite input (e.g. a ray built from an uninitialized
    // camera matrix one frame too early) — a NaN ray crashes Jolt's narrow phase.
    if (!std::isfinite(len) || len < 1e-8f || maxDistance <= 0.0f) return out;
    if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.z)) return out;
    Vec3 dir = direction * (1.0f / len);          // unit direction
    JPH::Vec3 disp(dir.x * maxDistance, dir.y * maxDistance, dir.z * maxDistance);
    JPH::RRayCast ray{ toJolt(origin), disp };

    JPH::RayCastResult result;
    TC_LOCK_GUARD(impl_->simMutex);
    // Closest hit against all bodies with default filters (sensors are skipped by
    // the narrow phase). mFraction is along the ray's displacement vector.
    if (!impl_->physicsSystem.GetNarrowPhaseQuery().CastRay(ray, result)) return out;

    out.hit      = true;
    out.distance = result.mFraction * maxDistance;
    out.point    = origin + dir * out.distance;
    out.body     = PhysicsBody(const_cast<PhysicsWorld*>(this),
                               result.mBodyID.GetIndexAndSequenceNumber());
    // Surface normal at the hit point needs the live body.
    JPH::BodyLockRead bodyLock(impl_->physicsSystem.GetBodyLockInterface(), result.mBodyID);
    if (bodyLock.Succeeded()) {
        out.normal = toTc(bodyLock.GetBody().GetWorldSpaceSurfaceNormal(
            result.mSubShapeID2, toJolt(out.point)));
    }
    return out;
}

void PhysicsWorld::removeBody(const PhysicsBody& body) {
    if (!impl_->initialized || !body.isValid()) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID id(body.getId());
    // Joints touching this body must go first (a constraint must never
    // outlive its bodies).
    impl_->removeJointsTouching(body.getId());
    impl_->bodies().RemoveBody(id);
    impl_->bodies().DestroyBody(id);
    auto& v = impl_->dynamicBodies;
    v.erase(std::remove(v.begin(), v.end(), id), v.end());
}

void PhysicsWorld::clearDynamicBodies() {
    if (!impl_->initialized) return;
    TC_LOCK_GUARD(impl_->simMutex);
    for (JPH::BodyID id : impl_->dynamicBodies) {
        impl_->removeJointsTouching(id.GetIndexAndSequenceNumber());
        impl_->bodies().RemoveBody(id);
        impl_->bodies().DestroyBody(id);
    }
    impl_->dynamicBodies.clear();
}

int PhysicsWorld::getNumBodies() const {
    if (!impl_->initialized) return 0;
    return (int)impl_->physicsSystem.GetNumBodies();
}

// ---------------------------------------------------------------------------
// Joints
// ---------------------------------------------------------------------------
namespace {

// Build the Jolt constraint described by `def` between two locked bodies.
// Everything is WorldSpace — matching the Joint factories' contract.
JPH::TwoBodyConstraint* createJoltConstraint(const Joint& def, JPH::Body& a, JPH::Body& b) {
    switch (def.type) {
        case JointType::Point: {
            JPH::PointConstraintSettings s;
            s.mSpace  = JPH::EConstraintSpace::WorldSpace;
            s.mPoint1 = s.mPoint2 = toJolt(def.anchorA);
            return s.Create(a, b);
        }
        case JointType::Hinge: {
            JPH::HingeConstraintSettings s;
            s.mSpace  = JPH::EConstraintSpace::WorldSpace;
            s.mPoint1 = s.mPoint2 = toJolt(def.anchorA);
            JPH::Vec3 axis = JPH::Vec3(def.axis.x, def.axis.y, def.axis.z).Normalized();
            s.mHingeAxis1  = s.mHingeAxis2  = axis;
            s.mNormalAxis1 = s.mNormalAxis2 = axis.GetNormalizedPerpendicular();
            if (def.hasLimits) {
                s.mLimitsMin = std::max(-JPH::JPH_PI, def.limitMin);
                s.mLimitsMax = std::min( JPH::JPH_PI, def.limitMax);
            }
            return s.Create(a, b);
        }
        case JointType::Slider: {
            JPH::SliderConstraintSettings s;
            s.mSpace = JPH::EConstraintSpace::WorldSpace;
            s.mAutoDetectPoint = true;   // attach at the bodies' current poses
            s.SetSliderAxis(JPH::Vec3(def.axis.x, def.axis.y, def.axis.z).Normalized());
            if (def.hasLimits) { s.mLimitsMin = def.limitMin; s.mLimitsMax = def.limitMax; }
            return s.Create(a, b);
        }
        case JointType::Distance: {
            JPH::DistanceConstraintSettings s;
            s.mSpace  = JPH::EConstraintSpace::WorldSpace;
            s.mPoint1 = toJolt(def.anchorA);
            s.mPoint2 = toJolt(def.anchorB);
            s.mMinDistance = def.minDist;   // negative = use the creation distance
            s.mMaxDistance = def.maxDist;
            if (def.hasSpring) {
                s.mLimitsSpringSettings.mFrequency = def.springHz;
                s.mLimitsSpringSettings.mDamping   = def.springDamping;
            }
            return s.Create(a, b);
        }
        case JointType::Fixed: {
            JPH::FixedConstraintSettings s;
            s.mSpace = JPH::EConstraintSpace::WorldSpace;
            s.mAutoDetectPoint = true;   // weld in the current relative pose
            return s.Create(a, b);
        }
        case JointType::Cone: {
            JPH::ConeConstraintSettings s;
            s.mSpace  = JPH::EConstraintSpace::WorldSpace;
            s.mPoint1 = s.mPoint2 = toJolt(def.anchorA);
            JPH::Vec3 axis = JPH::Vec3(def.axis.x, def.axis.y, def.axis.z).Normalized();
            s.mTwistAxis1 = s.mTwistAxis2 = axis;
            s.mHalfConeAngle = def.coneAngle;
            return s.Create(a, b);
        }
        case JointType::SwingTwist: {
            JPH::SwingTwistConstraintSettings s;
            s.mSpace     = JPH::EConstraintSpace::WorldSpace;
            s.mPosition1 = s.mPosition2 = toJolt(def.anchorA);
            JPH::Vec3 axis = JPH::Vec3(def.axis.x, def.axis.y, def.axis.z).Normalized();
            s.mTwistAxis1 = s.mTwistAxis2 = axis;
            JPH::Vec3 plane = axis.GetNormalizedPerpendicular();
            s.mPlaneAxis1 = s.mPlaneAxis2 = plane;
            // Circular swing cone (normal == plane half-angle).
            s.mNormalHalfConeAngle = s.mPlaneHalfConeAngle = def.coneAngle;
            s.mTwistMinAngle = def.twistMin;
            s.mTwistMaxAngle = def.twistMax;
            return s.Create(a, b);
        }
        case JointType::SixDof: {
            JPH::SixDOFConstraintSettings s;
            s.mSpace     = JPH::EConstraintSpace::WorldSpace;
            s.mPosition1 = s.mPosition2 = toJolt(def.anchorA);
            // World-aligned constraint frame (default mAxisX/Y). Per axis:
            // (0, 0) range = fixed, a real range = limited, free flag = free.
            using EAxis = JPH::SixDOFConstraintSettings::EAxis;
            const float tmin[3] = { def.sixTransMin.x, def.sixTransMin.y, def.sixTransMin.z };
            const float tmax[3] = { def.sixTransMax.x, def.sixTransMax.y, def.sixTransMax.z };
            const float rmin[3] = { def.sixRotMin.x,  def.sixRotMin.y,  def.sixRotMin.z };
            const float rmax[3] = { def.sixRotMax.x,  def.sixRotMax.y,  def.sixRotMax.z };
            for (int i = 0; i < 3; i++) {
                EAxis t = (EAxis)(EAxis::TranslationX + i);
                EAxis r = (EAxis)(EAxis::RotationX + i);
                if (def.sixTransFree)                          s.MakeFreeAxis(t);
                else if (tmin[i] == 0.0f && tmax[i] == 0.0f)   s.MakeFixedAxis(t);
                else                                           s.SetLimitedAxis(t, tmin[i], tmax[i]);
                if (def.sixRotFree)                            s.MakeFreeAxis(r);
                else if (rmin[i] == 0.0f && rmax[i] == 0.0f)   s.MakeFixedAxis(r);
                else                                           s.SetLimitedAxis(r, rmin[i], rmax[i]);
            }
            return s.Create(a, b);
        }
        case JointType::Gear:
        case JointType::RackAndPinion:
            // Built through addGearJoint / addRackAndPinionJoint (they connect
            // two EXISTING joints), never through this path.
            return nullptr;
    }
    return nullptr;
}

// World point/dir -> a body's local space. For JPH::Body::sFixedToWorld
// (identity pose) this is a no-op, so the world-fixed side stores world values.
inline JPH::Vec3 toLocalPoint(const JPH::Body& body, const JPH::Vec3& worldPoint) {
    return body.GetRotation().Conjugated() * (worldPoint - JPH::Vec3(body.GetPosition()));
}
inline JPH::Vec3 toLocalDir(const JPH::Body& body, const JPH::Vec3& worldDir) {
    return body.GetRotation().Conjugated() * worldDir;
}

} // anonymous namespace

PhysicsJoint PhysicsWorld::addJoint(const PhysicsBody& a, const PhysicsBody& b, const Joint& def) {
    if (!impl_->initialized || !a.isValid() || !b.isValid()) {
        logWarning() << "tcxPhysics: addJoint needs two valid bodies.";
        return PhysicsJoint();
    }
    return addJointInternal(a.getId(), b.getId(), def);
}

PhysicsJoint PhysicsWorld::addJoint(const PhysicsBody& a, const Joint& def) {
    if (!impl_->initialized || !a.isValid()) {
        logWarning() << "tcxPhysics: addJoint needs a valid body.";
        return PhysicsJoint();
    }
    // The world is the BASE (Jolt body 1): positive motor values then drive `a`
    // along/around the axis, which is what you expect of "jointed to the world".
    return addJointInternal(PhysicsBody::kInvalidId, a.getId(), def);
}

PhysicsJoint PhysicsWorld::addJointInternal(uint32_t idA, uint32_t idB, const Joint& def) {
    uint64_t jointId = 0;
    {
        TC_LOCK_GUARD(impl_->simMutex);
        auto& sys = impl_->physicsSystem;
        const JPH::BodyLockInterface& lockIf = sys.GetBodyLockInterface();

        // Either side may be the immovable world body (invalid id), never both.
        // Jolt convention: relative measures (hinge angle, slider position, motor
        // drive) are of body 2 RELATIVE TO body 1 — so body A is the "base" and
        // body B is the side that moves positively.
        JPH::Body* bodyA = &JPH::Body::sFixedToWorld;
        JPH::Body* bodyB = &JPH::Body::sFixedToWorld;
        // Lock BOTH bodies through ONE multi-lock. Two independent BodyLockWrite
        // calls deadlock when the two body IDs happen to hash to the same body
        // mutex (BodyLockInterfaceLocking buckets bodies into a fixed mutex
        // array): the second LockWrite re-locks a non-recursive SharedMutex the
        // first already holds, and std::shared_mutex::lock() then waits on its
        // internal condition_variable forever. BodyLockMultiWrite builds a
        // deduplicated mutex mask and locks each mutex exactly once, so a shared
        // bucket is safe. The collision is rare with many mutexes (native) but
        // common with few (web), which is why this only ever hung on web.
        JPH::BodyID ids[2] = {
            idA != PhysicsBody::kInvalidId ? JPH::BodyID(idA) : JPH::BodyID(),
            idB != PhysicsBody::kInvalidId ? JPH::BodyID(idB) : JPH::BodyID(),
        };
        JPH::BodyLockMultiWrite multiLock(lockIf, ids, 2);
        if (idA != PhysicsBody::kInvalidId) {
            bodyA = multiLock.GetBody(0);
            if (!bodyA) return PhysicsJoint();
        }
        if (idB != PhysicsBody::kInvalidId) {
            bodyB = multiLock.GetBody(1);
            if (!bodyB) return PhysicsJoint();
        }

        JPH::TwoBodyConstraint* c = createJoltConstraint(def, *bodyA, *bodyB);
        if (!c) return PhysicsJoint();
        sys.AddConstraint(c);   // Jolt ref-counts and simulates it from here

        Impl::JointRec rec;
        rec.id    = impl_->nextJointId++;
        rec.constraint = c;
        rec.bodyA = idA;
        rec.bodyB = idB;
        rec.type  = def.type;
        // Display anchors: the explicit ones for point/hinge/distance; the bodies'
        // poses at creation for the auto-detected types (slider / fixed).
        bool autoDetect = (def.type == JointType::Slider || def.type == JointType::Fixed);
        JPH::Vec3 wa = autoDetect ? JPH::Vec3(bodyA->GetPosition()) : JPH::Vec3(toJolt(def.anchorA));
        JPH::Vec3 wb = autoDetect ? JPH::Vec3(bodyB->GetPosition()) : JPH::Vec3(toJolt(def.anchorB));
        if (autoDetect && idA == PhysicsBody::kInvalidId) wa = wb;   // world base: show at B
        if (autoDetect && idB == PhysicsBody::kInvalidId) wb = wa;
        rec.localAnchorA = toLocalPoint(*bodyA, wa);
        rec.localAnchorB = toLocalPoint(*bodyB, wb);
        rec.localAxisA   = toLocalDir(*bodyA, JPH::Vec3(def.axis.x, def.axis.y, def.axis.z).Normalized());
        rec.breakForce   = def.breakForceN;
        rec.breakTorque  = def.breakTorqueNm;
        impl_->joints.push_back(rec);
        jointId = rec.id;
    }   // body locks released here — the motor command below re-locks via ActivateBody

    // Creation-time motor (Joint::hinge(...).motor(velocity)).
    if (def.hasMotor) jointMotorCommand(jointId, 0, def.motorVelocity, def.motorMaxForce);

    return PhysicsJoint(this, jointId, alive_);
}

PhysicsJoint PhysicsWorld::addGearJoint(const PhysicsJoint& hingeA, const PhysicsJoint& hingeB, float ratio) {
    if (!impl_->initialized) return PhysicsJoint();
    uint64_t jointId = 0;
    {
        TC_LOCK_GUARD(impl_->simMutex);
        auto* ra = impl_->find(hingeA.getId());
        auto* rb = impl_->find(hingeB.getId());
        if (!ra || !rb || ra->type != JointType::Hinge || rb->type != JointType::Hinge) {
            logWarning() << "tcxPhysics: addGearJoint needs two live HINGE joints.";
            return PhysicsJoint();
        }
        // The wheels are the moving sides (body B) of each hinge.
        uint32_t wheelA = ra->bodyB, wheelB = rb->bodyB;
        const JPH::BodyLockInterface& lockIf = impl_->physicsSystem.GetBodyLockInterface();
        // One multi-lock — two separate BodyLockWrite self-deadlock when the IDs
        // share a body-mutex bucket (see addJointInternal).
        JPH::BodyID ids[2] = { JPH::BodyID(wheelA), JPH::BodyID(wheelB) };
        JPH::BodyLockMultiWrite multiLock(lockIf, ids, 2);
        JPH::Body* bodyWheelA = multiLock.GetBody(0);
        JPH::Body* bodyWheelB = multiLock.GetBody(1);
        if (!bodyWheelA || !bodyWheelB) return PhysicsJoint();

        JPH::GearConstraintSettings s;
        s.mSpace      = JPH::EConstraintSpace::WorldSpace;
        s.mHingeAxis1 = impl_->axisWorld(*ra);
        s.mHingeAxis2 = impl_->axisWorld(*rb);
        s.mRatio      = ratio;
        auto* c = static_cast<JPH::GearConstraint*>(s.Create(*bodyWheelA, *bodyWheelB));
        // Knowing the hinges lets Jolt correct angular drift between the wheels.
        c->SetConstraints(ra->constraint.GetPtr(), rb->constraint.GetPtr());
        impl_->physicsSystem.AddConstraint(c);

        Impl::JointRec rec;
        rec.id = impl_->nextJointId++;
        rec.constraint = c;
        rec.bodyA = wheelA;
        rec.bodyB = wheelB;
        rec.type  = JointType::Gear;
        rec.localAnchorA = JPH::Vec3::sZero();   // draw between the wheel centres
        rec.localAnchorB = JPH::Vec3::sZero();
        rec.localAxisA   = ra->localAxisA;
        impl_->joints.push_back(rec);
        jointId = rec.id;
    }
    return PhysicsJoint(this, jointId, alive_);
}

PhysicsJoint PhysicsWorld::addRackAndPinionJoint(const PhysicsJoint& pinionHinge,
                                                 const PhysicsJoint& rackSlider, float ratio) {
    if (!impl_->initialized) return PhysicsJoint();
    uint64_t jointId = 0;
    {
        TC_LOCK_GUARD(impl_->simMutex);
        auto* rp = impl_->find(pinionHinge.getId());
        auto* rr = impl_->find(rackSlider.getId());
        if (!rp || !rr || rp->type != JointType::Hinge || rr->type != JointType::Slider) {
            logWarning() << "tcxPhysics: addRackAndPinionJoint needs a HINGE (pinion) and a SLIDER (rack).";
            return PhysicsJoint();
        }
        uint32_t pinion = rp->bodyB, rack = rr->bodyB;   // the moving sides
        const JPH::BodyLockInterface& lockIf = impl_->physicsSystem.GetBodyLockInterface();
        // One multi-lock — two separate BodyLockWrite self-deadlock when the IDs
        // share a body-mutex bucket (see addJointInternal).
        JPH::BodyID ids[2] = { JPH::BodyID(pinion), JPH::BodyID(rack) };
        JPH::BodyLockMultiWrite multiLock(lockIf, ids, 2);
        JPH::Body* bodyPinion = multiLock.GetBody(0);
        JPH::Body* bodyRack   = multiLock.GetBody(1);
        if (!bodyPinion || !bodyRack) return PhysicsJoint();

        JPH::RackAndPinionConstraintSettings s;
        s.mSpace      = JPH::EConstraintSpace::WorldSpace;
        s.mHingeAxis  = impl_->axisWorld(*rp);
        s.mSliderAxis = impl_->axisWorld(*rr);
        s.mRatio      = ratio;
        auto* c = static_cast<JPH::RackAndPinionConstraint*>(s.Create(*bodyPinion, *bodyRack));
        c->SetConstraints(rp->constraint.GetPtr(), rr->constraint.GetPtr());
        impl_->physicsSystem.AddConstraint(c);

        Impl::JointRec rec;
        rec.id = impl_->nextJointId++;
        rec.constraint = c;
        rec.bodyA = pinion;
        rec.bodyB = rack;
        rec.type  = JointType::RackAndPinion;
        rec.localAnchorA = JPH::Vec3::sZero();
        rec.localAnchorB = JPH::Vec3::sZero();
        rec.localAxisA   = rp->localAxisA;
        impl_->joints.push_back(rec);
        jointId = rec.id;
    }
    return PhysicsJoint(this, jointId, alive_);
}

void PhysicsWorld::removeJoint(const PhysicsJoint& joint) {
    removeJointById(joint.getId());
}

void PhysicsWorld::removeJointById(uint64_t id) {
    if (!impl_->initialized) return;
    TC_LOCK_GUARD(impl_->simMutex);
    auto& v = impl_->joints;
    for (auto it = v.begin(); it != v.end(); ++it) {
        if (it->id == id) {
            impl_->physicsSystem.RemoveConstraint(it->constraint.GetPtr());
            v.erase(it);
            return;
        }
    }
}

std::vector<PhysicsJoint> PhysicsWorld::getJoints() const {
    std::vector<PhysicsJoint> out;
    if (!impl_->initialized) return out;
    TC_LOCK_GUARD(impl_->simMutex);
    out.reserve(impl_->joints.size());
    for (const auto& r : impl_->joints)
        out.emplace_back(const_cast<PhysicsWorld*>(this), r.id, alive_);
    return out;
}

std::vector<PhysicsJoint> PhysicsWorld::getJointsForBody(uint32_t bodyId) const {
    std::vector<PhysicsJoint> out;
    if (!impl_->initialized || bodyId == PhysicsBody::kInvalidId) return out;
    TC_LOCK_GUARD(impl_->simMutex);
    for (const auto& r : impl_->joints)
        if (r.bodyA == bodyId || r.bodyB == bodyId)
            out.emplace_back(const_cast<PhysicsWorld*>(this), r.id, alive_);
    return out;
}

bool PhysicsWorld::hasJoint(uint64_t id) const {
    if (!impl_->initialized) return false;
    TC_LOCK_GUARD(impl_->simMutex);
    return impl_->find(id) != nullptr;
}

JointType PhysicsWorld::getJointType(uint64_t id) const {
    if (!impl_->initialized) return JointType::Point;
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->find(id);
    return r ? r->type : JointType::Point;
}

uint32_t PhysicsWorld::getJointBodyA(uint64_t id) const {
    if (!impl_->initialized) return PhysicsBody::kInvalidId;
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->find(id);
    return r ? r->bodyA : PhysicsBody::kInvalidId;
}

uint32_t PhysicsWorld::getJointBodyB(uint64_t id) const {
    if (!impl_->initialized) return PhysicsBody::kInvalidId;
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->find(id);
    return r ? r->bodyB : PhysicsBody::kInvalidId;
}

Vec3 PhysicsWorld::getJointAnchorA(uint64_t id) const {
    if (!impl_->initialized) return Vec3();
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->find(id);
    return r ? toTc(impl_->anchorWorld(r->bodyA, r->localAnchorA)) : Vec3();
}

Vec3 PhysicsWorld::getJointAnchorB(uint64_t id) const {
    if (!impl_->initialized) return Vec3();
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->find(id);
    return r ? toTc(impl_->anchorWorld(r->bodyB, r->localAnchorB)) : Vec3();
}

Vec3 PhysicsWorld::getJointAxis(uint64_t id) const {
    if (!impl_->initialized) return Vec3(0, 1, 0);
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->find(id);
    if (!r) return Vec3(0, 1, 0);
    if (r->bodyA == PhysicsBody::kInvalidId) return toTc(r->localAxisA);  // world base
    JPH::Quat rot = impl_->bodies().GetRotation(JPH::BodyID(r->bodyA));
    return toTc(rot * r->localAxisA);
}

// --- breakable joints ----------------------------------------------------------
namespace {

// Total linear / angular impulse (N·s, N·m·s) a constraint applied during the
// last step, per type (Jolt exposes the solver lambdas).
void constraintImpulses(JPH::Constraint* c, JointType type, float& outLinear, float& outAngular) {
    outLinear = 0.0f;
    outAngular = 0.0f;
    switch (type) {
        case JointType::Point:
            outLinear = static_cast<JPH::PointConstraint*>(c)->GetTotalLambdaPosition().Length();
            break;
        case JointType::Hinge: {
            auto* h = static_cast<JPH::HingeConstraint*>(c);
            outLinear = h->GetTotalLambdaPosition().Length();
            auto r = h->GetTotalLambdaRotation();   // the 2 locked rotation axes
            outAngular = std::sqrt(r[0] * r[0] + r[1] * r[1]);
            break;
        }
        case JointType::Slider: {
            auto* s = static_cast<JPH::SliderConstraint*>(c);
            auto p = s->GetTotalLambdaPosition();   // the 2 locked translation axes
            outLinear = std::sqrt(p[0] * p[0] + p[1] * p[1]);
            outAngular = s->GetTotalLambdaRotation().Length();
            break;
        }
        case JointType::Distance:
            outLinear = std::abs(static_cast<JPH::DistanceConstraint*>(c)->GetTotalLambdaPosition());
            break;
        case JointType::Fixed: {
            auto* f = static_cast<JPH::FixedConstraint*>(c);
            outLinear  = f->GetTotalLambdaPosition().Length();
            outAngular = f->GetTotalLambdaRotation().Length();
            break;
        }
        case JointType::Cone: {
            auto* k = static_cast<JPH::ConeConstraint*>(c);
            outLinear  = k->GetTotalLambdaPosition().Length();
            outAngular = std::abs(k->GetTotalLambdaRotation());
            break;
        }
        case JointType::SwingTwist: {
            auto* st = static_cast<JPH::SwingTwistConstraint*>(c);
            outLinear = st->GetTotalLambdaPosition().Length();
            float tw = st->GetTotalLambdaTwist(), sy = st->GetTotalLambdaSwingY(), sz = st->GetTotalLambdaSwingZ();
            outAngular = std::sqrt(tw * tw + sy * sy + sz * sz);
            break;
        }
        case JointType::SixDof: {
            auto* sd = static_cast<JPH::SixDOFConstraint*>(c);
            outLinear  = sd->GetTotalLambdaPosition().Length();
            outAngular = sd->GetTotalLambdaRotation().Length();
            break;
        }
        case JointType::Gear:
            outAngular = std::abs(static_cast<JPH::GearConstraint*>(c)->GetTotalLambda());
            break;
        case JointType::RackAndPinion:
            outAngular = std::abs(static_cast<JPH::RackAndPinionConstraint*>(c)->GetTotalLambda());
            break;
    }
}

} // anonymous namespace

void PhysicsWorld::checkJointBreaks(float dt) {
    if (!impl_->initialized || dt <= 0.0f) return;
    std::vector<JointBreakEventArgs> broken;
    {
        TC_LOCK_GUARD(impl_->simMutex);
        auto& v = impl_->joints;
        for (auto it = v.begin(); it != v.end();) {
            // Negative threshold = unbreakable (the default). Zero is a VALID
            // threshold: the joint snaps under any load at all — so a toughness
            // stat can be driven continuously down to "breaks instantly".
            if (it->breakForce < 0.0f && it->breakTorque < 0.0f) { ++it; continue; }
            float impulse = 0.0f, angImpulse = 0.0f;
            constraintImpulses(it->constraint.GetPtr(), it->type, impulse, angImpulse);
            float force  = impulse / dt;       // impulse over one step -> force
            float torque = angImpulse / dt;
            bool snap = (it->breakForce  >= 0.0f && force  > it->breakForce)
                     || (it->breakTorque >= 0.0f && torque > it->breakTorque);
            if (!snap) { ++it; continue; }

            JointBreakEventArgs e;
            e.jointId = it->id;
            e.type    = it->type;
            e.bodyA   = (it->bodyA != PhysicsBody::kInvalidId) ? PhysicsBody(this, it->bodyA) : PhysicsBody();
            e.bodyB   = (it->bodyB != PhysicsBody::kInvalidId) ? PhysicsBody(this, it->bodyB) : PhysicsBody();
            e.point   = toTc(impl_->anchorWorld(it->bodyA, it->localAnchorA));
            e.force   = force;
            e.torque  = torque;
            impl_->physicsSystem.RemoveConstraint(it->constraint.GetPtr());
            it = v.erase(it);
            broken.push_back(e);
        }
    }
    // Notify OUTSIDE the sim lock — handlers may freely talk to the world.
    for (auto& e : broken) jointBroke.notify(e);
}

// --- joint motors ------------------------------------------------------------
// mode: 0 = drive at velocity, 1 = drive to target, 2 = off.
void PhysicsWorld::jointMotorCommand(uint64_t id, int mode, float value, float maxForce) {
    if (!impl_->initialized) return;
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->find(id);
    if (!r) return;

    JPH::EMotorState state = (mode == 0) ? JPH::EMotorState::Velocity
                           : (mode == 1) ? JPH::EMotorState::Position
                                         : JPH::EMotorState::Off;
    if (r->type == JointType::Hinge) {
        auto* h = static_cast<JPH::HingeConstraint*>(r->constraint.GetPtr());
        if (maxForce >= 0.0f) h->GetMotorSettings().SetTorqueLimit(maxForce);
        if (mode == 0) h->SetTargetAngularVelocity(value);
        if (mode == 1) h->SetTargetAngle(value);
        h->SetMotorState(state);
    } else if (r->type == JointType::Slider) {
        auto* s = static_cast<JPH::SliderConstraint*>(r->constraint.GetPtr());
        if (maxForce >= 0.0f) s->GetMotorSettings().SetForceLimit(maxForce);
        if (mode == 0) s->SetTargetVelocity(value);
        if (mode == 1) s->SetTargetPosition(value);
        s->SetMotorState(state);
    } else {
        logWarning() << "tcxPhysics: only hinge / slider joints have a motor.";
        return;
    }
    // Wake both bodies so the drive takes effect immediately.
    if (r->bodyA != PhysicsBody::kInvalidId) impl_->bodies().ActivateBody(JPH::BodyID(r->bodyA));
    if (r->bodyB != PhysicsBody::kInvalidId) impl_->bodies().ActivateBody(JPH::BodyID(r->bodyB));
}

void PhysicsWorld::setJointMotorVelocity(uint64_t id, float velocity, float maxForce) {
    jointMotorCommand(id, 0, velocity, maxForce);
}

void PhysicsWorld::setJointMotorTarget(uint64_t id, float target, float maxForce) {
    jointMotorCommand(id, 1, target, maxForce);
}

void PhysicsWorld::setJointMotorOff(uint64_t id) {
    jointMotorCommand(id, 2, 0.0f, -1.0f);
}

// ---------------------------------------------------------------------------
// Characters (Jolt CharacterVirtual)
// ---------------------------------------------------------------------------
PhysicsCharacter PhysicsWorld::addCharacter(const Vec3& position, float radius,
                                            float cylinderHeight, float maxSlopeAngle) {
    if (!impl_->initialized) {
        logWarning() << "tcxPhysics: addCharacter before setup().";
        return PhysicsCharacter();
    }
    radius = std::max(0.05f, radius);
    cylinderHeight = std::max(0.0f, cylinderHeight);

    JPH::Ref<JPH::Shape> capsule = new JPH::CapsuleShape(cylinderHeight * 0.5f, radius);

    JPH::CharacterVirtualSettings settings;
    settings.mShape = capsule;
    settings.mMaxSlopeAngle = maxSlopeAngle;
    // Inner kinematic body: raycasts hit the character, sensors see it, and it
    // helps push dynamic bodies out of the way.
    settings.mInnerBodyShape = capsule;
    settings.mInnerBodyLayer = defaultObjectLayer();

    uint64_t charId = 0;
    {
        TC_LOCK_GUARD(impl_->simMutex);
        Impl::CharRec rec;
        rec.id = impl_->nextCharId++;
        rec.ch = new JPH::CharacterVirtual(&settings, toJolt(position),
                                           JPH::Quat::sIdentity(), 0, &impl_->physicsSystem);
        impl_->characters.push_back(rec);
        charId = rec.id;
    }
    return PhysicsCharacter(this, charId, alive_);
}

void PhysicsWorld::removeCharacter(const PhysicsCharacter& character) {
    if (!impl_->initialized) return;
    TC_LOCK_GUARD(impl_->simMutex);
    auto& v = impl_->characters;
    for (auto it = v.begin(); it != v.end(); ++it) {
        if (it->id == character.getId()) { v.erase(it); return; }   // Ref releases (inner body too)
    }
}

void PhysicsWorld::updateCharacters(float dt) {
    if (!impl_->initialized || dt <= 0.0f || impl_->characters.empty()) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::Vec3 gravity = impl_->physicsSystem.GetGravity();
    const JPH::Vec3 up = JPH::Vec3::sAxisY();

    for (auto& rec : impl_->characters) {
        JPH::CharacterVirtual& ch = *rec.ch;
        ch.UpdateGroundVelocity();   // moving platforms

        JPH::Vec3 current = ch.GetLinearVelocity();
        bool grounded = (ch.GetGroundState() == JPH::CharacterVirtual::EGroundState::OnGround);

        JPH::Vec3 newVel;
        if (grounded && (current.GetY() - ch.GetGroundVelocity().GetY()) < 0.1f) {
            // Standing: inherit the ground's velocity (riding platforms)...
            newVel = ch.GetGroundVelocity();
            // ...and take a pending jump.
            if (rec.jumpSpeed > 0.0f) newVel += up * rec.jumpSpeed;
        } else {
            // Airborne: keep only the vertical component (input gives horizontal).
            newVel = up * current.Dot(up);
        }
        rec.jumpSpeed = 0.0f;
        newVel += gravity * dt;
        newVel += rec.moveInput;
        ch.SetLinearVelocity(newVel);

        // Slope clamp + move + stair stepping + stick-to-floor, all in one.
        JPH::CharacterVirtual::ExtendedUpdateSettings us;
        ch.ExtendedUpdate(dt, gravity, us,
            impl_->physicsSystem.GetDefaultBroadPhaseLayerFilter(defaultObjectLayer()),
            impl_->physicsSystem.GetDefaultLayerFilter(defaultObjectLayer()),
            {}, {}, *impl_->tempAllocator);
    }
}

bool PhysicsWorld::hasCharacter(uint64_t id) const {
    if (!impl_->initialized) return false;
    TC_LOCK_GUARD(impl_->simMutex);
    return impl_->findChar(id) != nullptr;
}

void PhysicsWorld::setCharacterMoveInput(uint64_t id, const Vec3& velocity) {
    if (!impl_->initialized) return;
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->findChar(id);
    if (r) r->moveInput = JPH::Vec3(velocity.x, 0.0f, velocity.z);   // horizontal only
}

void PhysicsWorld::characterJump(uint64_t id, float speed) {
    if (!impl_->initialized) return;
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->findChar(id);
    if (r) r->jumpSpeed = std::max(0.0f, speed);
}

bool PhysicsWorld::isCharacterGrounded(uint64_t id) const {
    if (!impl_->initialized) return false;
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->findChar(id);
    return r && r->ch->GetGroundState() == JPH::CharacterVirtual::EGroundState::OnGround;
}

bool PhysicsWorld::isCharacterOnSteepSlope(uint64_t id) const {
    if (!impl_->initialized) return false;
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->findChar(id);
    return r && r->ch->GetGroundState() == JPH::CharacterVirtual::EGroundState::OnSteepGround;
}

Vec3 PhysicsWorld::getCharacterGroundNormal(uint64_t id) const {
    if (!impl_->initialized) return Vec3(0, 1, 0);
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->findChar(id);
    return r ? toTc(r->ch->GetGroundNormal()) : Vec3(0, 1, 0);
}

Vec3 PhysicsWorld::getCharacterPosition(uint64_t id) const {
    if (!impl_->initialized) return Vec3();
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->findChar(id);
    return r ? toTc(r->ch->GetPosition()) : Vec3();
}

void PhysicsWorld::setCharacterPosition(uint64_t id, const Vec3& p) {
    if (!impl_->initialized) return;
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->findChar(id);
    if (r) r->ch->SetPosition(toJolt(p));
}

Vec3 PhysicsWorld::getCharacterLinearVelocity(uint64_t id) const {
    if (!impl_->initialized) return Vec3();
    TC_LOCK_GUARD(impl_->simMutex);
    auto* r = impl_->findChar(id);
    return r ? toTc(r->ch->GetLinearVelocity()) : Vec3();
}

Vec3 PhysicsWorld::getBodyPosition(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return Vec3();
    TC_LOCK_GUARD(impl_->simMutex);
    return toTc(impl_->bodies().GetPosition(JPH::BodyID(id)));
}

Quaternion PhysicsWorld::getBodyRotation(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return Quaternion::identity();
    TC_LOCK_GUARD(impl_->simMutex);
    return toTc(impl_->bodies().GetRotation(JPH::BodyID(id)));
}

Vec3 PhysicsWorld::getBodySize(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return Vec3();
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    JPH::RefConst<JPH::Shape> shape = impl_->bodies().GetShape(bid);
    if (shape == nullptr) return Vec3();
    JPH::AABox bounds = shape->GetLocalBounds();
    return toTc(bounds.GetSize());
}

// ---------------------------------------------------------------------------
// Per-body forces / velocity / material (forwarded from PhysicsBody)
// ---------------------------------------------------------------------------
// Applying a force/impulse/velocity to a non-dynamic body is a no-op; we also
// wake the body so it actually responds. `isDynamic` is checked under the lock.
namespace {
inline bool isDynamicLocked(JPH::BodyInterface& bi, JPH::BodyID bid) {
    return bi.GetMotionType(bid) == JPH::EMotionType::Dynamic;
}
} // namespace

void PhysicsWorld::applyForceToBody(uint32_t id, const Vec3& f) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    if (!isDynamicLocked(impl_->bodies(), bid)) return;
    impl_->bodies().ActivateBody(bid);
    impl_->bodies().AddForce(bid, JPH::Vec3(f.x, f.y, f.z));
}

void PhysicsWorld::applyForceToBody(uint32_t id, const Vec3& f, const Vec3& pt) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    if (!isDynamicLocked(impl_->bodies(), bid)) return;
    impl_->bodies().ActivateBody(bid);
    impl_->bodies().AddForce(bid, JPH::Vec3(f.x, f.y, f.z), toJolt(pt));
}

void PhysicsWorld::applyTorqueToBody(uint32_t id, const Vec3& t) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    if (!isDynamicLocked(impl_->bodies(), bid)) return;
    impl_->bodies().ActivateBody(bid);
    impl_->bodies().AddTorque(bid, JPH::Vec3(t.x, t.y, t.z));
}

void PhysicsWorld::applyImpulseToBody(uint32_t id, const Vec3& i) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    if (!isDynamicLocked(impl_->bodies(), bid)) return;
    impl_->bodies().ActivateBody(bid);
    impl_->bodies().AddImpulse(bid, JPH::Vec3(i.x, i.y, i.z));
}

void PhysicsWorld::applyImpulseToBody(uint32_t id, const Vec3& i, const Vec3& pt) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    if (!isDynamicLocked(impl_->bodies(), bid)) return;
    impl_->bodies().ActivateBody(bid);
    impl_->bodies().AddImpulse(bid, JPH::Vec3(i.x, i.y, i.z), toJolt(pt));
}

void PhysicsWorld::applyAngularImpulseToBody(uint32_t id, const Vec3& i) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    if (!isDynamicLocked(impl_->bodies(), bid)) return;
    impl_->bodies().ActivateBody(bid);
    impl_->bodies().AddAngularImpulse(bid, JPH::Vec3(i.x, i.y, i.z));
}

void PhysicsWorld::addVelocityToBody(uint32_t id, const Vec3& dv) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    if (impl_->bodies().GetMotionType(bid) == JPH::EMotionType::Static) return;
    impl_->bodies().ActivateBody(bid);
    impl_->bodies().AddLinearVelocity(bid, JPH::Vec3(dv.x, dv.y, dv.z));
}

float PhysicsWorld::getBodyMass(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return 0.0f;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::RefConst<JPH::Shape> shape = impl_->bodies().GetShape(JPH::BodyID(id));
    if (shape == nullptr) return 0.0f;
    // Mass the shape's density implies (we set that density at creation).
    return shape->GetMassProperties().mMass;
}

void PhysicsWorld::setBodyLinearVelocity(uint32_t id, const Vec3& v) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    if (impl_->bodies().GetMotionType(bid) == JPH::EMotionType::Static) return;
    impl_->bodies().ActivateBody(bid);
    impl_->bodies().SetLinearVelocity(bid, JPH::Vec3(v.x, v.y, v.z));
}

Vec3 PhysicsWorld::getBodyLinearVelocity(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return Vec3();
    TC_LOCK_GUARD(impl_->simMutex);
    return toTc(impl_->bodies().GetLinearVelocity(JPH::BodyID(id)));
}

void PhysicsWorld::setBodyAngularVelocity(uint32_t id, const Vec3& v) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    if (impl_->bodies().GetMotionType(bid) == JPH::EMotionType::Static) return;
    impl_->bodies().ActivateBody(bid);
    impl_->bodies().SetAngularVelocity(bid, JPH::Vec3(v.x, v.y, v.z));
}

Vec3 PhysicsWorld::getBodyAngularVelocity(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return Vec3();
    TC_LOCK_GUARD(impl_->simMutex);
    return toTc(impl_->bodies().GetAngularVelocity(JPH::BodyID(id)));
}

void PhysicsWorld::setBodyPosition(uint32_t id, const Vec3& p) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    impl_->bodies().SetPosition(JPH::BodyID(id), toJolt(p), JPH::EActivation::Activate);
}

void PhysicsWorld::setBodyRotation(uint32_t id, const Quaternion& q) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    impl_->bodies().SetRotation(JPH::BodyID(id), toJolt(q), JPH::EActivation::Activate);
}

void PhysicsWorld::setBodyMotionType(uint32_t id, MotionType type) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    JPH::EMotionType mt = JPH::EMotionType::Dynamic;
    switch (type) {
        case MotionType::Static:    mt = JPH::EMotionType::Static;    break;
        case MotionType::Kinematic: mt = JPH::EMotionType::Kinematic; break;
        case MotionType::Dynamic:   mt = JPH::EMotionType::Dynamic;   break;
    }
    // Static bodies sit still; kinematic/dynamic want to be awake to act.
    JPH::EActivation act = (type == MotionType::Static)
        ? JPH::EActivation::DontActivate : JPH::EActivation::Activate;
    impl_->bodies().SetMotionType(bid, mt, act);

    // Keep the dynamicBodies tracking honest: only true Dynamic bodies belong in
    // it (clearDynamicBodies() must not wipe kinematic platforms / static scenery).
    auto& v = impl_->dynamicBodies;
    bool tracked = std::find(v.begin(), v.end(), bid) != v.end();
    if (type == MotionType::Dynamic && !tracked) v.push_back(bid);
    if (type != MotionType::Dynamic && tracked)
        v.erase(std::remove(v.begin(), v.end(), bid), v.end());
}

void PhysicsWorld::moveBodyKinematic(uint32_t id, const Vec3& pos, const Quaternion& rot, float dt) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    if (dt <= 0.0f) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    if (impl_->bodies().GetMotionType(bid) != JPH::EMotionType::Kinematic) {
        // Not kinematic — fall back to a teleport so the call still does the
        // obvious thing instead of silently nothing.
        impl_->bodies().SetPositionAndRotation(bid, toJolt(pos), toJolt(rot),
                                               JPH::EActivation::Activate);
        return;
    }
    impl_->bodies().MoveKinematic(bid, toJolt(pos), toJolt(rot), dt);
}

void PhysicsWorld::setBodyIsSensor(uint32_t id, bool sensor) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyLockWrite bodyLock(impl_->physicsSystem.GetBodyLockInterface(), JPH::BodyID(id));
    if (bodyLock.Succeeded()) bodyLock.GetBody().SetIsSensor(sensor);
}

bool PhysicsWorld::isBodySensor(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return false;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyLockRead bodyLock(impl_->physicsSystem.GetBodyLockInterface(), JPH::BodyID(id));
    return bodyLock.Succeeded() && bodyLock.GetBody().IsSensor();
}

void PhysicsWorld::setBodyUserData(uint32_t id, uint64_t data) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    impl_->bodies().SetUserData(JPH::BodyID(id), data);
}

uint64_t PhysicsWorld::getBodyUserData(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return 0;
    TC_LOCK_GUARD(impl_->simMutex);
    return impl_->bodies().GetUserData(JPH::BodyID(id));
}

void PhysicsWorld::setBodyAllowedDofs(uint32_t id, uint32_t allowedBits) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    allowedBits &= 0x3fu;
    if (allowedBits == 0) {
        logWarning() << "tcxPhysics: allowed DOFs can't all be locked — use a Static body instead.";
        return;
    }
    {
        TC_LOCK_GUARD(impl_->simMutex);
        JPH::BodyLockWrite bodyLock(impl_->physicsSystem.GetBodyLockInterface(), JPH::BodyID(id));
        if (!bodyLock.Succeeded()) return;
        JPH::Body& b = bodyLock.GetBody();
        JPH::MotionProperties* mp = b.GetMotionPropertiesUnchecked();
        if (!mp) return;   // pure static body (shouldn't happen — we allow switching)
        // Recompute mass/inertia for the new DOF set from the shape.
        mp->SetMassProperties((JPH::EAllowedDOFs)allowedBits,
                              b.GetShape()->GetMassProperties());
    }
    // Wake AFTER the body lock is released (ActivateBody re-locks the body).
    TC_LOCK_GUARD(impl_->simMutex);
    if (isDynamicLocked(impl_->bodies(), JPH::BodyID(id)))
        impl_->bodies().ActivateBody(JPH::BodyID(id));
}

uint32_t PhysicsWorld::getBodyAllowedDofs(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return 0x3fu;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyLockRead bodyLock(impl_->physicsSystem.GetBodyLockInterface(), JPH::BodyID(id));
    if (!bodyLock.Succeeded()) return 0x3fu;
    const JPH::MotionProperties* mp = bodyLock.GetBody().GetMotionPropertiesUnchecked();
    return mp ? (uint32_t)mp->GetAllowedDOFs() : 0x3fu;
}

void PhysicsWorld::setBodyCollisionLayer(uint32_t id, int layer) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    if (layer < 0) layer = 0;
    if (layer > 7) layer = 7;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    uint32_t mask = JPH::ObjectLayerPairFilterMask::sGetMask(impl_->bodies().GetObjectLayer(bid));
    impl_->bodies().SetObjectLayer(bid,
        JPH::ObjectLayerPairFilterMask::sGetObjectLayer(1u << layer, mask));
}

int PhysicsWorld::getBodyCollisionLayer(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return 0;
    TC_LOCK_GUARD(impl_->simMutex);
    uint32_t group = JPH::ObjectLayerPairFilterMask::sGetGroup(
        impl_->bodies().GetObjectLayer(JPH::BodyID(id)));
    for (int i = 0; i < 8; i++) if (group & (1u << i)) return i;
    return 0;
}

void PhysicsWorld::setBodyCollisionMask(uint32_t id, uint32_t mask) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    uint32_t group = JPH::ObjectLayerPairFilterMask::sGetGroup(impl_->bodies().GetObjectLayer(bid));
    impl_->bodies().SetObjectLayer(bid,
        JPH::ObjectLayerPairFilterMask::sGetObjectLayer(group, mask & 0xffu));
}

uint32_t PhysicsWorld::getBodyCollisionMask(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return 0xffu;
    TC_LOCK_GUARD(impl_->simMutex);
    return JPH::ObjectLayerPairFilterMask::sGetMask(
        impl_->bodies().GetObjectLayer(JPH::BodyID(id)));
}

void PhysicsWorld::setBodyFriction(uint32_t id, float friction) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    impl_->bodies().SetFriction(JPH::BodyID(id), friction);
}

float PhysicsWorld::getBodyFriction(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return 0.0f;
    TC_LOCK_GUARD(impl_->simMutex);
    return impl_->bodies().GetFriction(JPH::BodyID(id));
}

void PhysicsWorld::setBodyRestitution(uint32_t id, float restitution) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    impl_->bodies().SetRestitution(JPH::BodyID(id), restitution);
}

float PhysicsWorld::getBodyRestitution(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return 0.0f;
    TC_LOCK_GUARD(impl_->simMutex);
    return impl_->bodies().GetRestitution(JPH::BodyID(id));
}

void PhysicsWorld::activateBody(uint32_t id) {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID bid(id);
    if (isDynamicLocked(impl_->bodies(), bid)) impl_->bodies().ActivateBody(bid);
}

bool PhysicsWorld::isBodyActive(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return false;
    TC_LOCK_GUARD(impl_->simMutex);
    return impl_->bodies().IsActive(JPH::BodyID(id));
}

// ---------------------------------------------------------------------------
// Escape hatch — raw Jolt pointers (typed via tcxPhysicsJolt.h)
// ---------------------------------------------------------------------------
// In a const member, impl_-> yields a non-const Impl*, so these hand back
// mutable Jolt objects (the whole point of the hatch) without a const_cast.
void* PhysicsWorld::nativeSystem() const {
    return impl_->initialized ? static_cast<void*>(&impl_->physicsSystem) : nullptr;
}

void* PhysicsWorld::nativeBodyInterface() const {
    return impl_->initialized ? static_cast<void*>(&impl_->bodies()) : nullptr;
}

} // namespace tcx
