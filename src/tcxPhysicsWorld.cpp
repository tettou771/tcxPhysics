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
#include <Jolt/Physics/Collision/ContactListener.h>

// Compile the opt-in escape-hatch header here too, so it can never silently rot
// (it's inline-only — no ODR/runtime cost). Jolt is already on the path in this TU.
#include "tcxPhysicsJolt.h"

#include <thread>
#include <atomic>
#include <chrono>
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

// Two object layers: static scenery vs. moving bodies. Static never collides
// with static, which is most of the broadphase savings.
namespace Layers {
    static constexpr JPH::ObjectLayer NON_MOVING = 0;
    static constexpr JPH::ObjectLayer MOVING     = 1;
    static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
}

namespace BroadPhaseLayers {
    static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
    static constexpr JPH::BroadPhaseLayer MOVING(1);
    static constexpr JPH::uint NUM_LAYERS = 2;
}

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    BPLayerInterfaceImpl() {
        objectToBroadPhase_[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
        objectToBroadPhase_[Layers::MOVING]     = BroadPhaseLayers::MOVING;
    }
    JPH::uint GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::NUM_LAYERS; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
        JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
        return objectToBroadPhase_[inLayer];
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override {
        switch ((JPH::BroadPhaseLayer::Type)inLayer) {
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING: return "NON_MOVING";
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:     return "MOVING";
            default: return "INVALID";
        }
    }
#endif
private:
    JPH::BroadPhaseLayer objectToBroadPhase_[Layers::NUM_LAYERS];
};

class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override {
        switch (inLayer1) {
            case Layers::NON_MOVING: return inLayer2 == BroadPhaseLayers::MOVING;
            case Layers::MOVING:     return true;
            default:                 JPH_ASSERT(false); return false;
        }
    }
};

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override {
        switch (inObject1) {
            case Layers::NON_MOVING: return inObject2 == Layers::MOVING; // static only vs moving
            case Layers::MOVING:     return true;                        // moving vs anything
            default:                 JPH_ASSERT(false); return false;
        }
    }
};

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
class ContactListenerImpl final : public JPH::ContactListener {
public:
    // (bodyA, bodyB, worldPoint, worldNormal, approachSpeed, began)
    std::function<void(uint32_t, uint32_t, const Vec3&, const Vec3&, float, bool)> sink;

    void OnContactAdded(const JPH::Body& b1, const JPH::Body& b2,
                        const JPH::ContactManifold& manifold,
                        JPH::ContactSettings&) override {
        if (!sink) return;
        Vec3 normal = toTc(manifold.mWorldSpaceNormal);
        Vec3 point  = toTc(manifold.GetWorldSpaceContactPointOn1(0));
        // Approach speed along the contact normal — a cheap, useful proxy for
        // "how hard" the hit was (relative impulse isn't known pre-solve).
        float speed = std::abs((b1.GetLinearVelocity() - b2.GetLinearVelocity())
                                   .Dot(manifold.mWorldSpaceNormal));
        sink(b1.GetID().GetIndexAndSequenceNumber(),
             b2.GetID().GetIndexAndSequenceNumber(), point, normal, speed, true);
    }

    void OnContactRemoved(const JPH::SubShapeIDPair& pair) override {
        if (!sink) return;
        // No manifold on removal — point/normal/speed are unknown.
        sink(pair.GetBody1ID().GetIndexAndSequenceNumber(),
             pair.GetBody2ID().GetIndexAndSequenceNumber(),
             Vec3(), Vec3(), 0.0f, false);
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

    // Filters must outlive physicsSystem (it keeps references to them), so they
    // are declared first.
    BPLayerInterfaceImpl              bpLayer;
    ObjectVsBroadPhaseLayerFilterImpl objVsBpFilter;
    ObjectLayerPairFilterImpl         objPairFilter;

    JPH::PhysicsSystem physicsSystem;
    ContactListenerImpl contactListener;

    vector<JPH::BodyID> dynamicBodies; // tracked so clearDynamicBodies() works

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
        bool  began = false;
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

    impl_->physicsSystem.Init(
        maxB, /*numBodyMutexes*/ 0, maxPairs, maxCC,
        impl_->bpLayer, impl_->objVsBpFilter, impl_->objPairFilter);

    impl_->physicsSystem.SetGravity(JPH::Vec3(0.0f, -9.81f, 0.0f));

    // Route Jolt's (worker-thread) contact callbacks into a thread-safe buffer;
    // they are replayed as events on the main thread in dispatchContacts().
    impl_->contactListener.sink =
        [imp = impl_.get()](uint32_t a, uint32_t b, const Vec3& pt,
                            const Vec3& n, float s, bool began) {
            TC_LOCK_GUARD(imp->contactsMutex);
            imp->pending.push_back({a, b, pt, n, s, began});
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

void PhysicsWorld::update(float dt, int collisionSteps) {
    if (!impl_->initialized) return;
    if (impl_->async) {
        logWarning() << "tcxPhysics: update() ignored while async stepping runs "
                        "(call updateAsyncStop() first).";
        return;
    }
    {
        TC_LOCK_GUARD(impl_->simMutex);
        impl_->physicsSystem.Update(dt, std::max(1, collisionSteps),
                                    impl_->tempAllocator.get(), impl_->jobSystem.get());
    }
    dispatchContacts();
}

// ---------------------------------------------------------------------------
// Async stepping
// ---------------------------------------------------------------------------
void PhysicsWorld::updateAsyncStart(float hz) {
    if (!impl_->initialized) {
        logWarning() << "tcxPhysics: updateAsyncStart() before setup() — ignored.";
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
    impl_->frameListener = events().update.listen([this] { dispatchContacts(); });
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
        accum += std::chrono::duration<double>(now - prev).count();
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
    impl_->webAccum += dt;
    if (impl_->webAccum > 0.25) impl_->webAccum = 0.25;
    int steps = 0;
    while (impl_->webAccum >= step && steps < 8) {
        impl_->physicsSystem.Update(step, 1, impl_->tempAllocator.get(),
                                    impl_->jobSystem.get());
        impl_->webAccum -= step;
        ++steps;
    }
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
        if (pc.began) contactBegan.notify(args);
        else          contactEnded.notify(args);
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
        dynamic ? Layers::MOVING : Layers::NON_MOVING);

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
        dynamic ? Layers::MOVING : Layers::NON_MOVING);

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
        dynamic ? Layers::MOVING : Layers::NON_MOVING);

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
        dynamic ? Layers::MOVING : Layers::NON_MOVING);

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
        dynamic ? Layers::MOVING : Layers::NON_MOVING);

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
        JPH::EMotionType::Static, Layers::NON_MOVING);

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

void PhysicsWorld::removeBody(const PhysicsBody& body) {
    if (!impl_->initialized || !body.isValid()) return;
    TC_LOCK_GUARD(impl_->simMutex);
    JPH::BodyID id(body.getId());
    impl_->bodies().RemoveBody(id);
    impl_->bodies().DestroyBody(id);
    auto& v = impl_->dynamicBodies;
    v.erase(std::remove(v.begin(), v.end(), id), v.end());
}

void PhysicsWorld::clearDynamicBodies() {
    if (!impl_->initialized) return;
    TC_LOCK_GUARD(impl_->simMutex);
    for (JPH::BodyID id : impl_->dynamicBodies) {
        impl_->bodies().RemoveBody(id);
        impl_->bodies().DestroyBody(id);
    }
    impl_->dynamicBodies.clear();
}

int PhysicsWorld::getNumBodies() const {
    if (!impl_->initialized) return 0;
    return (int)impl_->physicsSystem.GetNumBodies();
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
