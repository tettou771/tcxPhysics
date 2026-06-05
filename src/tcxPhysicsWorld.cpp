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
#include <Jolt/Physics/Body/BodyCreationSettings.h>

#include <thread>
#include <algorithm>
#include <vector>
#include <cstdarg>
#include <cstdio>

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

constexpr JPH::uint cMaxPhysicsJobs     = 2048;
constexpr JPH::uint cMaxPhysicsBarriers = 8;

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

    vector<JPH::BodyID> dynamicBodies; // tracked so clearDynamicBodies() works

    JPH::BodyInterface& bodies() { return physicsSystem.GetBodyInterface(); }
    const JPH::BodyInterface& bodies() const { return physicsSystem.GetBodyInterface(); }
};

// ---------------------------------------------------------------------------
// PhysicsWorld
// ---------------------------------------------------------------------------
PhysicsWorld::PhysicsWorld() : impl_(make_unique<Impl>()) {}

PhysicsWorld::~PhysicsWorld() {
    if (impl_ && impl_->initialized) {
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
    impl_->initialized = true;
}

PhysicsWorld& PhysicsWorld::setGravity(const Vec3& g) {
    if (impl_->initialized) impl_->physicsSystem.SetGravity(JPH::Vec3(g.x, g.y, g.z));
    return *this;
}

Vec3 PhysicsWorld::getGravity() const {
    if (!impl_->initialized) return Vec3(0, -9.81f, 0);
    return toTc(impl_->physicsSystem.GetGravity());
}

void PhysicsWorld::update(float dt, int collisionSteps) {
    if (!impl_->initialized) return;
    impl_->physicsSystem.Update(dt, std::max(1, collisionSteps),
                                impl_->tempAllocator.get(), impl_->jobSystem.get());
}

PhysicsBody PhysicsWorld::addBox(const Vec3& position, const Vec3& size, bool dynamic) {
    if (!impl_->initialized) return PhysicsBody();

    JPH::Vec3 half(std::max(0.001f, size.x * 0.5f),
                   std::max(0.001f, size.y * 0.5f),
                   std::max(0.001f, size.z * 0.5f));
    // Convex radius must be <= the smallest half extent.
    float minHalf = std::min({half.GetX(), half.GetY(), half.GetZ()});
    float convexRadius = std::min(JPH::cDefaultConvexRadius, minHalf * 0.5f);

    JPH::BoxShapeSettings shapeSettings(half, convexRadius);
    JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) {
        logError() << "tcxPhysics: box shape error: " << result.GetError().c_str();
        return PhysicsBody();
    }

    JPH::BodyCreationSettings bcs(
        result.Get(), toJolt(position), JPH::Quat::sIdentity(),
        dynamic ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static,
        dynamic ? Layers::MOVING : Layers::NON_MOVING);

    JPH::BodyID id = impl_->bodies().CreateAndAddBody(
        bcs, dynamic ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
    if (id.IsInvalid()) return PhysicsBody();
    if (dynamic) impl_->dynamicBodies.push_back(id);
    return PhysicsBody(this, id.GetIndexAndSequenceNumber());
}

PhysicsBody PhysicsWorld::addSphere(const Vec3& position, float radius, bool dynamic) {
    if (!impl_->initialized) return PhysicsBody();

    JPH::SphereShapeSettings shapeSettings(std::max(0.001f, radius));
    JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) {
        logError() << "tcxPhysics: sphere shape error: " << result.GetError().c_str();
        return PhysicsBody();
    }

    JPH::BodyCreationSettings bcs(
        result.Get(), toJolt(position), JPH::Quat::sIdentity(),
        dynamic ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static,
        dynamic ? Layers::MOVING : Layers::NON_MOVING);

    JPH::BodyID id = impl_->bodies().CreateAndAddBody(
        bcs, dynamic ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
    if (id.IsInvalid()) return PhysicsBody();
    if (dynamic) impl_->dynamicBodies.push_back(id);
    return PhysicsBody(this, id.GetIndexAndSequenceNumber());
}

PhysicsBody PhysicsWorld::addGroundPlane(float y, float size) {
    PhysicsBody ground = addBox(Vec3(0, y - 5.0f, 0), Vec3(size, 10.0f, size), false);
    if (impl_->initialized) impl_->physicsSystem.OptimizeBroadPhase();
    return ground;
}

void PhysicsWorld::removeBody(const PhysicsBody& body) {
    if (!impl_->initialized || !body.isValid()) return;
    JPH::BodyID id(body.getId());
    impl_->bodies().RemoveBody(id);
    impl_->bodies().DestroyBody(id);
    auto& v = impl_->dynamicBodies;
    v.erase(std::remove(v.begin(), v.end(), id), v.end());
}

void PhysicsWorld::clearDynamicBodies() {
    if (!impl_->initialized) return;
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
    return toTc(impl_->bodies().GetPosition(JPH::BodyID(id)));
}

Quaternion PhysicsWorld::getBodyRotation(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return Quaternion::identity();
    return toTc(impl_->bodies().GetRotation(JPH::BodyID(id)));
}

Vec3 PhysicsWorld::getBodySize(uint32_t id) const {
    if (!impl_->initialized || id == PhysicsBody::kInvalidId) return Vec3();
    JPH::BodyID bid(id);
    JPH::RefConst<JPH::Shape> shape = impl_->bodies().GetShape(bid);
    if (shape == nullptr) return Vec3();
    JPH::AABox bounds = shape->GetLocalBounds();
    return toTc(bounds.GetSize());
}

} // namespace tcx
