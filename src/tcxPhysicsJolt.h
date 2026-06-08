#pragma once

// =============================================================================
// tcxPhysicsJolt.h - advanced escape hatch (OPT-IN, not pulled in by tcxPhysics.h)
//
// tcxPhysics wraps the common cases ergonomically AND thread-safely. Reach for
// this header only for things the wrapper doesn't surface yet — constraints /
// joints, ray & shape casts, custom shapes, per-body mass/damping, etc. It hands
// you the raw Jolt objects so you can do anything Jolt can.
//
//   #include <tcxPhysics.h>
//   #include <tcxPhysicsJolt.h>      // opt in to raw Jolt
//   ...
//   JPH::BodyInterface& bi = joltBodyInterface(world);
//   JPH::BodyID id = joltBodyId(myBody);
//   bi.SetLinearDamping(id, 0.2f);   // a feature not (yet) wrapped
//
// TWO things to know before you use it:
//
//  1. BUILD: including this pulls in Jolt headers, so your app must have Jolt on
//     its include path and be compiled with the SAME JPH_* defines the addon was
//     built with (default: single precision, no JPH_DOUBLE_PRECISION). Mismatched
//     defines change struct layouts → silent ABI breakage. The simplest safe
//     route is to let the addon's CMake propagate Jolt to your target; see the
//     "Advanced / escape hatch" section of the README.
//
//  2. THREADING: these calls go straight to Jolt and BYPASS tcxPhysics's step
//     lock. With synchronous update() that's fine (you're on the main thread
//     between steps). In async mode, call world.updateAsyncStop() first (or take
//     your own lock around the step) — otherwise you race the worker thread.
// =============================================================================

#include "tcxPhysicsWorld.h"
#include "tcxPhysicsBody.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyID.h>

namespace tcx {

// The whole Jolt simulation. Use it to add constraints, run queries
// (NarrowPhaseQuery / BroadPhaseQuery), tweak PhysicsSettings, etc.
inline JPH::PhysicsSystem& joltSystem(PhysicsWorld& world) {
    return *static_cast<JPH::PhysicsSystem*>(world.nativeSystem());
}

// The body interface — create/modify/query bodies directly.
inline JPH::BodyInterface& joltBodyInterface(PhysicsWorld& world) {
    return *static_cast<JPH::BodyInterface*>(world.nativeBodyInterface());
}

// The typed Jolt id for a wrapped body (reconstructed from its packed handle).
inline JPH::BodyID joltBodyId(const PhysicsBody& body) {
    return JPH::BodyID(body.getId());
}

} // namespace tcx
