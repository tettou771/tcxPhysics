#pragma once

// =============================================================================
// tcxPhysics - 3D rigid body physics for TrussC, powered by Jolt Physics.
//
// Umbrella header — include this and you get the whole addon:
//
//   #include <tcxPhysics.h>
//   using namespace tcx;
//
//   PhysicsWorld world;
//   world.setup();                                  // init the simulation
//   world.addGroundPlane(0.0f);                     // a static floor at y = 0
//   PhysicsBody b = world.addBox({0, 200, 0}, {10, 10, 10});  // a falling cube
//   ...
//   world.update(dt);                               // step once per frame
//   b.getPosition(); b.getRotation();               // read back the transform
//
// Jolt is fully hidden behind a PIMPL: nothing here pulls in a Jolt header, so
// the consuming app needs no Jolt include paths or compile defines.
// =============================================================================

#include "tcxPhysicsBody.h"
#include "tcxPhysicsJoint.h"
#include "tcxPhysicsWorld.h"
#include "tcxPhysicsMod.h"   // RigidBody / ColliderRenderer Node Mods (experimental)
