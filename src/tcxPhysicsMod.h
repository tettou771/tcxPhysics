#pragma once

// =============================================================================
// tcxPhysicsMod.h - physics as Node Mods (EXPERIMENTAL).
//
//   auto n = make_shared<Node>();
//   n->setPos(0, 3, 0);
//   addChild(n);
//   n->addMod<RigidBody>(world, ColliderShape::box({0.3f, 0.3f, 0.3f}));
//   n->addMod<ColliderRenderer>()->setColor(Color(1, 0.4f, 0.1f));
//   // → the node now falls under physics AND draws its shape; no custom Node.
//
// RigidBody owns ONE Jolt body (shape + body type + density + PHYSICS material:
// friction/restitution). It is deliberately not split into "collider" + "rigid
// body": Jolt bundles them in one Body, and `density` ties the shape to the mass.
// Gravity and stepping live on the PhysicsWorld.
//
// ColliderRenderer is a SEPARATE draw Mod: it reads the sibling RigidBody's
// shape, builds a matching mesh, and draws it with a RENDER material (baseColor /
// texture / metallic / roughness). Physics material vs. render material stay apart.
//
// EXPERIMENTAL: rides the core Node/Mod API; expect churn while that settles.
// Assumes the owner node is in world space (top-level / identity-transform parents).
// =============================================================================

#include "tcxPhysicsWorld.h"
#include "tcxPhysicsBody.h"
#include <TrussC.h>   // tc::Mod, tc::Node, createBox/Sphere/Capsule/Cylinder, Material

namespace tcx {

// What the body is made of.
struct ColliderShape {
    enum Kind { Box, Sphere, Capsule, Cylinder };
    Kind kind = Box;
    tc::Vec3 size{1.0f, 1.0f, 1.0f};   // box: full extents
    float radius = 0.5f;               // sphere / capsule / cylinder
    float height = 1.0f;               // capsule cylinderHeight / cylinder height

    static ColliderShape box(const tc::Vec3& s)     { ColliderShape c; c.kind = Box;      c.size = s;   return c; }
    static ColliderShape sphere(float r)            { ColliderShape c; c.kind = Sphere;   c.radius = r; return c; }
    static ColliderShape capsule(float r, float h)  { ColliderShape c; c.kind = Capsule;  c.radius = r; c.height = h; return c; }
    static ColliderShape cylinder(float r, float h) { ColliderShape c; c.kind = Cylinder; c.radius = r; c.height = h; return c; }
};

// Dynamic = simulated (physics drives the node). Static = a fixed obstacle placed
// at the node's transform when attached.
enum class BodyType { Dynamic, Static };

// Process-wide default world for the terse `addMod<RigidBody>(shape)` form. You
// still setup() and step it yourself (defaultWorld().update(dt) each frame).
inline PhysicsWorld& defaultWorld() {
    static PhysicsWorld w;
    return w;
}

// Build a render mesh matching a collider shape (used by the wireframe debug draw
// and by ColliderRenderer).
inline tc::Mesh buildShapeMesh(const ColliderShape& s) {
    switch (s.kind) {
        case ColliderShape::Box:      return tc::createBox(s.size.x, s.size.y, s.size.z);
        case ColliderShape::Sphere:   return tc::createSphere(s.radius, 20);
        case ColliderShape::Capsule:  return tc::createCapsule(s.radius, s.height, 16);
        case ColliderShape::Cylinder: return tc::createCylinder(s.radius, s.height, 24);
    }
    return tc::Mesh();
}

// =============================================================================
// RigidBody : the physics Mod (one Jolt body).
// =============================================================================
class RigidBody : public tc::Mod {
    // addMod() calls setup() through a RigidBody* (not Mod*), so Node needs access
    // to our protected override. (Ideally core's addMod would dispatch via Mod*.)
    friend class trussc::Node;

public:
    RigidBody(PhysicsWorld& world, const ColliderShape& shape,
              BodyType type = BodyType::Dynamic, float density = 1000.0f)
        : world_(&world), shape_(shape), type_(type), density_(density) {}

    RigidBody(const ColliderShape& shape,
              BodyType type = BodyType::Dynamic, float density = 1000.0f)
        : world_(&defaultWorld()), shape_(shape), type_(type), density_(density) {}

    const PhysicsBody& body() const { return body_; }
    const ColliderShape& shape() const { return shape_; }

    // PHYSICS material (chainable; applied immediately if the body exists).
    RigidBody& setFriction(float f)    { friction_ = f;    if (body_.isValid()) body_.setFriction(f);    return *this; }
    RigidBody& setRestitution(float r) { restitution_ = r; if (body_.isValid()) body_.setRestitution(r); return *this; }

    // Built-in wireframe debug draw of the collision shape.
    RigidBody& setWireframe(bool on, const tc::Color& color = tc::Color(0.3f, 1.0f, 0.5f)) {
        wireframe_ = on; wireColor_ = color; return *this;
    }

protected:
    void setup() override {
        tc::Node* n = getOwner();
        worldAlive_ = world_->aliveToken();
        // Physics is WORLD-space — create the body at the node's global transform.
        tc::Vec3 wpos = n->getGlobalPos();
        bool dyn = (type_ == BodyType::Dynamic);
        switch (shape_.kind) {
            case ColliderShape::Box:      body_ = world_->addBox(wpos, shape_.size, dyn, density_); break;
            case ColliderShape::Sphere:   body_ = world_->addSphere(wpos, shape_.radius, dyn, density_); break;
            case ColliderShape::Capsule:  body_ = world_->addCapsule(wpos, shape_.radius, shape_.height, dyn, density_); break;
            case ColliderShape::Cylinder: body_ = world_->addCylinder(wpos, shape_.radius, shape_.height, dyn, density_); break;
        }
        if (body_.isValid()) {
            body_.setRotation(globalQuat(n));
            if (friction_ >= 0.0f)    body_.setFriction(friction_);
            if (restitution_ >= 0.0f) body_.setRestitution(restitution_);
        }
    }

    // Dynamic: physics drives the node. This runs in the Mod dispatch, so a user's
    // Node::update()/draw() override (even without calling super) can never skip it.
    void earlyUpdate() override {
        if (type_ != BodyType::Dynamic || !body_.isValid()) return;
        tc::Node* n = getOwner();
        if (n->isDead()) return;
        // Body transform is WORLD-space; the node stores LOCAL, so convert through
        // the parent (handles nesting; a top-level/identity parent is a no-op).
        tc::Vec3 wpos = body_.getPosition();
        auto parent = n->getParent();
        n->setPos(parent ? parent->globalToLocal(wpos) : wpos);
        n->setQuaternion(parentGlobalQuat(n).conjugate() * body_.getRotation());
    }

    void draw() override {
        if (!wireframe_ || !body_.isValid()) return;
        if (wireMesh_.getNumVertices() == 0) wireMesh_ = buildShapeMesh(shape_);
        tc::setColor(wireColor_);
        wireMesh_.drawWireframe();
    }

    void onDestroy() override {
        // Only touch the world if it's still alive — at shutdown it may be
        // destroyed before its bodies' nodes (it frees all bodies itself).
        if (!worldAlive_.expired() && world_ && body_.isValid()) world_->removeBody(body_);
    }

    bool isExclusive() const override { return true; }

private:
    // Core has no getGlobalQuaternion(), so walk the parent chain.
    static tc::Quaternion parentGlobalQuat(tc::Node* n) {
        tc::Quaternion q = tc::Quaternion::identity();
        for (auto p = n->getParent(); p; p = p->getParent()) q = p->getQuaternion() * q;
        return q;
    }
    static tc::Quaternion globalQuat(tc::Node* n) {
        return parentGlobalQuat(n) * n->getQuaternion();
    }

    PhysicsWorld* world_ = nullptr;
    ColliderShape shape_;
    BodyType type_ = BodyType::Dynamic;
    float density_ = 1000.0f;
    float friction_ = -1.0f;
    float restitution_ = -1.0f;
    PhysicsBody body_;
    std::weak_ptr<int> worldAlive_;
    bool wireframe_ = false;
    tc::Color wireColor_{0.3f, 1.0f, 0.5f};
    tc::Mesh wireMesh_;
};

// =============================================================================
// ColliderRenderer : draw the sibling RigidBody's shape as solid faces.
// =============================================================================
class ColliderRenderer : public tc::Mod {
    friend class trussc::Node;

public:
    // RENDER material (separate from RigidBody's physics material).
    ColliderRenderer& setColor(const tc::Color& c)       { material_.setBaseColor(c); return *this; }
    ColliderRenderer& setMaterial(const tc::Material& m) { material_ = m; return *this; }
    ColliderRenderer& setTexture(const tc::Texture& t)   { material_.setBaseColorTexture(&t); return *this; }
    ColliderRenderer& clearTexture()                     { material_.setBaseColorTexture(nullptr); return *this; }

protected:
    void draw() override {
        // Lazily build the mesh from the sibling RigidBody (order-independent).
        if (mesh_.getNumVertices() == 0) {
            tc::Node* n = getOwner();
            if (!n->hasMod<RigidBody>()) return;
            mesh_ = buildShapeMesh(n->getMod<RigidBody>()->shape());
            if (mesh_.getNumVertices() == 0) return;
        }
        tc::setMaterial(material_);
        mesh_.draw();
        tc::clearMaterial();
    }

    bool isExclusive() const override { return true; }

private:
    tc::Mesh mesh_;
    tc::Material material_;
};

} // namespace tcx
