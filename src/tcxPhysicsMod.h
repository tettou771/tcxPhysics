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
#include <unordered_map>

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

// Dynamic   = simulated (physics drives the node).
// Static    = a fixed obstacle placed at the node's transform when attached.
// Kinematic = YOU drive the node (in setup/update); the body follows and pushes
//             dynamics with the right momentum, but ignores forces/gravity itself.
//             Great for moving platforms, paddles, doors.
enum class BodyType { Dynamic, Static, Kinematic };
TC_ENUM_LABELS(BodyType, "Dynamic", "Static", "Kinematic")

// Process-wide default world, so `addMod<RigidBody>(shape)` needs no world passed.
// Point it at your own instance with setDefaultWorld(); otherwise the first access
// lazily creates a built-in one. You still setup() and step it (the app usually
// does defaultWorld().setup() once and defaultWorld().update(dt) each frame).
inline PhysicsWorld*& currentDefaultWorld_() { static PhysicsWorld* p = nullptr; return p; }
inline void setDefaultWorld(PhysicsWorld& world) { currentDefaultWorld_() = &world; }
inline PhysicsWorld& defaultWorld() {
    PhysicsWorld*& p = currentDefaultWorld_();
    if (!p) { static PhysicsWorld builtin; p = &builtin; }
    return *p;
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

class RigidBody;  // fwd

// Argument for RigidBody::onCollisionBegan / onCollisionEnded, described from the
// receiving body's point of view. `other` is the body it touched, or null if that
// was a non-RigidBody collider (the ground plane, a raw addBox/addMesh body, ...).
struct Collision {
    RigidBody* other = nullptr;
    tc::Node*  otherNode = nullptr;   // other ? other->getOwner() : nullptr
    tc::Vec3   point;                 // world-space (zero on Ended)
    tc::Vec3   normal;                // world-space (zero on Ended)
    float      speed = 0.0f;          // approach speed at impact (zero on Ended)
};

namespace detail {
    // One router per world: maps Jolt body id -> the RigidBody that owns it, plus
    // the listeners on that world's contactBegan/Ended. The first RigidBody on a
    // world installs the listeners; everyone registers their body id here.
    struct ContactRouter {
        std::unordered_map<uint32_t, RigidBody*> bodies;
        tc::EventListener beganL, persistedL, endedL;
    };
    inline std::unordered_map<PhysicsWorld*, ContactRouter>& contactRouters() {
        static std::unordered_map<PhysicsWorld*, ContactRouter> r;
        return r;
    }
    // phase: 0 = began, 1 = stay, 2 = ended.
    void routeContact(PhysicsWorld* w, ContactEventArgs& c, int phase);  // after RigidBody
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

    // Make this body a trigger (sensor): it detects overlaps and fires onTrigger*,
    // but never blocks motion. Set it before or after attach. (chainable)
    RigidBody& setTrigger(bool on = true) { trigger_ = on; if (body_.isValid()) body_.setSensor(on); return *this; }
    bool isTrigger() const { return trigger_; }

    // Switch how the body moves at runtime (Dynamic <-> Static <-> Kinematic).
    // The sync direction follows: Dynamic = body drives node, Kinematic = node
    // drives body, Static = parked.
    RigidBody& setBodyType(BodyType t) {
        type_ = t;
        if (body_.isValid()) {
            body_.setMotionType(t == BodyType::Dynamic ? MotionType::Dynamic
                              : t == BodyType::Static  ? MotionType::Static
                                                       : MotionType::Kinematic);
        }
        return *this;
    }
    BodyType getBodyType() const { return type_; }

    // Collision filtering (see PhysicsBody::setCollisionLayer/Mask): one layer
    // 0..7 + a mask of layers this body collides with. Chainable; applied
    // immediately if the body exists, else at attach.
    RigidBody& setCollisionLayer(int layer) { layer_ = layer; if (body_.isValid()) body_.setCollisionLayer(layer); return *this; }
    int getCollisionLayer() const { return body_.isValid() ? body_.getCollisionLayer() : (layer_ < 0 ? 0 : layer_); }
    RigidBody& setCollisionMask(uint32_t mask) { mask_ = (int64_t)mask; if (body_.isValid()) body_.setCollisionMask(mask); return *this; }
    uint32_t getCollisionMask() const { return body_.isValid() ? body_.getCollisionMask() : (mask_ < 0 ? 0xffu : (uint32_t)mask_); }

    // Inspector-facing getters (live values when the body exists).
    float getFriction() const    { return body_.isValid() ? body_.getFriction()    : friction_; }
    float getRestitution() const { return body_.isValid() ? body_.getRestitution() : restitution_; }
    float getDensity() const     { return density_; }
    bool  isWireframe() const    { return wireframe_; }

    // --- joints --------------------------------------------------------------
    // Constrain this body to another node's RigidBody. The OTHER node is the
    // base; THIS body is the side that moves positively (motor +velocity, +limits):
    //
    //   door->getMod<RigidBody>()->jointTo(frame, Joint::hinge(edge, {0,1,0}));
    //   // "joint the door TO the frame" — the door swings.
    //
    // The world owns the joint; the returned PhysicsJoint is a lightweight
    // handle. Destroying either node removes the joint automatically.
    //
    // NOTE on timing: both bodies must exist. A RigidBody added inside a Node
    // Mouse picking (Mod::hitTest hook): ray vs the collider's bounding box
    // in local space. Any node with a RigidBody becomes selectable by node
    // picking / inspectors with no extra code; return-false = pass-through.
    bool hitTest(const tc::Ray& localRay, float& outDistance) override {
        tc::Vec3 half;
        switch (shape_.kind) {
            case ColliderShape::Box:      half = shape_.size * 0.5f; break;
            case ColliderShape::Sphere:   half = tc::Vec3(shape_.radius, shape_.radius, shape_.radius); break;
            case ColliderShape::Capsule:
            case ColliderShape::Cylinder:
                half = tc::Vec3(shape_.radius, shape_.height * 0.5f + shape_.radius, shape_.radius);
                break;
            default: return false;
        }
        float tmin = -1e30f, tmax = 1e30f;
        const float o[3] = {localRay.origin.x, localRay.origin.y, localRay.origin.z};
        const float d[3] = {localRay.direction.x, localRay.direction.y, localRay.direction.z};
        const float h[3] = {half.x, half.y, half.z};
        for (int i = 0; i < 3; i++) {
            if (std::fabs(d[i]) < 1e-8f) {
                if (o[i] < -h[i] || o[i] > h[i]) return false;
                continue;
            }
            float t1 = (-h[i] - o[i]) / d[i];
            float t2 = ( h[i] - o[i]) / d[i];
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            if (tmin > tmax) return false;
        }
        if (tmax < 0) return false;
        outDistance = tmin >= 0 ? tmin : tmax;
        return true;
    }

    // subclass's setup() is only created when that setup runs (the node's first
    // frame in the tree) — so wire joints AFTER both nodes are set up (e.g. the
    // frame after spawning), or add the mods directly from app code
    // (node->addMod<RigidBody>(...) creates the body immediately).
    PhysicsJoint jointTo(tc::Node* other, const Joint& def) {
        if (!body_.isValid()) {
            tc::logWarning() << "tcxPhysics: jointTo before this body exists (wire joints after setup).";
            return PhysicsJoint();
        }
        RigidBody* otherRb = other ? other->getMod<RigidBody>() : nullptr;
        if (!otherRb || !otherRb->body_.isValid()) {
            tc::logWarning() << "tcxPhysics: jointTo target has no live RigidBody.";
            return PhysicsJoint();
        }
        // other = base (body 1), this = the moving side (body 2) — Jolt measures
        // relative motion (and drives motors) as body 2 relative to body 1.
        return world_->addJoint(otherRb->body_, body_, def);
    }
    PhysicsJoint jointTo(const std::shared_ptr<tc::Node>& other, const Joint& def) {
        return jointTo(other.get(), def);
    }

    // Constrain this body to the WORLD (hang from the air, pin to a wall...).
    PhysicsJoint jointToWorld(const Joint& def) {
        if (!body_.isValid()) {
            tc::logWarning() << "tcxPhysics: jointToWorld before this body exists.";
            return PhysicsJoint();
        }
        return world_->addJoint(body_, def);
    }

    // Every live joint touching this body (queries the world; nothing cached).
    std::vector<PhysicsJoint> getJoints() const {
        if (!body_.isValid()) return {};
        return world_->getJointsForBody(body_.getId());
    }

    // Built-in wireframe debug draw of the collision shape.
    RigidBody& setWireframe(bool on, const tc::Color& color = tc::Color(0.3f, 1.0f, 0.5f)) {
        wireframe_ = on; wireColor_ = color; return *this;
    }

    // Collision events (listen via EventListener; fired on the main thread).
    //   listener = rb->onCollisionBegan.listen([](Collision& c){ ... });
    tc::Event<Collision> onCollisionBegan;   // started touching (Enter)
    tc::Event<Collision> onCollisionStay;    // still touching, every step (Stay)
    tc::Event<Collision> onCollisionEnded;   // stopped touching (Exit)

    // Trigger events — fired instead of the collision ones whenever EITHER side is
    // a trigger (a sensor: it detects overlaps but never blocks motion). Make this
    // body a trigger with setTrigger(true). `point`/`normal`/`speed` come from the
    // overlap manifold (zero on Exit), `other`/`otherNode` is who entered.
    tc::Event<Collision> onTriggerBegan;     // started overlapping (Enter)
    tc::Event<Collision> onTriggerStay;      // still overlapping, every step (Stay)
    tc::Event<Collision> onTriggerEnded;     // stopped overlapping (Exit)

    // Reflection: shows up in inspectors (e.g. tcxNodeInspector) with live,
    // editable physics state. Setters run, so edits hit the Jolt body.
    using Super = tc::Mod;
    TC_REFLECT(RigidBody)
        TC_ENUM_PROPERTY(bodyType, getBodyType, setBodyType)
        TC_PROPERTY_RO(density, getDensity)
        TC_PROPERTY(friction, getFriction, setFriction)
        TC_PROPERTY(restitution, getRestitution, setRestitution)
        TC_PROPERTY(trigger, isTrigger, setTrigger)
        TC_PROPERTY(collisionLayer, getCollisionLayer, setCollisionLayer)
        TC_PROPERTY(wireframe, isWireframe, setWireframe)
    TC_REFLECT_END

protected:
    void setup() override {
        tc::Node* n = getOwner();
        worldAlive_ = world_->aliveToken();
        // Physics is WORLD-space — create the body at the node's global transform.
        tc::Vec3 wpos = n->getGlobalPos();
        // Kinematic bodies must live in the moving layer (so they collide with and
        // push dynamics), so create them movable, then switch the motion type.
        bool movable = (type_ != BodyType::Static);
        switch (shape_.kind) {
            case ColliderShape::Box:      body_ = world_->addBox(wpos, shape_.size, movable, density_); break;
            case ColliderShape::Sphere:   body_ = world_->addSphere(wpos, shape_.radius, movable, density_); break;
            case ColliderShape::Capsule:  body_ = world_->addCapsule(wpos, shape_.radius, shape_.height, movable, density_); break;
            case ColliderShape::Cylinder: body_ = world_->addCylinder(wpos, shape_.radius, shape_.height, movable, density_); break;
        }
        if (body_.isValid()) {
            body_.setRotation(globalQuat(n));
            if (type_ == BodyType::Kinematic) body_.setMotionType(MotionType::Kinematic);
            if (trigger_)             body_.setSensor(true);
            if (friction_ >= 0.0f)    body_.setFriction(friction_);
            if (restitution_ >= 0.0f) body_.setRestitution(restitution_);
            if (layer_ >= 0)          body_.setCollisionLayer(layer_);
            if (mask_ >= 0)           body_.setCollisionMask((uint32_t)mask_);

            // Register for collision routing. The first body on this world hooks
            // the world's contact events.
            auto& router = detail::contactRouters()[world_];
            if (!router.beganL) {
                router.beganL = world_->contactBegan.listen(
                    [w = world_](ContactEventArgs& c) { detail::routeContact(w, c, 0); });
                router.persistedL = world_->contactPersisted.listen(
                    [w = world_](ContactEventArgs& c) { detail::routeContact(w, c, 1); });
                router.endedL = world_->contactEnded.listen(
                    [w = world_](ContactEventArgs& c) { detail::routeContact(w, c, 2); });
            }
            router.bodies[body_.getId()] = this;
        }
    }

    // Dynamic: physics drives the node — sync BEFORE Node::update() so user code
    // sees the fresh transform. This runs in the Mod dispatch, so a user's
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

    // Kinematic: the node drives the body — sync AFTER Node::update() so we pick up
    // wherever the user just moved the node this frame. moveKinematic derives the
    // velocity from the delta so the body shoves the dynamics it meets.
    void update() override {
        if (type_ != BodyType::Kinematic || !body_.isValid()) return;
        tc::Node* n = getOwner();
        if (n->isDead()) return;
        body_.moveKinematic(n->getGlobalPos(), globalQuat(n), (float)tc::getDeltaTime());
    }

    void draw() override {
        if (!wireframe_ || !body_.isValid()) return;
        if (wireMesh_.getNumVertices() == 0) wireMesh_ = buildShapeMesh(shape_);
        tc::setColor(wireColor_);
        wireMesh_.drawWireframe();
    }

    void onDestroy() override {
        // Unregister from contact routing (registry holds a raw this pointer).
        if (body_.isValid()) {
            auto it = detail::contactRouters().find(world_);
            if (it != detail::contactRouters().end()) it->second.bodies.erase(body_.getId());
        }
        // Only touch the world if it's still alive — at shutdown it may be
        // destroyed before its bodies' nodes (it frees all bodies itself).
        if (!worldAlive_.expired() && world_ && body_.isValid()) world_->removeBody(body_);
    }

    bool isExclusive() const override { return true; }

private:
    friend void detail::routeContact(PhysicsWorld* w, ContactEventArgs& c, int phase);

    // Build a Collision (from this body's view) and fire the matching event. When
    // the contact involves a sensor, the trigger events fire instead.
    void fireContact(RigidBody* other, const ContactEventArgs& c, int phase, bool trigger) {
        Collision col;
        col.other     = other;
        col.otherNode = other ? other->getOwner() : nullptr;
        col.point     = c.point;
        col.normal    = c.normal;
        col.speed     = c.speed;
        if (trigger) {
            switch (phase) {
                case 0:  onTriggerBegan.notify(col); break;
                case 1:  onTriggerStay.notify(col);  break;
                default: onTriggerEnded.notify(col); break;
            }
        } else {
            switch (phase) {
                case 0:  onCollisionBegan.notify(col); break;
                case 1:  onCollisionStay.notify(col);  break;
                default: onCollisionEnded.notify(col); break;
            }
        }
    }

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
    bool trigger_ = false;
    int layer_ = -1;        // pending collision layer (-1 = default)
    int64_t mask_ = -1;     // pending collision mask (-1 = default 0xff)
    PhysicsBody body_;
    std::weak_ptr<int> worldAlive_;
    bool wireframe_ = false;
    tc::Color wireColor_{0.3f, 1.0f, 0.5f};
    tc::Mesh wireMesh_;
};

namespace detail {
// Fan a world contact out to the RigidBody(s) involved (main thread). Each side
// hears about the OTHER body; a side that isn't a RigidBody resolves to null.
inline void routeContact(PhysicsWorld* w, ContactEventArgs& c, int phase) {
    auto it = contactRouters().find(w);
    if (it == contactRouters().end()) return;
    auto& bodies = it->second.bodies;
    auto fa = bodies.find(c.a.getId());
    auto fb = bodies.find(c.b.getId());
    RigidBody* ra = (fa != bodies.end()) ? fa->second : nullptr;
    RigidBody* rb = (fb != bodies.end()) ? fb->second : nullptr;
    // If either side is a trigger, the whole contact is a trigger overlap (Jolt
    // produced no collision response for it), so both sides get onTrigger*.
    bool trigger = (ra && ra->isTrigger()) || (rb && rb->isTrigger());
    if (ra) ra->fireContact(rb, c, phase, trigger);
    if (rb) rb->fireContact(ra, c, phase, trigger);
}
} // namespace detail

// =============================================================================
// ColliderRenderer : draw the sibling RigidBody's shape as solid faces.
// =============================================================================
class ColliderRenderer : public tc::Mod {
    friend class trussc::Node;

public:
    // RENDER material (separate from RigidBody's physics material).
    ColliderRenderer& setColor(const tc::Color& c)       { material_.setBaseColor(c); return *this; }
    tc::Color getColor() const                           { return material_.getBaseColor(); }
    ColliderRenderer& setMaterial(const tc::Material& m) { material_ = m; return *this; }
    ColliderRenderer& setTexture(const tc::Texture& t)   { material_.setBaseColorTexture(&t); return *this; }
    ColliderRenderer& clearTexture()                     { material_.setBaseColorTexture(nullptr); return *this; }

    using Super = tc::Mod;
    TC_REFLECT(ColliderRenderer)
        TC_PROPERTY(color, getColor, setColor)
    TC_REFLECT_END

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
