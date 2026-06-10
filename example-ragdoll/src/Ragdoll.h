#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace tc;
using namespace tcx;

// Ragdoll - a small humanoid built from RigidBody mods and ragdoll joints.
//
// The anatomy of the wiring:
//   neck            : Joint::swingTwist - small swing cone + small twist
//   shoulders, hips : Joint::swingTwist - wide swing cone + limited twist
//   elbows, knees   : Joint::hinge with one-sided angle limits
//
// build() spawns the parts into `parent` (a plain Node each, with RigidBody +
// ColliderRenderer added from here so the bodies exist immediately), then wires
// the joints. All parts are tracked in parts_ so the ragdoll can be tossed
// (kick()) or destroyed (clear() — which also removes its joints automatically).
class Ragdoll {
public:
    void build(Node& parent, const Vec3& at, const Color& tint) {
        clear();

        // --- parts (capsules hang along Y, so build in a standing pose) -----
        torso_ = spawn(parent, ColliderShape::box(Vec3(0.34f, 0.46f, 0.18f)), tint,
                       at + Vec3(0.0f, 1.50f, 0.0f));
        Node* head = spawn(parent, ColliderShape::sphere(0.13f), tint,
                           at + Vec3(0.0f, 1.92f, 0.0f));

        Node* upperArmL = spawn(parent, ColliderShape::capsule(0.055f, 0.16f), tint, at + Vec3(-0.25f, 1.52f, 0.0f));
        Node* upperArmR = spawn(parent, ColliderShape::capsule(0.055f, 0.16f), tint, at + Vec3( 0.25f, 1.52f, 0.0f));
        Node* lowerArmL = spawn(parent, ColliderShape::capsule(0.050f, 0.16f), tint, at + Vec3(-0.25f, 1.20f, 0.0f));
        Node* lowerArmR = spawn(parent, ColliderShape::capsule(0.050f, 0.16f), tint, at + Vec3( 0.25f, 1.20f, 0.0f));

        Node* upperLegL = spawn(parent, ColliderShape::capsule(0.07f, 0.20f), tint, at + Vec3(-0.10f, 1.06f, 0.0f));
        Node* upperLegR = spawn(parent, ColliderShape::capsule(0.07f, 0.20f), tint, at + Vec3( 0.10f, 1.06f, 0.0f));
        Node* lowerLegL = spawn(parent, ColliderShape::capsule(0.06f, 0.20f), tint, at + Vec3(-0.10f, 0.70f, 0.0f));
        Node* lowerLegR = spawn(parent, ColliderShape::capsule(0.06f, 0.20f), tint, at + Vec3( 0.10f, 0.70f, 0.0f));

        // --- joints (the part doing the moving is the one calling jointTo) --
        auto rb = [](Node* n) { return n->getMod<RigidBody>(); };
        const Vec3 Y(0.0f, 1.0f, 0.0f), X(1.0f, 0.0f, 0.0f);

        // Neck: a stiff-ish swing-twist.
        rb(head)->jointTo(torso_, Joint::swingTwist(at + Vec3(0.0f, 1.76f, 0.0f), Y)
                                      .swing(TAU * 30.0f / 360.0f)
                                      .twist(-TAU * 40.0f / 360.0f, TAU * 40.0f / 360.0f));
        // Shoulders: wide swing, limited twist.
        rb(upperArmL)->jointTo(torso_, Joint::swingTwist(at + Vec3(-0.25f, 1.66f, 0.0f), Y)
                                           .swing(TAU * 70.0f / 360.0f)
                                           .twist(-TAU * 30.0f / 360.0f, TAU * 30.0f / 360.0f));
        rb(upperArmR)->jointTo(torso_, Joint::swingTwist(at + Vec3( 0.25f, 1.66f, 0.0f), Y)
                                           .swing(TAU * 70.0f / 360.0f)
                                           .twist(-TAU * 30.0f / 360.0f, TAU * 30.0f / 360.0f));
        // Elbows: one-way hinges.
        rb(lowerArmL)->jointTo(upperArmL, Joint::hinge(at + Vec3(-0.25f, 1.36f, 0.0f), X)
                                              .limits(0.0f, TAU * 140.0f / 360.0f));
        rb(lowerArmR)->jointTo(upperArmR, Joint::hinge(at + Vec3( 0.25f, 1.36f, 0.0f), X)
                                              .limits(0.0f, TAU * 140.0f / 360.0f));
        // Hips: like shoulders but tighter.
        rb(upperLegL)->jointTo(torso_, Joint::swingTwist(at + Vec3(-0.10f, 1.24f, 0.0f), Y)
                                           .swing(TAU * 50.0f / 360.0f)
                                           .twist(-TAU * 20.0f / 360.0f, TAU * 20.0f / 360.0f));
        rb(upperLegR)->jointTo(torso_, Joint::swingTwist(at + Vec3( 0.10f, 1.24f, 0.0f), Y)
                                           .swing(TAU * 50.0f / 360.0f)
                                           .twist(-TAU * 20.0f / 360.0f, TAU * 20.0f / 360.0f));
        // Knees: one-way hinges (bend backward only).
        rb(lowerLegL)->jointTo(upperLegL, Joint::hinge(at + Vec3(-0.10f, 0.88f, 0.0f), X)
                                              .limits(-TAU * 140.0f / 360.0f, 0.0f));
        rb(lowerLegR)->jointTo(upperLegR, Joint::hinge(at + Vec3( 0.10f, 0.88f, 0.0f), X)
                                              .limits(-TAU * 140.0f / 360.0f, 0.0f));
    }

    // Mass-independent toss on the torso (the limbs follow through the joints).
    void kick(const Vec3& dv) {
        if (torso_) torso_->getMod<RigidBody>()->body().addVelocity(dv);
    }

    void clear() {
        for (Node* p : parts_) p->destroy();   // joints auto-removed with the bodies
        parts_.clear();
        torso_ = nullptr;
    }

    bool empty() const { return parts_.empty(); }

private:
    Node* spawn(Node& parent, const ColliderShape& shape, const Color& color, const Vec3& pos) {
        auto n = std::make_shared<Node>();
        n->setPos(pos);
        parent.addChild(n);
        n->addMod<RigidBody>(shape)->setFriction(0.6f);
        n->addMod<ColliderRenderer>()->setColor(color);
        parts_.push_back(n.get());
        return n.get();
    }

    std::vector<Node*> parts_;
    Node* torso_ = nullptr;
};
