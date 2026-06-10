#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace tc;
using namespace tcx;

// Slider - a KINEMATIC platform that slides back and forth.
//
// It is a RigidBody of BodyType::Kinematic: you drive the NODE (here in update())
// and the body follows, shoving any dynamic bodies it meets — but it ignores
// gravity and impacts itself. The RigidBody mod reads the node transform AFTER
// this update() and feeds it to Jolt as a kinematic move (deriving the velocity).
class Slider : public Node {
public:
    Slider(const Vec3& size, float amplitude, float speed, float y)
        : size_(size), amp_(amplitude), speed_(speed), y_(y) {}

    void setup() override {
        addMod<RigidBody>(ColliderShape::box(size_), BodyType::Kinematic)->setFriction(0.9f);
        addMod<ColliderRenderer>()->setColor(Color(0.85f, 0.55f, 0.15f));
    }

    void update() override {
        float t = getElapsedTimef();
        setPos(std::sin(t * speed_) * amp_, y_, 0.0f);
    }

private:
    Vec3  size_;
    float amp_, speed_, y_;
};

// Spinner - a KINEMATIC bar that rotates about Y, sweeping bodies like a paddle.
class Spinner : public Node {
public:
    Spinner(const Vec3& size, float speed, const Vec3& pos)
        : size_(size), speed_(speed) { setPos(pos); }

    void setup() override {
        addMod<RigidBody>(ColliderShape::box(size_), BodyType::Kinematic)->setFriction(0.6f);
        addMod<ColliderRenderer>()->setColor(Color(0.2f, 0.6f, 0.85f));
    }

    void update() override {
        setEuler(0.0f, getElapsedTimef() * speed_, 0.0f);
    }

private:
    Vec3  size_;
    float speed_;
};
