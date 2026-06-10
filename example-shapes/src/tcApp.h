#pragma once

#include <TrussC.h>
#include <tcxPhysics.h>

using namespace std;
using namespace tc;
using namespace tcx;

// shapes - every shape tcxPhysics wraps, colliding as its REAL silhouette.
//
// A static triangle-mesh "terrain" (addMesh) catches a rain of boxes, spheres,
// capsules, cylinders and convex hulls. Each body is drawn with the mesh that
// matches its collider — capsules roll on their side, cylinders settle flat, the
// hull tumbles on its facets — so you can see the collision shape, not just an AABB.
//
//   click : drop a fresh batch
//   R     : clear the dropped bodies (terrain stays)
//   drag  : orbit camera
class tcApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;

    void mousePressed(Vec2 pos, int button) override;
    void keyPressed(int key) override;

private:
    void spawnBatch(int n);

    EasyCam cam;
    PhysicsWorld world;
    Light keyLight;
    Light fillLight;

    // Shared draw meshes (one per shape kind; bodies use fixed dimensions so a
    // single mesh per kind is enough).
    Mesh boxMesh;       // unit cube, scaled by getSize() at draw
    Mesh sphereMesh;    // unit sphere, scaled by getSize()*0.5
    Mesh capsuleMesh;   // pre-sized — drawn as-is
    Mesh cylinderMesh;  // pre-sized
    Mesh hullMesh;      // icosphere; also the hull's source points
    Mesh terrainMesh;
    Material terrainMat;

    enum Kind { BOX, SPHERE, CAPSULE, CYLINDER, HULL, KIND_COUNT };
    struct Obj { PhysicsBody body; int kind; Material mat; };
    std::vector<Obj> objs;

    float lastTime = 0.0f;
};
