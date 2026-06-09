#include "tcApp.h"

// Fixed per-kind dimensions (metres) so one shared mesh per kind suffices.
static constexpr float BOX_SIZE   = 0.24f;
static constexpr float SPH_R      = 0.14f;
static constexpr float CAP_R      = 0.10f, CAP_H = 0.22f;   // radius, cylinder height
static constexpr float CYL_R      = 0.13f, CYL_H = 0.26f;
static constexpr float HULL_R     = 0.16f;
static constexpr int   MAX_OBJS   = 160;

// Build a gently bumpy ground as a triangle mesh, with analytic normals.
static Mesh buildTerrain(float halfExtent, int cells, float bumpAmp, float bumpFreq) {
    Mesh m;
    m.setMode(PrimitiveMode::Triangles);
    auto height = [&](float x, float z) { return bumpAmp * std::sin(x * bumpFreq) * std::cos(z * bumpFreq); };
    for (int j = 0; j <= cells; j++) {
        for (int i = 0; i <= cells; i++) {
            float x = -halfExtent + 2.0f * halfExtent * i / cells;
            float z = -halfExtent + 2.0f * halfExtent * j / cells;
            float y = height(x, z);
            // Analytic gradient of A*sin(kx)*cos(kz).
            float dx =  bumpAmp * bumpFreq * std::cos(x * bumpFreq) * std::cos(z * bumpFreq);
            float dz = -bumpAmp * bumpFreq * std::sin(x * bumpFreq) * std::sin(z * bumpFreq);
            Vec3 n = Vec3(-dx, 1.0f, -dz).normalized();
            m.addVertex(x, y, z);
            m.addNormal(n.x, n.y, n.z);
            m.addTexCoord((float)i / cells, (float)j / cells);
        }
    }
    int stride = cells + 1;
    for (int j = 0; j < cells; j++) {
        for (int i = 0; i < cells; i++) {
            int a = j * stride + i, b = a + 1, c = a + stride, d = c + 1;
            m.addTriangle(a, c, b);
            m.addTriangle(b, c, d);
        }
    }
    return m;
}

void tcApp::setup() {
    setWindowTitle("tcxPhysics - shapes  (box / sphere / capsule / cylinder / hull on a mesh)");

    cam.setTarget(0.0f, 0.4f, 0.0f);
    cam.setDistance(6.0f);
    cam.setAzimuth(0.6f);
    cam.setElevation(0.35f);
    cam.enableMouseInput();

    boxMesh      = createBox(1.0f);
    sphereMesh   = createSphere(1.0f, 20);
    capsuleMesh  = createCapsule(CAP_R, CAP_H, 20);
    cylinderMesh = createCylinder(CYL_R, CYL_H, 24);
    hullMesh     = createIcoSphere(HULL_R, 1);          // faceted ball = nice hull
    terrainMesh  = buildTerrain(2.2f, 24, 0.18f, 2.2f);

    terrainMat.setBaseColor(0.16f, 0.18f, 0.22f).setMetallic(0.0f).setRoughness(0.9f);

    keyLight.setDirectional(Vec3(-0.4f, -1.0f, -0.6f));
    keyLight.setDiffuse(1.0f, 0.95f, 0.88f);
    keyLight.setIntensity(2.6f);
    addLight(keyLight);

    fillLight.setDirectional(Vec3(0.6f, -0.3f, 0.5f));
    fillLight.setDiffuse(0.6f, 0.6f, 0.65f);
    fillLight.setIntensity(1.1f);
    addLight(fillLight);

    world.setup(MAX_OBJS + 16);                 // default gravity -9.81
    world.addMesh(Vec3(0.0f, 0.0f, 0.0f), terrainMesh);   // static terrain collider

    spawnBatch(20);
    lastTime = getElapsedTimef();
}

void tcApp::spawnBatch(int n) {
    if ((int)objs.size() + n > MAX_OBJS) {
        world.clearDynamicBodies();
        objs.clear();
    }
    for (int i = 0; i < n; i++) {
        int kind = (int)random(0.0f, (float)KIND_COUNT);
        if (kind >= KIND_COUNT) kind = KIND_COUNT - 1;
        Vec3 p(random(-1.6f, 1.6f), random(2.4f, 3.4f), random(-1.6f, 1.6f));

        Obj o;
        o.kind = kind;
        switch (kind) {
            case BOX:      o.body = world.addBox(p, Vec3(BOX_SIZE)); break;
            case SPHERE:   o.body = world.addSphere(p, SPH_R); break;
            case CAPSULE:  o.body = world.addCapsule(p, CAP_R, CAP_H); break;
            case CYLINDER: o.body = world.addCylinder(p, CYL_R, CYL_H); break;
            case HULL:     o.body = world.addConvexHull(p, hullMesh); break;
        }
        o.body.setRestitution(0.2f);
        o.mat.setBaseColor(random(0.15f, 0.95f), random(0.15f, 0.95f), random(0.15f, 0.95f))
             .setMetallic(0.0f)
             .setRoughness(random(0.3f, 0.7f));
        objs.push_back(o);
    }
}

void tcApp::update() {
    float t = getElapsedTimef();
    float dt = std::min(0.05f, std::max(0.0f, t - lastTime));
    lastTime = t;
    world.update(dt);
}

void tcApp::draw() {
    clear(0.08f, 0.09f, 0.11f);

    cam.begin();
    setCameraPosition(cam.getPosition());

    // Static terrain (drawn from the same mesh fed to addMesh).
    setMaterial(terrainMat);
    terrainMesh.draw();
    clearMaterial();

    for (Obj& o : objs) {
        if (!o.body.isValid()) continue;
        setMaterial(o.mat);
        pushMatrix();
        translate(o.body.getPosition());
        rotate(o.body.getRotation());
        switch (o.kind) {
            case BOX: {
                Vec3 s = o.body.getSize();
                scale(s.x, s.y, s.z);
                boxMesh.draw();
            } break;
            case SPHERE: {
                Vec3 s = o.body.getSize();
                scale(s.x * 0.5f, s.y * 0.5f, s.z * 0.5f);
                sphereMesh.draw();
            } break;
            case CAPSULE:  capsuleMesh.draw();  break;   // pre-sized meshes
            case CYLINDER: cylinderMesh.draw(); break;
            case HULL:     hullMesh.draw();     break;
        }
        popMatrix();
    }
    clearMaterial();

    cam.end();

    setColor(1.0f);
    drawBitmapString(
        "bodies: " + std::to_string((int)objs.size()) + "\n" +
        "fps:    " + std::to_string((int)(getFrameRate() + 0.5)) + "\n" +
        "\n" +
        "box / sphere / capsule / cylinder / convex-hull\n" +
        "on a static triangle-mesh terrain\n" +
        "\n" +
        "click: drop a batch   R: clear\n" +
        "drag:  orbit camera",
        20.0f, 20.0f);
}

void tcApp::mousePressed(Vec2 pos, int button) {
    (void)pos; (void)button;
    spawnBatch(15);
}

void tcApp::keyPressed(int key) {
    if (key == 'R') {
        world.clearDynamicBodies();
        objs.clear();
    }
}
