#include "headers.h"

void step_bodies();
void broad_phase();
void narrow_phase();

constexpr uint16_t TOTAL_ITER = 512;
float DT = 0.01f;
bool IS_PAUSED = false;

std::vector<std::vector<size_t>> COLLISION_PAIRS;
std::vector<Collisions::CollisionResult> COLLISION_RESULTS;
std::vector<Rigidbody> BODIES;

void step()
{
    for (size_t iter = 0; iter < TOTAL_ITER; ++iter)
    {
        COLLISION_PAIRS.clear();
        COLLISION_RESULTS.clear();

        step_bodies();
        broad_phase();
        narrow_phase();
    }
}

void step_bodies()
{
    if (IS_PAUSED)
        return;

    for (auto &body : BODIES)
    {
        if (body.m_is_static)
            continue;
        body.update(DT / static_cast<float>(TOTAL_ITER));
    }
}

void broad_phase()
{
    for (size_t i = 0; i < BODIES.size() - 1; ++i)
    {
        Rigidbody &rb_a = BODIES[i];

        for (size_t j = i + 1; j < BODIES.size(); ++j)
        {
            Rigidbody &rb_b = BODIES[j];

            if (rb_a.m_is_static && rb_b.m_is_static)
                continue;
            if (!Collisions::intersect_aabb(rb_a, rb_b))
                continue;

            COLLISION_PAIRS.push_back({i, j});
        }
    }
}

void narrow_phase()
{
    for (const auto &pair : COLLISION_PAIRS)
    {
        Rigidbody &rb_a = BODIES[pair[0]];
        Rigidbody &rb_b = BODIES[pair[1]];

        rb_a.transform_verts();
        rb_b.transform_verts();

        Collisions::CollisionResult result = Collisions::collide(rb_a, rb_b);

        if (result.m_is_colliding)
        {
            Vec2 mtv = result.m_normal * result.m_depth;
            CollisionResolution::separate_bodies(rb_a, rb_b, mtv);

            Collisions::find_collision_points(rb_a, rb_b, result);

            CollisionResolution::CollisionManifold manifold(&rb_a, &rb_b, result);

            CollisionResolution::resolve_collision_with_friction(manifold);

            COLLISION_RESULTS.push_back(result);
        }
    }
}
