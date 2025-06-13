//
// Created by Lukas Campbell on 08/06/2025.
//
#include "headers.h"

using namespace emscripten;

struct Vec2Wrapper
{
    float x, y;

    Vec2Wrapper() : x(0.0f), y(0.0f) {}

    Vec2Wrapper(float x_, float y_) : x(x_), y(y_) {}
};

extern void step();

static int g_draggedBodyIndex = -1;
static Vec2 g_mousePos;


int start_mouse_drag(float x, float y)
{
    g_mousePos = Vec2(x, y);

    for (int i = 0; i < BODIES.size(); ++i)
    {
        Rigidbody &body = BODIES[i];

        if (!Collisions::intersect_aabb_with_point(body.m_aabb, g_mousePos))
            continue;

        bool hit = false;
        if (body.m_shape_type == ShapeType::CIRCLE)
        {
            Vec2 diff = g_mousePos - body.m_pos;
            hit = (diff.len() <= body.m_radius);
        }
        else
        {
            float cos_theta = std::cos(-body.m_angle);
            float sin_theta = std::sin(-body.m_angle);
            Vec2 local = g_mousePos - body.m_pos;
            Vec2 rotated(
                local.m_x * cos_theta - local.m_y * sin_theta,
                local.m_x * sin_theta + local.m_y * cos_theta);

            hit = (std::abs(rotated.m_x) <= body.m_width / 2.0f &&
                   std::abs(rotated.m_y) <= body.m_height / 2.0f);
        }

        if (hit)
        {
            g_draggedBodyIndex = i;

            return i;
        }
    }

    return -1;
}

void update_mouse_position(float x, float y)
{
    g_mousePos = Vec2(x, y);

}

void end_mouse_drag()
{
 

int get_body_at_position(float x, float y)
{
    Vec2 pos(x, y);

    for (int i = 0; i < BODIES.size(); ++i)
    {
        Rigidbody &body = BODIES[i];

        if (!Collisions::intersect_aabb_with_point(body.m_aabb, pos))
            continue;

        bool hit = false;
        if (body.m_shape_type == ShapeType::CIRCLE)
        {
            Vec2 diff = pos - body.m_pos;
            hit = (diff.len() <= body.m_radius);
        }
        else
        {
            float cos_theta = std::cos(-body.m_angle);
            float sin_theta = std::sin(-body.m_angle);
            Vec2 local = pos - body.m_pos;
            Vec2 rotated(
                local.m_x * cos_theta - local.m_y * sin_theta,
                local.m_x * sin_theta + local.m_y * cos_theta);

            hit = (std::abs(rotated.m_x) <= body.m_width / 2.0f &&
                   std::abs(rotated.m_y) <= body.m_height / 2.0f);
        }

        if (hit)
            return i;
    }

    return -1;
}

void clear_forces_on_body(int index)
{
    if (index < 0 || index >= BODIES.size())
        return;

    Rigidbody &body = BODIES[index];
    // for (auto &force : body.m_f_registry)
    // {
    //     // delete force;
    // }
    body.m_f_registry.clear();
}

void set_body_position(int index, float x, float y)
{
    if (index >= 0 && index < BODIES.size())
    {
        BODIES[index].m_pos = Vec2(x, y);
        BODIES[index].m_transform_update_req = true;
        BODIES[index].m_aabb_update_req = true;
    }
}

void set_body_velocity(int index, float vx, float vy)
{
    if (index >= 0 && index < BODIES.size())
    {
        BODIES[index].m_vel = Vec2(vx, vy);
    }
}

void set_body_angle(int index, float angle)
{
    if (index >= 0 && index < BODIES.size())
    {
        BODIES[index].m_angle = angle;
        BODIES[index].m_transform_update_req = true;
    }
}

void set_body_angular_velocity(int index, float omega)
{
    if (index >= 0 && index < BODIES.size())
    {
        BODIES[index].m_angular_vel = omega;
    }
}

void set_body_restitution(int index, float restitution)
{
    if (index >= 0 && index < BODIES.size())
    {
        BODIES[index].m_restitution = PhysicsMath::clamp(restitution, 0.0f, 1.0f);
    }
}

void set_body_friction(int index, float static_friction, float kinetic_friction)
{
    if (index >= 0 && index < BODIES.size())
    {
        BODIES[index].m_static_friction = static_friction;
        BODIES[index].m_kinetic_friction = kinetic_friction;
    }
}

float getBodyX(int index)
{
    if (index >= 0 && index < BODIES.size())
        return BODIES[index].m_pos.m_x;
    return 0.0f;
}

float getBodyY(int index)
{
    if (index >= 0 && index < BODIES.size())
        return BODIES[index].m_pos.m_y;
    return 0.0f;
}

float getBodyAngle(int index)
{
    if (index >= 0 && index < BODIES.size())
        return BODIES[index].m_angle;
    return 0.0f;
}

Vec2Wrapper get_body_pos(int index)
{
    if (index >= 0 && index < BODIES.size())
    {
        return Vec2Wrapper(BODIES[index].m_pos.m_x, BODIES[index].m_pos.m_y);
    }
    return Vec2Wrapper(0.0f, 0.0f);
}

float get_body_angle(int index)
{
    if (index >= 0 && index < BODIES.size())
        return BODIES[index].m_angle;
    return 0.0f;
}

Vec2Wrapper get_body_vel(int index)
{
    if (index >= 0 && index < BODIES.size())
    {
        return Vec2Wrapper(BODIES[index].m_vel.m_x, BODIES[index].m_vel.m_y);
    }
    return Vec2Wrapper(0.0f, 0.0f);
}

float get_body_angular_velocity(int index)
{
    if (index >= 0 && index < BODIES.size())
        return BODIES[index].m_angular_vel;
    return 0.0f;
}

int get_body_shape_type(int index)
{
    if (index >= 0 && index < BODIES.size())
        return static_cast<int>(BODIES[index].m_shape_type);
    return -1;
}

float get_body_radius(int index)
{
    if (index >= 0 && index < BODIES.size() && BODIES[index].m_shape_type == ShapeType::CIRCLE)
        return BODIES[index].m_radius;
    return 0.0f;
}

float get_body_width(int index)
{
    if (index >= 0 && index < BODIES.size() && BODIES[index].m_shape_type == ShapeType::BOX)
        return BODIES[index].m_width;
    return 0.0f;
}

float get_body_height(int index)
{
    if (index >= 0 && index < BODIES.size() && BODIES[index].m_shape_type == ShapeType::BOX)
        return BODIES[index].m_height;
    return 0.0f;
}

void remove_body(int index)
{
    if (index >= 0 && index < BODIES.size())
    {
        clear_forces_on_body(index);
        BODIES.erase(BODIES.begin() + index);
    }
}

void clear_bodies()
{
    for (int i = 0; i < BODIES.size(); ++i)
    {
        clear_forces_on_body(i);
    }
    BODIES.clear();
}

int get_body_count()
{
    return BODIES.size();
}

void set_dt(float dt)
{
    DT = dt;
}
float get_dt()
{
    return DT;
}



float getDT()
{
    return DT;
}

void set_paused(bool paused)
{
    IS_PAUSED = paused;
}

std::vector<Vec2Wrapper> get_transformed_vertices(int index)
{
    std::vector<Vec2Wrapper> result;
    if (index >= 0 && index < BODIES.size() && BODIES[index].m_shape_type == ShapeType::BOX)
    {
        BODIES[index].transform_verts();
        for (const auto &v : BODIES[index].m_transformed_verts)
        {
            result.emplace_back(v.m_x, v.m_y);
        }
    }
    return result;
}

void add_gravity(int index)
{
    if (index >= 0 && index < BODIES.size())
    {
        BODIES[index].m_f_registry.push_back(std::make_shared<Gravity>());
    }
}

void set_gravity(float gx, float gy)
{
}


emscripten::val get_wall_body_indices()
{
    emscripten::val result = emscripten::val::array();
    for (int i = 0; i < WALL_BODIES.size(); ++i)
    {
        const auto &body = WALL_BODIES[i];
        int index = static_cast<int>(body - &BODIES[0]);
        result.set(i, index);
    }
    return result;
}

void get_all_forces_for_visualization()
{
//     std::set<Force *> uniqueForces;

//     for (const auto &body : BODIES)
//     {
//         for (auto &force : body.m_f_registry)
//         {
//             // uniqueForces.insert(force);
//         }
//     }

//     emscripten::val result = emscripten::val::array();
//     int index = 0;

//     for (auto *force : uniqueForces)
// {
//         emscripten::val forceInfo = emscripten::val::object();

//         forceInfo.set("id", index++);

//         Spring *spring = dynamic_cast<Spring *>(force);
//         Motor *motor = dynamic_cast<Motor *>(force);
//         Gravity *gravity = dynamic_cast<Gravity *>(force);

//         if (spring)
//         {
//             forceInfo.set("type", "spring");

//             if (spring->m_two_b_spring)
//             {
//                 int bodyAIndex = -1, bodyBIndex = -1;

//                 for (size_t i = 0; i < BODIES.size(); i++)
//                 {
//                     if (&BODIES[i] == spring->m_rb_a)
//                         bodyAIndex = i;
//                     if (&BODIES[i] == spring->m_rb_b)
//                         bodyBIndex = i;
//                 }

//                 forceInfo.set("bodyAIndex", bodyAIndex);
//                 forceInfo.set("bodyBIndex", bodyBIndex);

//                 Vec2 worldAnchorA = PhysicsMath::transform(spring->m_loc_anchor_a, spring->m_rb_a->m_pos, spring->m_rb_a->m_angle);
//                 Vec2 worldAnchorB = PhysicsMath::transform(spring->m_loc_anchor_b, spring->m_rb_b->m_pos, spring->m_rb_b->m_angle);

//                 forceInfo.set("pointA", emscripten::val::object());
//                 forceInfo["pointA"].set("x", worldAnchorA.m_x);
//                 forceInfo["pointA"].set("y", worldAnchorA.m_y);

//                 forceInfo.set("pointB", emscripten::val::object());
//                 forceInfo["pointB"].set("x", worldAnchorB.m_x);
//                 forceInfo["pointB"].set("y", worldAnchorB.m_y);
//             }
//             else
//             {
//                 int bodyIndex = -1;
//                 for (size_t i = 0; i < BODIES.size(); i++)
//                 {
//                     if (&BODIES[i] == spring->m_rb_a)
//                         bodyIndex = i;
//                 }

//                 forceInfo.set("bodyIndex", bodyIndex);

//                 Vec2 worldAnchorA = spring->m_rb_a->m_pos + spring->m_loc_anchor_a;

//                 forceInfo.set("pointA", emscripten::val::object());
//                 forceInfo["pointA"].set("x", worldAnchorA.m_x);
//                 forceInfo["pointA"].set("y", worldAnchorA.m_y);

//                 forceInfo.set("pointB", emscripten::val::object());
//                 forceInfo["pointB"].set("x", spring->m_anchor.m_x);
//                 forceInfo["pointB"].set("y", spring->m_anchor.m_y);
//             }

//             forceInfo.set("stiffness", spring->m_stiffness);
//             forceInfo.set("damping", spring->m_damping);
//             forceInfo.set("restLength", spring->m_equilibrium_len);
//         }
//         else if (motor)
//         {
//             forceInfo.set("type", "motor");

//             int bodyIndex = -1;
//             for (size_t i = 0; i < BODIES.size(); i++)
//             {
//                 for (auto &f : BODIES[i].m_f_registry)
//                 {
//                     if (f == motor)
//                     {
//                         bodyIndex = i;
//                         break;
//                     }
//                 }
//                 if (bodyIndex >= 0)
//                     break;
//             }

//             forceInfo.set("bodyIndex", bodyIndex);
//             forceInfo.set("targetAngularVelocity", motor->m_target_ang_vel);
//         }
//         else if (gravity)
//         {
//             forceInfo.set("type", "gravity");

//             emscripten::val affectedBodies = emscripten::val::array();
//             int bodyCount = 0;

//             for (size_t i = 0; i < BODIES.size(); i++)
//             {
//                 for (auto &f : BODIES[i].m_f_registry)
//                 {
//                     if (f == gravity)
//                     {
//                         affectedBodies.set(bodyCount++, i);
//                         break;
//                     }
//                 }
//             }

//             forceInfo.set("affectedBodies", affectedBodies);
//         }
//         else
//         {
//             forceInfo.set("type", "unknown");
//         }

//         result.set(index - 1, forceInfo);
//     }

    // return result;
}

EMSCRIPTEN_BINDINGS(physics_engine)
{
    value_object<Vec2Wrapper>("Vec2")
        .field("x", &Vec2Wrapper::x)
        .field("y", &Vec2Wrapper::y);

    register_vector<Vec2Wrapper>("VectorVec2");
    register_vector<float>("VectorFloat");
    register_vector<int>("VectorInt");

    enum_<ShapeType>("ShapeType")
        .value("BOX", ShapeType::BOX)
        .value("CIRCLE", ShapeType::CIRCLE);

    // core functions
    function("step", &step);
    function("setDT", &setDT);
    function("getDT", &getDT);
    function("setPaused", &set_paused);

    // body creation/removal
    function("removeBody", &remove_body);
    function("clearBodies", &clear_bodies);
    function("getBodyCount", &get_body_count);

    // body properties
    function("setBodyPosition", &set_body_position);
    function("setBodyVelocity", &set_body_velocity);
    function("setBodyAngle", &set_body_angle);
    function("setBodyAngularVelocity", &set_body_angular_velocity);
    function("setBodyRestitution", &set_body_restitution);
    function("setBodyFriction", &set_body_friction);

    // body getters
    function("getBodyX", &getBodyX);
    function("getBodyY", &getBodyY);
    function("getBodyAngle", &getBodyAngle);
    function("getBodyPos", &get_body_pos);
    function("getBodyVel", &get_body_vel);
    function("getBodyAngularVelocity", &get_body_angular_velocity);
    function("getBodyShapeType", &get_body_shape_type);
    function("getBodyRadius", &get_body_radius);
    function("getBodyWidth", &get_body_width);
    function("getBodyHeight", &get_body_height);
    function("getTransformedVertices", &get_transformed_vertices);

    // forces
    function("clearForcesOnBody", &clear_forces_on_body);

    // mouse interaction
    function("start_mouse_drag", &start_mouse_drag);
    function("update_mouse_position", &update_mouse_position);
    function("end_mouse_drag", &end_mouse_drag);
    function("getBodyAtPosition", &get_body_at_position);

    // environment


}
