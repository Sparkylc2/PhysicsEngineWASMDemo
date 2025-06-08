//
// Created by Lukas Campbell on 08/06/2025.
//
#include "headers.h"

using namespace emscripten;


extern std::vector<Rigidbody> BODIES;
extern void step();


int add_circle_body(float x, float y, float radius, bool is_static)
{
    BODIES.emplace_back(radius, is_static);
    Rigidbody &body = BODIES.back();
    body.m_pos = Vec2(x, y);
    body.update_aabb();
    return BODIES.size() - 1;
}

int add_box_body(float x, float y, float width, float height, bool is_static)
{
    BODIES.emplace_back(width, height, is_static);
    Rigidbody &body = BODIES.back();
    body.m_pos = Vec2(x, y);
    body.update_aabb();
    return BODIES.size() - 1;
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

float get_body_x(int index)
{
    if (index >= 0 && index < BODIES.size())
        return BODIES[index].m_pos.m_x;
    return 0.0f;
}

float get_body_y(int index)
{
    if (index >= 0 && index < BODIES.size())
        return BODIES[index].m_pos.m_y;
    return 0.0f;
}

float get_body_angle(int index)
{
    if (index >= 0 && index < BODIES.size())
        return BODIES[index].m_angle;
    return 0.0f;
}

float get_body_vx(int index)
{
    if (index >= 0 && index < BODIES.size())
        return BODIES[index].m_vel.m_x;
    return 0.0f;
}

float get_body_vy(int index)
{
    if (index >= 0 && index < BODIES.size())
        return BODIES[index].m_vel.m_y;
    return 0.0f;
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
        BODIES.erase(BODIES.begin() + index);
    }
}

void clear_bodies()
{
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
void set_paused(bool paused)
{
    IS_PAUSED = paused;
}

struct Vec2Wrapper
{
    float x, y;

    Vec2Wrapper() : x(0.0f), y(0.0f) {}

    Vec2Wrapper(float x_, float y_) : x(x_), y(y_) {}
};

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





EMSCRIPTEN_BINDINGS(physics_engine)
{
    value_object<Vec2Wrapper>("Vec2")
        .field("x", &Vec2Wrapper::x)
        .field("y", &Vec2Wrapper::y);

    register_vector<Vec2Wrapper>("VectorVec2");

    enum_<ShapeType>("ShapeType")
        .value("BOX", ShapeType::BOX)
        .value("CIRCLE", ShapeType::CIRCLE);

    function("step", &step);
    function("setDT", &set_dt);
    function("setPaused", &set_paused);

    function("addCircleBody", &add_circle_body);
    function("addBoxBody", &add_box_body);
    function("removeBody", &remove_body);
    function("clearBodies", &clear_bodies);
    function("getBodyCount", &get_body_count);

    function("setBodyPosition", &set_body_position);
    function("setBodyVelocity", &set_body_velocity);
    function("setBodyAngle", &set_body_angle);
    function("setBodyAngularVelocity", &set_body_angular_velocity);
    function("setBodyRestitution", &set_body_restitution);
    function("setBodyFriction", &set_body_friction);

    function("getBodyX", &get_body_x);
    function("getBodyY", &get_body_y);
    function("getBodyAngle", &get_body_angle);
    function("getBodyVx", &get_body_vx);
    function("getBodyVy", &get_body_vy);
    function("getBodyAngularVelocity", &get_body_angular_velocity);
    function("getBodyShapeType", &get_body_shape_type);
    function("getBodyRadius", &get_body_radius);
    function("getBodyWidth", &get_body_width);
    function("getBodyHeight", &get_body_height);
    function("getTransformedVertices", &get_transformed_vertices);

    function("getDT", &get_dt);



}


