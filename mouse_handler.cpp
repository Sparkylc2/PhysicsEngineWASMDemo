//
// Created by Lukas Campbell on 10/06/2025.
//
#include "headers.h"
#include "mouse_handler.h"



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

void add_spring_between(int index_a, int index_b, const Vec2 &world_anchor_a, const Vec2 &world_anchor_b, float stiffness, float damping)
{
    if (index_a < 0 || index_a >= BODIES.size() ||
        index_b < 0 || index_b >= BODIES.size() ||
        index_a == index_b)
        return;

    Rigidbody *rb_a = &BODIES[index_a];
    Rigidbody *rb_b = &BODIES[index_b];

    Spring *spring = new Spring(rb_a, rb_b, world_anchor_a, world_anchor_b);
    spring->m_stiffness = stiffness;
    spring->m_damping = damping;

    rb_a->m_f_registry.push_back(spring);
    rb_b->m_f_registry.push_back(spring);
}

void add_spring_to_point_js(int index, float anchor_x, float anchor_y, float stiffness, float damping)
{
    if (index < 0 || index >= BODIES.size())
        return;

    Rigidbody *rb = &BODIES[index];

    Vec2 anchor(anchor_x, anchor_y);
    Spring *spring = new Spring(rb, Vec2(0, 0), anchor);
    spring->m_stiffness = stiffness;
    spring->m_damping = damping;

    rb->m_f_registry.push_back(spring);
}

void add_spring_to_point(int index, const Vec2 &anchor, float stiffness, float damping)
{
    if (index < 0 || index >= BODIES.size())
        return;

    Rigidbody *rb = &BODIES[index];

    Spring *spring = new Spring(rb, Vec2(0, 0), anchor);
    spring->m_stiffness = stiffness;
    spring->m_damping = damping;

    rb->m_f_registry.push_back(spring);
}
void add_spring_to_point(int index, const Vec2 &world_anchor_a, const Vec2 &anchor, float stiffness, float damping)
{
    if (index < 0 || index >= BODIES.size())
        return;

    Rigidbody *rb = &BODIES[index];

    Spring *spring = new Spring(rb, world_anchor_a, anchor);
    spring->m_stiffness = stiffness;
    spring->m_damping = damping;

    rb->m_f_registry.push_back(spring);
}

void add_motor_to_body(int index, float target_angular_velocity)
{
    if (index < 0 || index >= BODIES.size())
        return;

    Rigidbody *rb = &BODIES[index];
    Motor *motor = new Motor(rb, target_angular_velocity);

    rb->m_f_registry.push_back(motor);
}




Vec2 g_mouseCoords{};
bool g_isMouseDown = false;
bool g_mouseDownLeft = false;
bool g_isMouseUp = false;
ToolState g_toolState{};
Rigidbody *g_currUnderMouse = nullptr;
Rigidbody *g_initialSelected = nullptr;

MouseObjectResult::MouseObjectResult(Rigidbody *selected, const Vec2 &coordinate)
    : selected_rb(selected), coord(coordinate)
{
    if (selected_rb)
    {
        selected_rb_pos = selected_rb->m_pos;
        local_coord = PhysicsMath::transform(coord - selected_rb_pos,
                                             selected_rb_pos,
                                             -selected_rb->m_angle);
    }
}

Rigidbody *getRigidbodyUnderMouse(const Vec2 &p)
{
    for (Rigidbody &rb : BODIES)
    {
        if (&rb == g_initialSelected)
            continue;

        if (!Collisions::intersect_aabb_with_point(rb.m_aabb, p))
            continue;

        if (rb.contains(p))
            return &rb;
    }
    return nullptr;
}

void dispatchToolClick(const MouseObjectResult &clickInfo)
{
    switch (g_toolState.active)
    {
    case Tool::Circle:
        if (!clickInfo.selected_rb)
            createCircleBody(clickInfo.coord, g_toolState.circleRadius);
        break;

    case Tool::Box:
        if (!clickInfo.selected_rb)
            createBoxBody(clickInfo.coord);
        break;

    case Tool::Motor:
        if (clickInfo.selected_rb)
            attachMotor(clickInfo.selected_rb, clickInfo.local_coord);
        break;

    case Tool::Spring:
        if (!g_toolState.pendingAnchor)
        {
            g_toolState.pendingAnchor = clickInfo;
        }
        else
        {
            createSpring(*g_toolState.pendingAnchor, clickInfo);
            g_toolState.pendingAnchor.reset();
        }
        break;

    case Tool::None:
    default:
        break;
    }
}

void update_mouse_coords(emscripten::val jsPos)
{
    g_mouseCoords = {jsPos["x"].as<float>(), jsPos["y"].as<float>()};
    g_currUnderMouse = getRigidbodyUnderMouse(g_mouseCoords);
    std::cout << "Mouse coords updated: (" << g_mouseCoords.m_x << ", "
              << g_mouseCoords.m_y << "), under mouse: "
              << (g_currUnderMouse ? "yes" : "no") << std::endl;
}

void update_mouse_button_state(emscripten::val jsEvt)
{
    bool left = jsEvt["left"].as<bool>();

    update_mouse_coords(jsEvt);

    if (left && !g_isMouseDown)
    {
        g_mouseDownLeft = g_isMouseDown = true;
    }
    else if (!left && g_isMouseDown)
    {
        g_isMouseDown = false;
        g_mouseDownLeft = false;

        MouseObjectResult clickInfo(g_currUnderMouse, g_mouseCoords);
        dispatchToolClick(clickInfo);

        g_initialSelected = g_currUnderMouse;
    }
}

static inline int bodyIndex(Rigidbody *rb)
{
    return rb ? static_cast<int>(rb - &BODIES[0]) : -1;
}





void createCircleBody(const Vec2 &at, float r)
{
    add_circle_body(at.m_x, at.m_y, r, false);
}

void createBoxBody(const Vec2 &at)
{
    constexpr float SIDE = 30.f;
    add_box_body(at.m_x, at.m_y, SIDE, SIDE, false);
}

void attachMotor(Rigidbody *rb, const Vec2 &)
{
    int idx = bodyIndex(rb);
    if (idx >= 0)
        add_motor_to_body(idx, 5.f /* rad s-1 → tweak in UI later */);
}





void createSpring(const MouseObjectResult &a,
                  const MouseObjectResult &b)
{
    int idxA = bodyIndex(a.selected_rb);
    int idxB = bodyIndex(b.selected_rb);

    constexpr float K = 200.f;
    constexpr float D = 10.f;

    if (idxA >= 0 && idxB >= 0)
    {
        add_spring_between(idxA, idxB, a.coord, b.coord, K, D);
    }
    else if (idxA >= 0)
    {
        add_spring_to_point(idxA, a.coord, b.coord, K, D);
    }
    else if (idxB >= 0)
    {
        add_spring_to_point(idxB, b.coord, a.coord, K, D);
    }
}



extern "C"
{
    EMSCRIPTEN_KEEPALIVE void setActiveTool(int toolId)
    {
        g_toolState.active = static_cast<Tool>(toolId);
        g_toolState.pendingAnchor.reset();
    }

    EMSCRIPTEN_KEEPALIVE void setCircleRadius(float r)
    {
        g_toolState.circleRadius = r;
    }
}
