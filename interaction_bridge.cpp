//
// Created by Lukas Campbell on 10/06/2025.
//

#include "headers.h"

using namespace emscripten;

enum class ActiveTool
{
    None = 0,
    Circle = 1,
    Box = 2,
    Spring = 3,
    Motor = 4
};

enum class EventType
{
    None,
    CreateCircle,
    CreateBox,
    CreateSpring,
    AttachMotor,
    DeleteBody
};

struct Event;
struct SpringEvent;

ActiveTool g_active_tool = ActiveTool::None;
std::vector<std::shared_ptr<Event>> g_event_queue;
std::shared_ptr<SpringEvent> g_pending_spring = nullptr;

Rigidbody *g_selected_body = nullptr;

float g_box_width = 1.0f;
float g_box_height = 1.0f;
bool g_box_gravity_enabled = true;
bool g_box_static = false;

float g_circle_radius = 0.5f;
bool g_circle_static = false;
bool g_circle_gravity_enabled = true;

float g_spring_stiffness = 1.0f;
float g_spring_damping = 0.1f;

float g_motor_speed = 1.0f;

struct Event
{
    EventType m_type = EventType::None;
    bool m_is_handled = false;
    bool m_has_read = false;

    virtual void handle() = 0;
    virtual bool is_handled() = 0;
    virtual ~Event() = default;
};

struct CircleEvent : public Event
{
    Vec2 m_pos = Vec2();
    float m_radius = 0.5f;
    bool m_static = false;
    bool m_gravity_enabled = true;
    CircleEvent(const Vec2 &pos, float r, bool is_static, bool gravity_enabled)
        : m_pos(pos), m_radius(r), m_static(is_static), m_gravity_enabled(gravity_enabled)
    {
        m_type = EventType::CreateCircle;
    }

    void handle() override
    {
        Rigidbody rb = Rigidbody(m_radius, m_static);
        rb.m_pos = m_pos;
        rb.m_is_static = m_static;
        if (m_gravity_enabled)
        {
            rb.m_f_registry.push_back(new Gravity());
        }
        BODIES.push_back(rb);
        m_is_handled = true;
    }

    bool is_handled() override
    {
        return m_is_handled && m_has_read;
    }
    ~CircleEvent() override = default;
};

struct BoxEvent : public Event
{
    Vec2 m_pos = Vec2();
    float m_width = 20.0f;
    float m_height = 20.0f;
    bool m_static = false;
    bool m_gravity_enabled = true;

    BoxEvent(const Vec2 &pos, float w, float h, bool is_static, bool gravity_enabled)
        : m_pos(pos), m_width(w), m_height(h), m_static(is_static), m_gravity_enabled(gravity_enabled)
    {
        m_type = EventType::CreateBox;
    }

    void handle() override
    {
        Rigidbody rb = Rigidbody(m_width, m_height, m_static);
        rb.m_pos = m_pos;
        rb.m_is_static = m_static;
        if (m_gravity_enabled)
        {
            rb.m_f_registry.push_back(new Gravity());
        }
        BODIES.push_back(rb);
        m_is_handled = true;
    }

    bool is_handled() override
    {
        return m_is_handled && m_has_read;
    }
};

struct SpringEvent : public Event
{
    Rigidbody *m_rb_a = nullptr;
    Rigidbody *m_rb_b = nullptr;

    Vec2 m_anchor_a = Vec2();
    Vec2 m_anchor_b = Vec2();

    float m_stiffness = 1.0f;
    float m_damping = 0.1f;

    SpringEvent(Rigidbody *rb_a, Rigidbody *rb_b, const Vec2 &anchor_a, const Vec2 &anchor_b, float stiffness, float damping)
        : m_rb_a(rb_a), m_rb_b(rb_b), m_anchor_a(anchor_a), m_anchor_b(anchor_b), m_stiffness(stiffness), m_damping(damping)
    {
        m_type = EventType::CreateSpring;
    }

    void handle() override
    {
        if (m_rb_a && m_rb_b)
        {
            Spring *spring = new Spring(m_rb_a, m_rb_b, m_anchor_a, m_anchor_b, m_stiffness, m_damping);
            m_rb_a->m_f_registry.push_back(spring);
            m_rb_b->m_f_registry.push_back(spring);
        }
        else if (m_rb_a && !m_rb_b)
        {
            Spring *spring = new Spring(m_rb_a, m_anchor_a, m_anchor_b, m_stiffness, m_damping);
            m_rb_a->m_f_registry.push_back(spring);
        }
        else if (!m_rb_a && m_rb_b)
        {
            Spring *spring = new Spring(m_rb_b, m_anchor_b, m_anchor_a, m_stiffness, m_damping);
            m_rb_b->m_f_registry.push_back(spring);
        }
        m_is_handled = true;
    }

    bool is_handled() override
    {
        return m_is_handled && m_has_read;
    }
};

struct MotorEvent : public Event
{
    Rigidbody *m_rb = nullptr;
    float m_speed = 1.0f;

    MotorEvent(Rigidbody *rb, float speed)
        : m_rb(rb), m_speed(speed)
    {
        m_type = EventType::AttachMotor;
    }

    void handle() override
    {
        if (m_rb)
        {
            Motor *motor = new Motor(m_rb, m_speed);
            m_rb->m_f_registry.push_back(motor);
        }
        m_is_handled = true;
    }

    bool is_handled() override
    {
        return m_is_handled && m_has_read;
    }
};

void add_event(std::shared_ptr<Event> event)
{
    g_event_queue.push_back(event);
}

void process_events()
{
    for (auto it = g_event_queue.begin(); it != g_event_queue.end();)
    {
        auto &event = *it;
        if (!event->is_handled())
        {
            event->handle();
        }
        if (event->m_is_handled)
        {
            it = g_event_queue.erase(it);
        }
        else
        {
            ++it;
        }
    }
}


bool is_wall_body(const Rigidbody *body)
{
    for (const auto &wall_body : WALL_BODIES)
    {
        if (wall_body == body)
            return true;
    }
    return false;
}

Rigidbody *get_rigidbody_under_mouse(const Vec2 &p)
{
    for (Rigidbody &rb : BODIES)
    {
        if (&rb == g_selected_body)
            continue;

        if (!Collisions::intersect_aabb_with_point(rb.m_aabb, p))
            continue;

        if (is_wall_body(&rb))
            continue;

        if (rb.contains(p))
            return &rb;
    }
    return nullptr;
}

emscripten::val get_bodies_for_render()
{
    emscripten::val bodies = emscripten::val::array();

    for (size_t i = 0; i < BODIES.size(); ++i)
    {
        const Rigidbody &rb = BODIES[i];
        emscripten::val body_obj = emscripten::val::object();

        body_obj.set("id", static_cast<int>(i));
        body_obj.set("x", rb.m_pos.m_x);
        body_obj.set("y", rb.m_pos.m_y);
        body_obj.set("angle", rb.m_angle);
        body_obj.set("type", rb.m_shape_type == ShapeType::CIRCLE ? "circle" : "box");

        if (rb.m_shape_type == ShapeType::CIRCLE)
        {
            body_obj.set("radius", rb.m_radius);
        }
        else
        {
            body_obj.set("width", rb.m_width);
            body_obj.set("height", rb.m_height);
        }

        body_obj.set("isStatic", rb.m_is_static);
        body_obj.set("isSelected", &rb == g_selected_body);

        bodies.call<void>("push", body_obj);
    }

    return bodies;
}

emscripten::val get_forces_for_render()
{
    emscripten::val forces = emscripten::val::array();

    std::set<Force *> processed_forces;

    for (size_t i = 0; i < BODIES.size(); ++i)
    {
        const Rigidbody &rb = BODIES[i];

        for (const Force *f : rb.m_f_registry)
        {
            if (processed_forces.find(const_cast<Force *>(f)) != processed_forces.end())
                continue;

            processed_forces.insert(const_cast<Force *>(f));

            emscripten::val force_obj = emscripten::val::object();

            if (const Spring *spring = dynamic_cast<const Spring *>(f))
            {
                if (spring->m_two_b_spring)
                {
                    force_obj.set("type", "spring");

                    Vec2 world_a = PhysicsMath::transform(spring->m_loc_anchor_a, spring->m_rb_a->m_pos, spring->m_rb_a->m_angle);
                    Vec2 world_b = PhysicsMath::transform(spring->m_loc_anchor_b, spring->m_rb_b->m_pos, spring->m_rb_b->m_angle);

                    force_obj.set("xa", world_a.m_x);
                    force_obj.set("ya", world_a.m_y);
                    force_obj.set("xb", world_b.m_x);
                    force_obj.set("yb", world_b.m_y);
                }
                else
                {
                    force_obj.set("type", "spring");

                    Vec2 world_a = PhysicsMath::transform(spring->m_loc_anchor_a, spring->m_rb_a->m_pos, spring->m_rb_a->m_angle);

                    force_obj.set("xa", world_a.m_x);
                    force_obj.set("ya", world_a.m_y);
                    force_obj.set("xb", spring->m_anchor.m_x);
                    force_obj.set("yb", spring->m_anchor.m_y);
                }
            }
            else if (const Motor *motor = dynamic_cast<const Motor *>(f))
            {
                force_obj.set("type", "motor");
                force_obj.set("bodyId", static_cast<int>(i));
                force_obj.set("speed", motor->m_target_ang_vel);
            }

            forces.call<void>("push", force_obj);
        }
    }

    if (g_pending_spring)
    {
        emscripten::val preview = emscripten::val::object();
        preview.set("type", "spring_preview");

        if (g_pending_spring->m_rb_a)
        {
            Vec2 world_a = PhysicsMath::transform(g_pending_spring->m_anchor_a,
                                                  g_pending_spring->m_rb_a->m_pos, g_pending_spring->m_rb_a->m_angle);
            preview.set("xa", world_a.m_x);
            preview.set("ya", world_a.m_y);
        }
        else
        {
            preview.set("xa", g_pending_spring->m_anchor_a.m_x);
            preview.set("ya", g_pending_spring->m_anchor_a.m_y);
        }

        forces.call<void>("push", preview);
    }

    return forces;
}

void mouse_down(float x, float y)
{
    Vec2 mouse_pos(x, y);
    std::cout << "Mouse down at: " << mouse_pos.m_x << ", " << mouse_pos.m_y << std::endl;

    std::cout << "Active tool: " << static_cast<int>(g_active_tool) << std::endl;
    std::cout << BODIES.size() << " bodies in the simulation." << std::endl;
    switch (g_active_tool)
    {
    case ActiveTool::Circle:
        add_event(std::make_shared<CircleEvent>(mouse_pos, g_circle_radius, g_circle_static, g_circle_gravity_enabled));
        break;

    case ActiveTool::Box:
        add_event(std::make_shared<BoxEvent>(mouse_pos, g_box_width, g_box_height, g_box_static, g_box_gravity_enabled));
        break;

    case ActiveTool::Spring:
        if (!g_pending_spring)
        {
            Rigidbody *rb = get_rigidbody_under_mouse(mouse_pos);
            if (rb)
            {
                Vec2 loc_anchor = PhysicsMath::transform(mouse_pos - rb->m_pos, Vec2(), -rb->m_angle);
                g_pending_spring = std::make_shared<SpringEvent>(rb, nullptr, loc_anchor, Vec2(), g_spring_stiffness, g_spring_damping);
            }
            else
            {
                g_pending_spring = std::make_shared<SpringEvent>(nullptr, nullptr, mouse_pos, Vec2(), g_spring_stiffness, g_spring_damping);
            }
        }
        else
        {
            Rigidbody *rb = get_rigidbody_under_mouse(mouse_pos);
            if (rb)
            {
                g_pending_spring->m_rb_b = rb;
                g_pending_spring->m_anchor_b = PhysicsMath::transform(mouse_pos - rb->m_pos, Vec2(), -rb->m_angle);
            }
            else
            {
                g_pending_spring->m_anchor_b = mouse_pos;
            }

            add_event(g_pending_spring);
            g_pending_spring = nullptr;
        }
        break;

    case ActiveTool::Motor:
    {
        Rigidbody *rb = get_rigidbody_under_mouse(mouse_pos);
        if (rb)
        {
            add_event(std::make_shared<MotorEvent>(rb, g_motor_speed));
        }
    }
    break;

    case ActiveTool::None:
        g_selected_body = get_rigidbody_under_mouse(mouse_pos);
        break;
    }
}

void mouse_move(float x, float y)
{
    if (g_pending_spring)
    {
        g_pending_spring->m_anchor_b = Vec2(x, y);
    }
}

void mouse_up(float x, float y)
{
}

void set_active_tool(int tool_id)
{
    g_active_tool = static_cast<ActiveTool>(tool_id);
    if (g_pending_spring)
    {
        g_pending_spring = nullptr;
    }
}

void set_box_properties(float width, float height, bool is_static, bool gravity)
{
    g_box_width = width;
    g_box_height = height;
    g_box_static = is_static;
    g_box_gravity_enabled = gravity;
}

void set_circle_properties(float radius, bool is_static, bool gravity)
{
    g_circle_radius = radius;
    g_circle_static = is_static;
    g_circle_gravity_enabled = gravity;
}

void set_spring_properties(float stiffness, float damping)
{
    g_spring_stiffness = stiffness;
    g_spring_damping = damping;
}

void set_motor_properties(float speed)
{
    g_motor_speed = speed;
}

EMSCRIPTEN_BINDINGS(physics_engine)
{
    function("mouseDown", &mouse_down);
    function("mouseMove", &mouse_move);
    function("mouseUp", &mouse_up);

    function("setActiveTool", &set_active_tool);
    function("processEvents", &process_events);

    function("getBodiesForRender", &get_bodies_for_render);
    function("getForcesForRender", &get_forces_for_render);

    function("setBoxProperties", &set_box_properties);
    function("setCircleProperties", &set_circle_properties);
    function("setSpringProperties", &set_spring_properties);
    function("setMotorProperties", &set_motor_properties);
}
