//
// Created by Lukas Campbell on 10/06/2025.
//
#pragma once
#include "headers.h"

enum class Tool
{
    None,
    Circle,
    Box,
    Spring,
    Motor
};

struct MouseObjectResult
{
    Rigidbody *selected_rb = nullptr;
    Vec2 selected_rb_pos = {};
    Vec2 coord = {};
    Vec2 local_coord = {};

    MouseObjectResult(Rigidbody *selected, const Vec2 &coordinate);
};

struct ToolState
{
    Tool active = Tool::None;
    float circleRadius = 0.5f;

    std::optional<MouseObjectResult> pendingAnchor{};
};

extern Vec2 g_mouseCoords;
extern bool g_isMouseDown;
extern bool g_mouseDownLeft;
extern bool g_isMouseUp;
extern ToolState g_toolState;
extern Rigidbody *g_currUnderMouse;
extern Rigidbody *g_initialSelected;

Rigidbody *getRigidbodyUnderMouse(const Vec2 &p);
void createCircleBody(const Vec2 &at, float radius);
void createBoxBody(const Vec2 &at);
void attachMotor(Rigidbody *body, const Vec2 &localAnchor);
void createSpring(const MouseObjectResult &a, const MouseObjectResult &b);
void dispatchToolClick(const MouseObjectResult &clickInfo);
void update_mouse_coords(emscripten::val jsPos);
void update_mouse_button_state(emscripten::val jsEvt);

extern "C"
{
    EMSCRIPTEN_KEEPALIVE void setActiveTool(int toolId);
    EMSCRIPTEN_KEEPALIVE void setCircleRadius(float r);
}
