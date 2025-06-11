//
// Created by Lukas Campbell on 10/06/2025.
//

#include "headers.h"

void update_mouse_coords(emscripten::val);
void update_mouse_button_state(emscripten::val);

extern "C" {

EMSCRIPTEN_KEEPALIVE void mouseMove(float x, float y)
{
    emscripten::val evt = emscripten::val::object();
    evt.set("x", x); evt.set("y", y);
    update_mouse_coords(evt);
}

EMSCRIPTEN_KEEPALIVE void mouseDown(float x, float y)
{
    std::cout << "Mouse down at (" << x << ", " << y << ")" << std::endl;
    emscripten::val evt = emscripten::val::object();
    evt.set("x", x); evt.set("y", y);
    evt.set("left", true);
    update_mouse_button_state(evt);

    std::cout << BODIES.size() << " bodies in the simulation." << std::endl;
}

EMSCRIPTEN_KEEPALIVE void mouseUp(float x, float y)
{
    emscripten::val evt = emscripten::val::object();
    evt.set("x", x); evt.set("y", y);
    evt.set("left", false);
    update_mouse_button_state(evt);
}

} 
