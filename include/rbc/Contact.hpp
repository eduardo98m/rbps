#pragma once
#include <math3d/math3d.hpp>

namespace rbc
{
    struct Contact
    {
        m3d::vec3 normal; // Direction of the contact from body A to body B 
        m3d::scalar penetration_depth; // Positive if bodies are overlapping
        m3d::vec3 pos;              
    };
}