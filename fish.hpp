

#ifndef FISH_HPP
#define FISH_HPP
// Structure of a particle
#include "scenes/base/base.hpp"

struct particle_structure
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
    vcl::vec3 f; // Forces

    vcl::vec3 c; // Color
    float r;     // Radius
};

#endif

