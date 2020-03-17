

#ifndef FISH_HPP
#define FISH_HPP
// Structure of a particle
#include "scenes/base/base.hpp"

struct particle_structure
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
    vcl::vec3 f; // Forces
	float t;
	float ampl;
    vcl::vec3 c; // Color
    float r;     // Radius

};

vcl::vec3 align(std::vector<particle_structure> particles, int idx_particle, float r, float maxa, float maxv);
vcl::vec3 cohesion(std::vector<particle_structure> particles, int idx_particle, float r, float maxa, float maxv);
vcl::vec3 separate(std::vector<particle_structure> particles, int idx_particle, float r, float maxa, float maxv);
vcl::vec3 turn(std::vector<particle_structure> particles, int idx_particle, float r, float maxa, float maxv);

#endif

