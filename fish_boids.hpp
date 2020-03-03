#pragma once

#include "scenes/base/base.hpp"

#ifdef FISH_BOIDS

#include "fish.hpp"

struct gui_scene_structure
{
    bool add_sphere = true;
    float time_interval_new_sphere = 0.5f;
};

struct scene_model : scene_base
{

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    void set_gui();

    void compute_time_step(float dt);
    void create_new_particle();
    void display_particles(scene_structure& scene);

    std::vector<particle_structure> particles;

    vcl::mesh_drawable sphere;      // Visual display of particles
    vcl::segments_drawable borders; // Visual display of borders

    vcl::timer_event timer;
    gui_scene_structure gui_scene;
};






#endif
