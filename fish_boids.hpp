#pragma once

#include "scenes/base/base.hpp"

#ifdef FISH_BOIDS

#include "fish.hpp"

struct gui_scene_structure
{
    bool add_sphere = false;
    float time_interval_new_sphere = 0.5f;

	//forces factors
	float allign_factor = 0.0f;
	float cohesion_factor = 0.0f;
	float separate_factor = 0.0f;

	//max parameters
	float maxa = 0.f;
	float maxv = 0.f;

	//space
	float space = 2.0f;
	float radiusAllign = 0.f;
	float radiusCohersion = 0.f;
	float radiusSeparate = 0.f;
};

struct scene_model : scene_base
{

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    void set_gui();

    void compute_time_step(float dt);
    void create_new_bubble();
	void create_new_particle();
    void display_particles(scene_structure& scene);

    std::vector<particle_structure> particles;
	std::vector<particle_structure> bubbles;

    vcl::mesh_drawable sphere;      // Visual display of particles
    vcl::segments_drawable borders; // Visual display of borders
	vcl::mesh_drawable pool;
	vcl::mesh_drawable pool2;
	vcl::mesh_drawable pool3;
	vcl::mesh_drawable pool4;
    vcl::timer_event timer;
    gui_scene_structure gui_scene;

	vcl::mesh_drawable bubble;
	vcl::timer_event timer_bubble;
};



#endif
