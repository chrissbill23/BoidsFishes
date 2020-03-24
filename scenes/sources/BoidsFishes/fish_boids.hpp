#pragma once

#include "scenes/base/base.hpp"

#ifdef FISH_BOIDS

#include "fish.hpp"

struct gui_scene_structure
{
        bool add_sphere = false;
        bool skeleton = true;
        bool mesh = false;
        bool texture = false;
        float time_interval_new_sphere = 0.5f;
	int number_fish=1;


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
    void create_new_particle(std::map<std::string,GLuint>& shaders);

    std::vector<fish_structure> particles;

    vcl::mesh_drawable pool;
    vcl::mesh pool0;
    vcl::timer_event timer;
    gui_scene_structure gui_scene;
};

vcl::vec3 align(std::vector<fish_structure> particles, const size_t idx_particle, float r, float maxa, float maxv);
vcl::vec3 cohesion(std::vector<fish_structure> particles, const size_t idx_particle, float r, float maxa, float maxv);
vcl::vec3 separate(std::vector<fish_structure> particles, const size_t idx_particle, float r, float maxa, float maxv);
vcl::vec3 turn(std::vector<fish_structure> particles, const size_t idx_particle);


#endif
