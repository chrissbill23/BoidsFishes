

#ifndef FISH_HPP
#define FISH_HPP

#include "scenes/base/base.hpp"
#include "utils.h"

struct fish_structure
{
    
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
    vcl::vec3 f; // Forces
    vcl::vec3 c; // Color
    vcl::vec3 barmesh; // Color
    vcl::mesh_drawable sphere;

    float r;//scaling factor
    int updated = 10;

    bool left = false;
    bool show_skeleton = true;
    bool show_mesh = false;
    bool show_texture = false;

    skeleton_structure skeleton;
    skinning_structure skinning;
    vcl::mesh_drawable character_visual;

    vcl::segment_drawable_immediate_mode segment_drawer;
    GLuint shader_mesh;

    void setup_data(std::map<std::string,GLuint>& shaders);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene);
    void update();
    void display_skeleton(const std::map<std::string,GLuint>& shaders, const scene_structure& scene, vcl::segment_drawable_immediate_mode& segment_drawer);
};

#endif

