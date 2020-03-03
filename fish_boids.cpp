
#include "fish_boids.hpp"

#include <random>

#ifdef FISH_BOIDS

using namespace vcl;







void scene_model::frame_draw(std::map<std::string,GLuint>& , scene_structure& scene, gui_structure& )
{
    float dt = 0.02f;
    if(! (timer.update()>0) ) dt=0;

    set_gui();
	if(particles.size()<3)
		create_new_particle();
    compute_time_step(dt);

    display_particles(scene);
    draw(borders, scene.camera);

}

void scene_model::compute_time_step(float dt)
{

    float alpha = 0.99f;
    float beta = 0.99f;
    float epsilon = 0.1f;
    float mu = 0.2f;
    float mu_box = 1;

    // Set forces (gravity)
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
        particles[k].f = vec3(0,0,0); //particles[k].f = vec3(0,-9.81f,0);


    // Integrate position and speed of particles through time
    for(size_t k=0; k<N; ++k) {
        particle_structure& particle = particles[k];
        vec3& v = particle.v;
        vec3& p = particle.p;
        vec3 const& f = particle.f;
        float& r = particle.r;

        //v = (1-0.9f*dt) * v + dt * f; // gravity + friction force
        p = p + dt * v;


        // check other particles
        for (size_t k_other = 0; k_other < N; ++k_other)
        {
            if (k_other == k)
                continue;
            particle_structure& particle_o = particles[k_other];
            vec3& v_o = particle_o.v;
            vec3& p_o = particle_o.p;
            vec3 const& f_o = particle_o.f;
            float& r_o = particle_o.r;

            float d = r + r_o - norm(p - p_o);
            
            
            if (d > 0)
            {
                vec3 u = (p - p_o) / norm(p - p_o);
                vec3 v_relat = (v_o - v) / norm(v - v_o);
                float j = dot(v_o - v, u);
                if (norm(v_relat) > epsilon)
                {
                    v = alpha * v + beta * j * u;
                    v_o = alpha * v_o - beta * j * u;
                }
                else
                {
                    v *= mu;
                    v_o *= mu;
                }

                p = p + 0.5 * d * u;
                p_o = p_o - 0.5 * d * u;
                
            }

        }
               
        // check bounding box
		//r + r_o - norm(p - p_o);
        float d_x = abs(p.x) + r - 1;
        float d_y = abs(p.y) + r - 1;
        float d_z = abs(p.z) + r - 1;

        if (d_x > 0) 
        {
            v.x = -mu_box * v.x;
            p.x = p.x - d_x * p.x/abs(p.x);
        }
        if (d_y > 0)
        {
            v.y = -mu_box * v.y;
            p.y = p.y - d_y * p.y / abs(p.y);
        }
        if (d_z > 0)
        {
            v.z = -mu_box * v.z;
            p.z = p.z - d_z * p.z / abs(p.z);
        }
        
    }
}


void scene_model::create_new_particle()
{
    // Emission of new particle if needed
    timer.periodic_event_time_step = gui_scene.time_interval_new_sphere;
    const bool is_new_particle = timer.event;
    static const std::vector<vec3> color_lut = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1}};

    if( is_new_particle && gui_scene.add_sphere)
    {
        particle_structure new_particle;

        new_particle.r = 0.08f;
        new_particle.c = color_lut[int(rand_interval()*color_lut.size())];

        // Initial position
        new_particle.p = vec3(0,0,0);

        // Initial speed
        const float theta = rand_interval(0, 2*3.14f);
        new_particle.v = vec3( 0.5*std::cos(theta), 0.5f, 0.5*std::sin(theta));

        particles.push_back(new_particle);

    }
}
void scene_model::display_particles(scene_structure& scene)
{
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
    {
        const particle_structure& part = particles[k];

        sphere.uniform.transform.translation = part.p;
        sphere.uniform.transform.scaling = part.r;
        sphere.uniform.color = part.c;
        draw(sphere, scene.camera);
    }
}




void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    sphere = mesh_drawable( mesh_primitive_sphere(1.0f));
    sphere.shader = shaders["mesh"];

    std::vector<vec3> borders_segments = {{-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
                                          {-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
                                          {-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1}};
    borders = segments_gpu(borders_segments);
    borders.uniform.color = {0,0,0};
    borders.shader = shaders["curve"];

}



void scene_model::set_gui()
{
    // Can set the speed of the animation
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Interval create sphere", &gui_scene.time_interval_new_sphere, 0.05f, 2.0f, "%.2f s");
    ImGui::Checkbox("Add sphere", &gui_scene.add_sphere);

    bool stop_anim  = ImGui::Button("Stop"); ImGui::SameLine();
    bool start_anim = ImGui::Button("Start");

    if(stop_anim)  timer.stop();
    if(start_anim) timer.start();
}





#endif
