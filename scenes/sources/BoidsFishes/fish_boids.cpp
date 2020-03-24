
#include "fish_boids.hpp"

#include <random>

#ifdef FISH_BOIDS

using namespace vcl;

void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    float dt = 0.02f;
    if(! (timer.update()>0) ) dt=0;

    set_gui();
	if(particles.size()<size_t(gui_scene.number_fish))
		create_new_particle(shaders);

    compute_time_step(dt);
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
    {
        fish_structure& part = particles[k];
        part.show_skeleton = gui_scene.skeleton;
        part.show_mesh = gui_scene.mesh;
        part.show_texture = gui_scene.texture;
        part.frame_draw(shaders,scene);
    }
    draw(pool, scene.camera);
}

void scene_model::compute_time_step(float dt)
{

    float mu_box = 1;

    const size_t N = particles.size();

	for (size_t i = 0; i < N; ++i) {
		fish_structure& particle = particles[i];
		vec3& v = particle.v;
		vec3& p = particle.p;
		vec3& f = particle.f;
		float& r = particle.r;

		vcl::vec3 f1= align(particles, i, gui_scene.radiusAllign, gui_scene.maxa,gui_scene.maxv);
		vcl::vec3 f2 = cohesion(particles, i, gui_scene.radiusCohersion,  gui_scene.maxa, gui_scene.maxv);
		vcl::vec3 f3 = separate(particles, i, gui_scene.radiusSeparate, gui_scene.maxa, gui_scene.maxv);
		vcl::vec3 f4 = turn(particles, i);
		vcl::vec3 ftot = gui_scene.allign_factor * f1 +
			gui_scene.cohesion_factor * f2 +
			gui_scene.separate_factor * f3 + 0.5*f4;

		f += ftot ;
		v = v + dt * f;
		if (norm(v) > gui_scene.maxv) {
			v = normalize(v) * gui_scene.maxv;
		}
		p = p + dt * v;
		//f *= 0;
		// check bounding box

		float d_x = abs(p.x) + r - gui_scene.space;
		float d_y = abs(p.y) + r - gui_scene.space;
		float d_z = abs(p.z) + r - gui_scene.space;

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
	
	pool.update_position(pool0.position);
	pool.update_normal(pool0.normal);

	pool.uniform.transform.rotation = rotation_from_axis_angle_mat3({ 0,1,0 }, std::sin(timer.t/(2*3.14159)));
}


vcl::vec3 align(std::vector<fish_structure> particles, const size_t idx_particle, float r, float maxa, float maxv) {
	vcl::vec3 avg_v;
	fish_structure currentp = particles[idx_particle];
	int neighCount = 0;
	for (size_t i = 0; i < particles.size(); i++) {
		float d = norm(currentp.p - particles[i].p);
		if (idx_particle != i && d<r) {
			avg_v += particles[i].v;
			neighCount++;
		}

	}

	if (neighCount > 0) {
		avg_v /= neighCount;
		avg_v = normalize(avg_v) * maxv;
		//steer force  desired velocity - current velocity
		avg_v = avg_v - currentp.v;

		if (norm(avg_v) > maxa) {
			avg_v = normalize(avg_v) * maxa;
		}

	}
	else {
		avg_v = vcl::vec3(0, 0, 0);
	}
	
	return avg_v;
}


vcl::vec3 cohesion(std::vector<fish_structure> particles, const size_t idx_particle, float r, float maxa, float maxv) {
	vcl::vec3 avg_p;
	fish_structure currentp = particles[idx_particle];
	int neighCount = 0;
	for (size_t i = 0; i < particles.size(); i++) {
		float d = norm(currentp.p - particles[i].p);
		if (idx_particle != i && d < r) {
			avg_p += particles[i].p;
			neighCount++;
		}

	}

	if (neighCount > 0) {
		avg_p /= neighCount;
		avg_p = avg_p - currentp.p;
		avg_p = normalize(avg_p) * maxv;

		if (norm(avg_p) > maxa) {
			avg_p = normalize(avg_p) * maxa;
		}
	}
	else {
		avg_p = vcl::vec3(0, 0, 0);
	}

	return avg_p;
}


vcl::vec3 separate(std::vector<fish_structure> particles, const size_t idx_particle, float r, float maxa, float maxv) {
	vcl::vec3 avg_v;
	fish_structure currentp = particles[idx_particle];
	int neighCount = 0;
	for (size_t i = 0; i < particles.size(); i++) {
		float d = norm(currentp.p - particles[i].p);
		if (idx_particle != i && d < r) {
			vcl::vec3 oppvec = currentp.p - particles[i].p;
			oppvec /= pow(d,2);
			avg_v += oppvec;
			neighCount++;
		}

	}
	if (neighCount > 0) {
		avg_v /= neighCount;
		avg_v = normalize(avg_v) * maxv;
		avg_v = avg_v - currentp.v;
		if (norm(avg_v) > maxa) {
			avg_v = normalize(avg_v) * maxa;
		}

	}
	return avg_v;
} 


vcl::vec3 turn(std::vector<fish_structure> particles, const size_t idx_particle) {
	vcl::vec3 fturn;
	fish_structure currentp = particles[idx_particle];
	vec3 p2 = { 0,0,0 };
	
	return (norm(currentp.p - p2))*normalize(p2- currentp.p);
}



void scene_model::create_new_particle(std::map<std::string,GLuint>& shaders)
{
    // Emission of new particle if needed
    timer.periodic_event_time_step = gui_scene.time_interval_new_sphere;
    const bool is_new_particle = timer.event;
    static const std::vector<vec3> color_lut = {{1,0,0},{0,1,0},{1,1,0},{1,0,1},{0,1,1}};

    if( is_new_particle && gui_scene.add_sphere)
    {
        fish_structure new_particle;

        new_particle.r = 0.005f;
        new_particle.c = color_lut[int(rand_interval()*color_lut.size())];

        // Initial position
        /*
		x = r cos(v) cos(u)
		y = r cos(v) sin(u)     u = [0, 2*Pi)
		z = r sin(v)            v = [-Pi/2, Pi/2]
		*/

        // Initial speed
        const float u = rand_interval(0, 2*3.1415f);
		const float v = rand_interval(-3.1415f/2, 3.1415f/2);
		new_particle.p = vec3( cos(v) * cos(u),  cos(v) * sin(u),  sin(v));
        new_particle.v = vec3( cos(v) * cos(u),  cos(v) * sin(u),  sin(v));

        new_particle.setup_data(shaders);

        particles.push_back(new_particle);

    }
}

void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
	float s = 50;
	pool0 = mesh_primitive_sphere(s, { 0,0,0 }, 150, 2 * 150);
	pool = pool0;
	pool.shader = shaders["mesh_bf"];
	pool.uniform.color = { 1,1,1 };
	pool.texture_id = create_texture_gpu(image_load_png("assets/caustics.png"));
}

void scene_model::set_gui()
{
    // Can set the speed of the animation
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Interval create fish", &gui_scene.time_interval_new_sphere, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderInt("Number of fishes", &gui_scene.number_fish, 1, 50, "%1d fishes");
    //Boids forces factor:
    ImGui::SliderFloat("factor allign", &gui_scene.allign_factor, 0.0f, 1.0f, "%.1f ");
    ImGui::SliderFloat("factor cohesion", &gui_scene.cohesion_factor, 0.0f, 1.0f, "%.1f ");
    ImGui::SliderFloat("factor separate", &gui_scene.separate_factor, 0.0f, 1.0f, "%.1f ");

    //Boids Max parameters
    ImGui::SliderFloat("max acceleration", &gui_scene.maxa, 0.0f, 2.0f, "%.1f ");
    ImGui::SliderFloat("max velocity", &gui_scene.maxv, 0.0f, 2.0f, "%.1f ");

    //Space
    ImGui::SliderFloat("Space distance", &gui_scene.space, 1.0f, 20.0f, "%.5f ");
    ImGui::SliderFloat("radius Allign", &gui_scene.radiusAllign, 0.f, 20.0f, "%.5f ");
    ImGui::SliderFloat("radius cohersion", &gui_scene.radiusCohersion, 0.f, 20.0f, "%.5f ");
    ImGui::SliderFloat("radius separate", &gui_scene.radiusSeparate, 0.f, 20.0f, "%.5f ");

    ImGui::Checkbox("Add fishes", &gui_scene.add_sphere);
    ImGui::Checkbox("Skeleton", &gui_scene.skeleton);
    ImGui::Checkbox("Mesh", &gui_scene.mesh);
    ImGui::Checkbox("Texture", &gui_scene.texture);

    bool stop_anim  = ImGui::Button("Stop"); ImGui::SameLine();
    bool start_anim = ImGui::Button("Start");

    if(stop_anim)  timer.stop();
    if(start_anim) timer.start();
}


#endif
