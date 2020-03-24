
#include "fish.hpp"

using namespace vcl;

#define PI 3.14159265

void fish_structure::setup_data(std::map<std::string,GLuint>& shaders){

    segment_drawer.init();
    segment_drawer.uniform_parameter.color = {0.0f,0.0f,0.0f};
    sphere = mesh_drawable( mesh_primitive_sphere(.3f));
    sphere.shader = shaders["mesh"];

    load_character_data(skeleton, skinning, character_visual,barmesh);
    update();
}


void fish_structure::display_skeleton(const std::map<std::string,GLuint>& shaders,
                      const scene_structure& scene,
                      segment_drawable_immediate_mode& segment_drawer)
{

    auto skeleton_current = skeleton.curr_pose;
    const size_t N = skeleton_current.size();
    for(size_t k=1; k<N; ++k)
    {
        int parent = skeleton.connectivity[k].parent;
        const vec3& p1 = skeleton_current[parent].p;
        const vec3& p2 = skeleton_current[k].p;

        segment_drawer.uniform_parameter.p1 = p1;
        segment_drawer.uniform_parameter.p2 = p2;
        segment_drawer.draw(shaders.at("segment_im"),scene.camera);
        
        sphere.uniform.transform.translation = skeleton_current[k].p;
        sphere.uniform.transform.scaling = 0.05;
        sphere.uniform.color = c;
        draw(sphere, scene.camera);
    }
    sphere.uniform.transform.translation = skeleton_current[0].p;
    sphere.uniform.transform.scaling = 0.05;
    sphere.uniform.color = c;
    draw(sphere, scene.camera);

}

/*void particle_structure::update(){
    //p.z= skeleton.rest_pose[skeleton.plead].p.z;
    auto t = p - skeleton.rest_pose[skeleton.plead].p;
    auto tcutt = p - skeleton.curr_pose[skeleton.plead].p;
    const size_t N = skeleton.curr_pose.size();
    auto newp = skeleton.rest_pose[skeleton.plead].p + t;
    auto oldp = skeleton.curr_pose[skeleton.plead].p;
    skeleton.middlecurr = skeleton.middle + t;
    //float angle = std::acos(dot(skeleton.middle,newp)/(mag(skeleton.middle)*mag(newp)));
    //float angle = std::acos(dot(skeleton.curr_pose[skeleton.plead].p,newp)/(mag(skeleton.curr_pose[skeleton.plead].p)*mag(newp)));
    skeleton.curr_pose[skeleton.plead].p = newp;
    //if(angle* 180.0 / PI <=1)
    //std::cout<<newp.x<<" --- "<<newp.y<<" --- "<<newp.z<<" ANGLE = "<<angle* 180.0 / PI<<std::endl;
    //skeleton.middlecurr= q.apply(skeleton.middlecurr);


    timer.update();
   auto v = std::sin(timer.t*PI);
    auto norm = normalize(cross(skeleton.curr_pose[N-2].p ,vec3(1,0,0)) + (skeleton.curr_pose[N-2].p - skeleton.rest_pose[N-2].p) );
    mat3 mat = vcl::rotation_from_axis_angle_mat3(norm,std::sin(timer.t*2*PI)*float(PI/4));
    vec3 vect = normalize(mat * skeleton.curr_pose[N-1].p);
    //skeleton.curr_pose[N-1].p = vect;
    //rotation_from_axis_angle_mat3(cross(skeleton.curr_pose[N-1].p ,),PI/4)
    if(updated == 0){
    //std::cout<<" --- "<<std::sin(timer.t*2*PI)*0.1<<std::endl;
    std::cout<<norm.x<<" --- "<<norm.y<<" --- "<<norm.z<<std::endl;
         if(left == true)
            skeleton.curr_pose[N-1].p.x += 0.1;
         else
            skeleton.curr_pose[N-1].p.x -= 0.1;
        left = !left;
        updated = 10;
      } else 
           updated -= 1;

 
    for(size_t k=0; k<N; ++k)
    {
           if(k != skeleton.plead){
               //std::cout<<" ANGLE = "<<angle * 180.0 / PI<<std::endl;
              //skeleton.curr_pose[k].p = skeleton.rest_pose[k].p + t;
              auto tmp = skeleton.curr_pose[k].p;
              //skeleton.curr_pose[k].p = oldp;
              auto dist = oldp - tmp;
              skeleton.curr_pose[k].p = dist*0.1 + tmp; 
              oldp = tmp;
              //skeleton.curr_pose[k].p = q.apply(skeleton.curr_pose[k].p);// + t;
              //skeleton.curr_pose[k].p.x = skeleton.rest_pose[k].p.z*sign + t.x;
              //skeleton.curr_pose[k].p.z = 0.f;
              //if(skeleton.curr_pose[k].p.x != skeleton.curr_pose[k-1].p.x){
                 //quaternion q= quaternion::axis_angle(skeleton.curr_pose[skeleton.plead].p , PI/2);
                 //q = normalize(q);
                 //skeleton.curr_pose[k].p = q.apply(skeleton.curr_pose[k].p);
              //}
             //std::cout<<"k ="<<k<<" " <<dist.x<<" --- "<<dist.y<<" --- "<<dist.z<<std::endl;
           }
           //normalise(skeleton.curr_pose[k].p, 2.f);
        //std::cout<<skeleton.curr_pose[k].p.x<<" --- "<<skeleton.curr_pose[k].p.y<<" --- "<<skeleton.curr_pose[k].p.z<<" head= "<<skeleton.plead<<std::endl;
    }

}*/
void fish_structure::update(){
    const size_t N = skeleton.curr_pose.size();
    skeleton.curr_pose[skeleton.plead].p = p;

    if(updated == 0){
       skeleton.curr_pose[N-1].p.x += (0.1 * int(left) - 0.1* (1-int(left))) ;
       left = !left;
       updated = 10;
     } else 
         updated -= 1;
    for(size_t k = 0; k < N; ++k) {
       auto tmp = skeleton.curr_pose[k].p;
       if(k < N - 1){
         tmp = skeleton.curr_pose[k + 1].p;
         auto head = skeleton.curr_pose[k].p;
         auto tail = skeleton.curr_pose[k + 1].p;
         auto dist = (head - tail);
         auto ndist = norm(dist);
         auto rest_dist = norm(skeleton.rest_pose[k + 1].p) - norm(skeleton.rest_pose[k].p);
         auto corr = ndist - rest_dist;
         auto corrdist = normalize(dist) * corr;
         skeleton.curr_pose[k + 1].p = corrdist + tail;
       }
    }
    size_t closest = skeleton.curr_pose[skeleton.plead].closest;
    const vec3 goal = skinning.deformed.position[closest];
    
    skinning.deformed.position[skeleton.plead] += (skeleton.curr_pose[0].p - goal);
    skinning.deformed.position[closest] += (skeleton.curr_pose[0].p - goal);

    vec3 vecorig = (skeleton.curr_pose[skeleton.plead].p - skeleton.curr_pose[1].p);
    vec3 vecmesh = (skinning.deformed.position[closest] - skinning.deformed.position[skeleton.plead]);
    mat3 rot_orig = rotation_between_vector(vecmesh, vecorig);

    for(size_t j = 0; j < skinning.deformed.position.size(); j++) {
        if(closest != j && j != skeleton.plead)
           skinning.deformed.position[j] += (skeleton.curr_pose[0].p - goal);
        skinning.deformed.position[j] = rot_orig * skinning.deformed.position[j];
    }
    /*skinning.deformed.normal.clear();
    skinning.deformed.fill_empty_fields();*/

}
void fish_structure::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene)
{
    character_visual.update_position(skinning.deformed.position);
    character_visual.update_normal(skinning.deformed.normal);
    update();

    if(show_skeleton)
      display_skeleton( shaders, scene, segment_drawer);
    if(show_mesh)
       draw(character_visual, scene.camera, shaders["wireframe_quads"]);
    glPolygonOffset( 1.0, 1.0 );
    if(show_texture){
       GLuint const texture_id = character_visual.texture_id; 
       draw(character_visual, scene.camera, character_visual.shader, texture_id);
    }
}

