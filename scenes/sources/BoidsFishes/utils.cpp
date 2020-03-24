
#include "utils.h"

#include <fstream>
#include <sstream>

using namespace vcl;

mat3 rotation_between_vector(const vec3& a, const vec3& b)
{
    const vec3 u0 = normalize(a);
    const vec3 u1 = normalize(b);

    if( norm(u0-u1)<1e-4f )
        return mat3::identity();
    if( norm(u0+u1)<1e-4f )
        return -mat3::identity();

    float d = dot(u0,u1);
    d = clamp(d, -1.0f, 1.0f);
    const float angle = std::acos( d );

    if (angle < 1e-3f)
        return mat3::identity();
    vec3 axis = normalize(cross(u0, u1));

    return rotation_from_axis_angle_mat3(axis,angle);
}

void scale( vcl::vec3&v, const vcl::vec3& max, const vcl::vec3& min, const vcl::vec3& maxnew, const vcl::vec3& minnew){
     auto divx = (max.x - min.x) > 0 ? (max.x - min.x) : 1;
     auto divy = (max.y - min.y) > 0 ? (max.y - min.y) : 1;
     auto divz = (max.z - min.z) > 0 ? (max.z - min.z) : 1;
     v.x = minnew.x + ((v.x - min.x)/divx) * (maxnew.x-minnew.x);
     v.y =  minnew.y + ((v.y - min.y)/ divy) * (maxnew.y-minnew.y);
     v.z = minnew.z + ((v.z - min.z)/ divz) * (maxnew.z-minnew.z);

}
vcl::vec3 closestPointOnLine(const vcl::vec3& a, const vcl::vec3& b, const vcl::vec3& p){
    auto ap = p-a;
    auto ab = b-a;
    auto result = a + (dot(ap,ab)/dot(ab,ab)) * ab;
    return result;
}
vec3 rotate_through_line(const vec3& v1, const vec3& v2, const vec3& v3, float angle){
     auto coef = normalize((v2 - v1)*0.2 + v1);
     float a = coef.x;
     float b = coef.y;
     float c = coef.z;
     float d = std::sqrt(b*b+c*c);
     mat4 t(1.f,0.,0.,0., 0.,1.,.0,.0,0.,0.,1.,0.,-v3.x,-v3.y,-v3.z,1.);
     mat4 tinv(1.f,0.,0.,0., 0.,1.,.0,.0,0.,0.,1.,0.,v3.x,v3.y,v3.z,1.);
     mat4 Rx(1.f,0.,0., 0.,   0.,c/d,b/d,.0,  0,-b/d,c/d,0,  0.,0,0.,1.);
     mat4 Ry(d,0.,a, 0.,   0.,1,0,.0,  -1,0,d,0,  0.,0,0.,1.);
     mat4 Rz(std::cos(angle),std::sin(angle),0., 0.,   -std::sin(angle),std::cos(angle),0,.0,  0,0,1,0,  0.,0,0.,1.);
     auto tr = t*Rx*Ry*Rz*tinv;
     vec4 v(v3.x, v3.y, v3.z,1);
     vec4 tv = tr * v;
     return vec3(tv.x, tv.y,tv.z);
}

vec3 rotate_through_vector(const vec3& v1, const vec3& v2, float angle){

    mat4 t(1.f,0.,0.,0., 0.,1.,.0,.0,0.,0.,1.,0.,-v1.x,-v1.y,-v1.z,1.);
    mat4 tinv(1.f,0.,0.,0., 0.,1.,.0,.0,0.,0.,1.,0.,v1.x,v1.y,v1.z,1.);
    mat4 Rx(1.f,0.,0., 0.,   0.,std::cos(angle),std::sin(angle),.0,  0,-std::sin(angle),std::cos(angle),0,  0.,0,0.,1.);
    mat4 Ry(std::cos(angle),0.,-std::sin(angle),0.,   0.,1,0,.0,  std::sin(angle),0,std::cos(angle),0,  0.,0,0.,1.);
    mat4 Rz(std::cos(angle),std::sin(angle),0., 0.,   -std::sin(angle),std::cos(angle),0,.0,  0,0,1,0,  0.,0,0.,1.);
    vec4 v(v2.x, v2.y, v2.z,1);
    v = t*v;
    v = Rx *v;
    v = Ry*v;
    v = Rz * v;
    v = tinv *v;
    return vec3(v.x, v.y,v.z);
}
vec3 rotate_through_angles(const vec3& v1, float anglex, float angley,float anglez){

     mat4 Rx(1.f,0.,0., 0.,   0.,std::cos(anglex),std::sin(anglex),.0,  0,-std::sin(anglex),std::cos(anglex),0,  0.,0,0.,1.);
     mat4 Ry(std::cos(angley),0.,-std::sin(angley),0.,   0.,1,0,.0,  std::sin(angley),0,std::cos(angley),0,  0.,0,0.,1.);
     mat4 Rz(std::cos(anglez),std::sin(anglez),0., 0.,   -std::sin(anglez),std::cos(anglez),0,.0,  0,0,1,0,  0.,0,0.,1.);
     vec4 v(v1.x, v1.y, v1.z,1);
     v = Rx *v;
     v = Ry*v;
     v = Rz * v;
     return vec3(v.x, v.y,v.z);
}

void load_character_data(skeleton_structure& skeleton, skinning_structure& skinning, mesh_drawable& shape_visual, vec3& barmesh)
{

    // Load skeleton + mesh + texture

    buffer<buffer<int> > vertex_correspondance;
    mesh character = mesh_load_file_obj("assets/mesh.obj", vertex_correspondance);

    int head = read_skeleton_connectivity("assets/fish_connectivity",skeleton.connectivity);
    skeleton.rest_pose      = read_skeleton_geometry("assets/fish_geometry");
    skeleton.plead = head;
    skeleton.middle = skeleton.rest_pose[head].p + (skeleton.rest_pose[skeleton.rest_pose.size()-1].p*0.5 - skeleton.rest_pose[head].p*0.5);
    skeleton.middlecurr = skeleton.middle;

    GLuint texture_id = create_texture_gpu(image_load_png("assets/13004_Bicolor_Blenny_v1_diff.png"));

    auto max = skeleton.rest_pose[skeleton.rest_pose.size()-1].p;
    max.x = max.y = 0.06;
    vec3 minm = {10000.f, 10000.f, 10000.f};
    barmesh = {0.f,0.f,0.f};
    for(vec3& p : character.position){
       p*=0.05;
       barmesh += p;
       minm = {std::min(minm.x,p.x),std::min(minm.y,p.y),std::min(minm.z,p.z)};
    }
    barmesh /= character.position.size();
    auto t =  barmesh - skeleton.middle;
    skeleton.curr_pose = skeleton.rest_pose;
    
    skeleton.middlecurr = barmesh;
    skeleton.curr_pose[head].p += t;
    skeleton.curr_pose[head].p.y = minm.y;
    skeleton.curr_pose[head].p.z = barmesh.z;
    skeleton.rest_pose[head].moved = skeleton.curr_pose[head].p;
    skeleton.curr_pose[head].moved = skeleton.curr_pose[head].p;
    for(size_t i = 1; i < skeleton.rest_pose.size(); i++ ){
       skeleton.curr_pose[i].p += t;
       skeleton.curr_pose[i].p.z = skeleton.curr_pose[i-1].p.z;
       auto diff = skeleton.rest_pose[i].p.z - skeleton.rest_pose[i-1].p.z;
       skeleton.curr_pose[i].p.y = skeleton.curr_pose[i-1].p.y + diff;
       skeleton.rest_pose[i].moved = skeleton.curr_pose[i].p;
       skeleton.curr_pose[i].moved = skeleton.curr_pose[i].p;
    }
    vec4 mind = {100000.f, 100000, 100000, 100000};
    for(size_t i = 0; i < character.position.size(); i++){
       const vec3& p = character.position[i];
       if(p.y>= skeleton.curr_pose[0].p.y && p.y <= skeleton.curr_pose[1].p.y ){
           skeleton.rest_pose[0].v.push_back(i);
           skeleton.curr_pose[0].v.push_back(i);
           auto diff = norm(skeleton.curr_pose[0].p - p);
           if(diff < mind[0]){
              skeleton.rest_pose[0].closest = i;
              skeleton.curr_pose[0].closest = i;
               mind[0] = std::min(diff, mind[0]);
           }
           continue;
       } 
       if(p.y>= skeleton.curr_pose[1].p.y && p.y <= skeleton.curr_pose[2].p.y ){
           skeleton.rest_pose[1].v.push_back(i);
           skeleton.curr_pose[1].v.push_back(i);
           auto diff = norm(skeleton.curr_pose[1].p - p);
           if(diff < mind[1]){
              skeleton.rest_pose[1].closest = i;
              skeleton.curr_pose[1].closest = i;
               mind[1] = std::min(diff, mind[1]);
           }
          continue;
       }
       skeleton.curr_pose[3].v.push_back(i);
       skeleton.rest_pose[3].v.push_back(i);
       auto diff = norm(skeleton.curr_pose[3].p - p);
       if(diff < mind[3]){
          skeleton.rest_pose[3].closest = i;
          skeleton.curr_pose[3].closest = i;
          mind[3] = std::min(diff, mind[3]);
       }
        
    }

    shape_visual.clear();
    shape_visual = mesh_drawable(character);
    shape_visual.shader = create_shader_program("assets/shader.vert.glsl", "assets/shader.frag.glsl");
    shape_visual.uniform.shading.specular = 0.1f;
    shape_visual.uniform.shading.specular_exponent = 8;

    character.fill_empty_fields();
    skinning.rest_pose = character.position;
    skinning.rest_pose_normal = character.normal;
    skinning.deformed  = character;

    shape_visual.texture_id = texture_id;

}


int read_skeleton_connectivity(const std::string& filename,  buffer<joint_connectivity>& skeleton)
{
    skeleton.clear();

    std::ifstream fid(filename);
    int head = -1;
    while(fid.good())
    {
        std::string line;
        std::getline(fid, line);

        if(fid.good())
        {
            std::stringstream sstream(line);

            int k;
            int parent;
            std::string name;

            sstream >> k >> parent >> name;

            if(name == "head"){
              head = k;
            }

            skeleton.push_back({parent,name});
        }
    }

    fid.close();

    return head;
}

buffer<joint_geometry> read_skeleton_geometry(const std::string& filename)
{
    buffer<joint_geometry> skeleton;
    std::ifstream fid(filename);
    while(fid.good())
    {
        std::string line;
        std::getline(fid, line);

        if(fid.good())
        {
            std::stringstream sstream(line);
            vec3 p;
            sstream >> p.x >> p.y >> p.z;
            skeleton.push_back({p});
        }
    }

    fid.close();

    return skeleton;
}

