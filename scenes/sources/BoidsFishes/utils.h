
#ifndef UTILS_H


#define UTILS_H

#include "scenes/base/base.hpp"

// Connectivity information of a joint in the hierarchy
//  Store parent index, and the current name
struct joint_connectivity
{
    int parent;
    std::string name;
};

// 3D Geometry of a joint (position p, and rotation r)
struct joint_geometry
{
    vcl::vec3 p;
    vcl::buffer<int> v;
    vcl::vec3 moved;
    int closest;
    joint_geometry(const vcl::vec3& pos):p(pos),closest(-1){}
};

// Structure storing skeleton data to perform skinning afterward
struct skeleton_structure
{
    vcl::buffer<joint_connectivity> connectivity;           // Connectivity of the skeleton
    vcl::buffer<joint_geometry>     rest_pose;              // Skeleton of the rest pose expressed in local coordinates
    vcl::buffer<joint_geometry>     curr_pose;              // Skeleton of the current pose expressed in local coordinates
    size_t plead;
    vcl::vec3 middle;
    vcl::vec3 middlecurr;
};

// Storage structure to perform skinning deformation of a surface
struct skinning_structure
{
    vcl::buffer<vcl::vec3> rest_pose;                         // 3D position of the mesh in rest pose
    vcl::buffer<vcl::vec3> rest_pose_normal;                  // 3D normals of the mesh in rest pose
    vcl::mesh deformed;                                       // Deformed mesh
};


vcl::buffer<joint_geometry> read_skeleton_geometry(const std::string& filename);
int read_skeleton_connectivity(const std::string& filename,vcl::buffer<joint_connectivity>& skeleton);

void load_character_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::mesh_drawable& shape_visual, vcl::vec3& barmesh);

vcl::vec3 rotate_through_line(const vcl::vec3& v1, const vcl::vec3& v2, const vcl::vec3& v3, float angle);
vcl::vec3 closestPointOnLine(const vcl::vec3& a, const vcl::vec3& b, const vcl::vec3& p);
vcl::vec3 rotate_through_vector(const vcl::vec3& v1, const vcl::vec3& v2, float angle);
vcl::vec3 rotate_through_angles(const vcl::vec3& v1, float anglex, float angley,float anglez);
vcl::mat3 rotation_between_vector(const vcl::vec3& a, const vcl::vec3& b);

#endif

