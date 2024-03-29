

#include <GL/glew.h>

#include "scene.hpp"
#include "../../lib/opengl/glutils.hpp"

#include "../../lib/perlin/perlin.hpp"
#include "../../lib/interface/camera_matrices.hpp"

#include "../interface/myWidgetGL.hpp"

#include <cmath>


#include <string>
#include <sstream>
#include "../../lib/mesh/mesh_io.hpp"


using namespace cpe;


static cpe::mesh build_ground(float const L,float const h)
{
    mesh m;
    m.add_vertex(vec3(-L, h,-L));
    m.add_vertex(vec3(-L, h, L));
    m.add_vertex(vec3( L, h, L));
    m.add_vertex(vec3( L, h,-L));

    m.add_triangle_index({0,2,1});
    m.add_triangle_index({0,3,2});

    m.fill_color(vec3(0.8,0.9,0.8));

    return m;
}


void scene::create_cylinder(float radius, float length,
                            unsigned c_steps, unsigned l_steps)
{
    // Weight parameters for the skinning (à décommenter à partir de la question 15)
     vertex_weight_parameter weight_parameter;
     weight_parameter[0].joint_id = 0;
     weight_parameter[1].joint_id = 1;
    // Creating the vertices
    for (unsigned i = 0; i <= l_steps; i++)
    {
        float z = length * i / l_steps;
        for (unsigned j = 0; j < c_steps; j++)
        {
            float angle = M_PI * 2.f * j / c_steps;
            float x = radius * cos(angle);
            float y = radius * sin(angle);
            mesh_cylinder.add_vertex(vec3(x,y, z));
            // skinning weights (à décommenter à partir de la question 15)
            // À modifier lors de la question 19

             if (z / length < 0.5)
             {
               weight_parameter[0].weight = 1.0f;
               weight_parameter[1].weight = 0.0f;
             } else {
               weight_parameter[0].weight = 1-z/length;
               weight_parameter[1].weight = z/length;
             }
             mesh_cylinder.add_vertex_weight(weight_parameter);
        }
    }

    // And now for the connectivity!
    for (unsigned i = 0; i < l_steps; i++)
    {
        for (unsigned j = 0; j < c_steps; j++)
        {
            int i0 = (i + 0) * c_steps + (j + 0) % c_steps;
            int i1 = (i + 0) * c_steps + (j + 1) % c_steps;
            int i2 = (i + 1) * c_steps + (j + 0) % c_steps;
            int i3 = (i + 1) * c_steps + (j + 1) % c_steps;
            mesh_cylinder.add_triangle_index({i0,i1,i2});
            mesh_cylinder.add_triangle_index({i1,i3,i2});
        }
    }
    mesh_cylinder.fill_empty_field_by_default();
    mesh_cylinder_opengl.fill_vbo(mesh_cylinder);

    // The skeleton parent structure
    // Ajoutez les indices des parents dans la variable sk_cylinder_parent_id (question 7)
    sk_cylinder_parent_id.push_back(-1);
    sk_cylinder_parent_id.push_back(0);
    sk_cylinder_parent_id.push_back(1);

    // Skeleton of the bind pose for the cylinder
    // Ajoutez les définitions des repères locaux (joint = position + orientation)
    // dans la variable sk_cylinder_bind_pose (question 7)
    quaternion q(0,0,0,1);
    //quaternon q2();
    skeleton_joint joint0(vec3(0,0,0),q);
    skeleton_joint joint1(vec3(0,0,length/2),q);
    skeleton_joint joint2(vec3(0,0,length/2),q); // Attention c'est aussi en relatif par rapport au joint parent
    sk_cylinder_bind_pose.push_back(joint0);
    sk_cylinder_bind_pose.push_back(joint1);
    sk_cylinder_bind_pose.push_back(joint2);
    sk_cylinder_animation.push_back(sk_cylinder_bind_pose);




    // Animation of the skeleton for the cylinder
    // Créez une animation en remplissant la variable sk_cylinder_animation (question 12)
    // Chacun de ses éléments est une pose complète (= des valeurs pour chacun joints)
    //
    for(unsigned int i=1; i<4; i++)
    {
        float teta = M_PI*i*30.0f/180.0f;
        quaternion q1;
        q1.set_axis_angle(vec3(1,0,0),teta);
        skeleton_joint joint1(vec3(0,0,length/2),q1);
        skeleton_joint joint2(vec3(0,0,length/2),q); // q : pas de rotation par rapport au joint 1
        skeleton_geometry pose;
        pose.push_back(joint0);
        pose.push_back(joint1);
        pose.push_back(joint2);
        std::cout << pose << std::endl;
        sk_cylinder_animation.push_back(pose);
    }
}


void scene::load_scene()
{
    time_cylinder.restart();
    frame_cylinder_current = 0;

    time_cat.restart();
    frame_cat_current = 0;

    //*****************************************//
    // Preload default structure               //
    //*****************************************//
    texture_default = load_texture_file("data/white.jpg");
    shader_mesh     = read_shader("shaders/shader_mesh.vert",
                                  "shaders/shader_mesh.frag");           PRINT_OPENGL_ERROR();
    shader_skeleton = read_shader("shaders/shader_skeleton.vert",
                                  "shaders/shader_skeleton.frag");       PRINT_OPENGL_ERROR();


    //*****************************************//
    // Build ground
    //*****************************************//
    mesh_ground = build_ground(100.0f , -25.0f);
    mesh_ground.fill_empty_field_by_default();
    mesh_ground_opengl.fill_vbo(mesh_ground);

    //*****************************************//
    // Build cylinder
    //*****************************************//
    create_cylinder(); // use default values


    //*****************************************//
    // Load cat
    //*****************************************//
    mesh_cat.load("data/cat.obj");
    texture_cat=load_texture_file("data/cat.png");
    mesh_cat.fill_empty_field_by_default();
    mesh_cat_opengl.fill_vbo(mesh_cat);

    sk_cat_bind_pose.load("data/cat_bind_pose.skeleton");
    sk_cat_parent_id.load("data/cat_bind_pose.skeleton");
    sk_cat_animation.load("data/cat.animations",sk_cat_parent_id.size());
}

void scene::draw_scene()
{
    // Gestion des timers (question 14)
     int const time_cylinder_max = 2000;
     if( time_cylinder.elapsed()>time_cylinder_max )
     {
         time_cylinder.restart();
         frame_cylinder_current = (frame_cylinder_current+1)%sk_cylinder_animation.size();
     }
     frame_cylinder_alpha = time_cylinder.elapsed()/static_cast<float>(time_cylinder_max);

//     int const time_cat_max = 60;
//     if( time_cat.elapsed()>time_cat_max )
//     {
//         time_cat.restart();
//         frame_cat_current = (frame_cat_current+1)%sk_cat_animation.size();
//     }
//     frame_cat_alpha = time_cat.elapsed()/static_cast<float>(time_cat_max);

    // Mise en place du shader pour les squelettes
    setup_shader_skeleton(shader_skeleton);

    skeleton_geometry const sk_cylinder_global = local_to_global(sk_cylinder_animation[frame_cylinder_current],sk_cylinder_parent_id);
    std::vector<vec3> const sk_cylinder_bones = extract_bones(sk_cylinder_global,sk_cylinder_parent_id);
    skeleton_geometry const sk_cylinder_inverse_bind_pose = inversed(sk_cylinder_bind_pose); //Bj-1
    skeleton_geometry const sk_cylinder_binded = multiply(sk_cylinder_global,sk_cylinder_inverse_bind_pose); //Sj

    mesh_cylinder.apply_skinning(sk_cylinder_binded);

    draw_skeleton(sk_cylinder_bones);

    //Here we can draw skeletons as 3D segments
    // question 11

    setup_shader_mesh(shader_mesh);

    mesh_ground_opengl.draw();

    // Draw the cylinder
    // ...

    mesh_cylinder.fill_normal();
    mesh_cylinder_opengl.update_vbo_vertex(mesh_cylinder);
    mesh_cylinder_opengl.update_vbo_normal(mesh_cylinder);
    mesh_cylinder_opengl.draw();


    // Draw the cat
    glBindTexture(GL_TEXTURE_2D,texture_cat);
    // ...
    //mesh_cat_opengl.draw();

}


void scene::setup_shader_mesh(GLuint const shader_id)
{
    //Setup uniform parameters
    glUseProgram(shader_id);                                                                           PRINT_OPENGL_ERROR();

    //Get cameras parameters (modelview,projection,normal).
    camera_matrices const& cam=pwidget->camera();

    //Set Uniform data to GPU
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_modelview"),1,false,cam.modelview.pointer());     PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_projection"),1,false,cam.projection.pointer());   PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"normal_matrix"),1,false,cam.normal.pointer());           PRINT_OPENGL_ERROR();

    //load white texture
    glBindTexture(GL_TEXTURE_2D,texture_default);                                                      PRINT_OPENGL_ERROR();
    glLineWidth(1.0f);                                                                                 PRINT_OPENGL_ERROR();

}

void scene::setup_shader_skeleton(GLuint const shader_id)
{
    //Setup uniform parameters
    glUseProgram(shader_id);                                                                           PRINT_OPENGL_ERROR();

    //Get cameras parameters (modelview,projection,normal).
    camera_matrices const& cam=pwidget->camera();

    //Set Uniform data to GPU
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_modelview"),1,false,cam.modelview.pointer());     PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_projection"),1,false,cam.projection.pointer());   PRINT_OPENGL_ERROR();
    glUniform3f(get_uni_loc(shader_id,"color") , 0.0f,0.0f,0.0f);                                      PRINT_OPENGL_ERROR();

    //size of the lines
    glLineWidth(3.0f);                                                                                 PRINT_OPENGL_ERROR();
}

void scene::draw_skeleton(std::vector<vec3> const& positions) const
{
    // Create temporary a VBO to store data
    GLuint vbo_skeleton=0;
    glGenBuffers(1,&vbo_skeleton);                                                                     PRINT_OPENGL_ERROR();
    glBindBuffer(GL_ARRAY_BUFFER,vbo_skeleton);                                                        PRINT_OPENGL_ERROR();

    // Update data on the GPU
    glBufferData(GL_ARRAY_BUFFER , sizeof(vec3)*positions.size() , &positions[0] , GL_STATIC_DRAW);    PRINT_OPENGL_ERROR();

    // Draw data
    glEnableClientState(GL_VERTEX_ARRAY);                                                              PRINT_OPENGL_ERROR();
    glVertexPointer(3, GL_FLOAT, 0, 0);                                                                PRINT_OPENGL_ERROR();
    glDrawArrays(GL_LINES,0,positions.size());                                                         PRINT_OPENGL_ERROR();

    // Delete temporary VBO
    glDeleteBuffers(1,&vbo_skeleton);                                                                  PRINT_OPENGL_ERROR();
}

scene::scene()
    :shader_mesh(0)
{}


GLuint scene::load_texture_file(std::string const& filename)
{
    return pwidget->load_texture_file(filename);
}

void scene::set_widget(myWidgetGL* widget_param)
{
    pwidget=widget_param;
}


