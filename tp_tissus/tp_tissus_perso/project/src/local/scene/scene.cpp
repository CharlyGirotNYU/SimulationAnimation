

#include <GL/glew.h>

#include "scene.hpp"
#include "../../lib/opengl/glutils.hpp"

#include "../../lib/perlin/perlin.hpp"
#include "../../lib/interface/camera_matrices.hpp"

#include "../interface/myWidgetGL.hpp"
#include "../../lib/mesh/mesh_io.hpp"
#include "../../lib/common/error_handling.hpp"


#include <cmath>
#include <string>
#include <sstream>



using namespace cpe;



static cpe::mesh build_ground(float const L,float const h);
static cpe::mesh build_sphere(float radius,vec3 center);
static cpe::mesh build_ellipse(float rad1, float rad2, vec3 center);
static cpe::mesh build_cylinder(float radius, float length,
                                unsigned c_steps, unsigned l_steps);


void scene::load_scene()
{
    time_integration.restart();
    divergence=false;

    //*****************************************//
    // Preload default structure               //
    //*****************************************//
    texture_default = load_texture_file("data/white.jpg");
    shader_mesh     = read_shader("shaders/shader_mesh.vert",
                                  "shaders/shader_mesh.frag");

    texture_cloth = load_texture_file("data/cloth.png");
    texture_ground = load_texture_file("data/wood_texture.png");
    texture_cat = load_texture_file("data/cat.png");

    //*****************************************//
    // Build ground
    //*****************************************//
    mesh_ground = build_ground(1.0f , -1.101f);
    mesh_ground.fill_empty_field_by_default();
    mesh_ground_opengl.fill_vbo(mesh_ground);

    //*****************************************//
    // Sphere
    //*****************************************//
    centre_sphere = {0.0f,-0.0f,-0.8f};
    radius_sphere=0.16f;
    mesh_sphere = build_sphere(radius_sphere , centre_sphere);
    mesh_sphere.fill_empty_field_by_default();
    //mesh_sphere.transform_apply_translation(vec3(0.5f,0.5f,0.5f));
    mesh_sphere_opengl.fill_vbo(mesh_sphere);

    //*****************************************//
    // Build cloth
    //*****************************************//
    mesh_cloth.set_plane_xy_unit(30,30,-0.0f,-0.5f);

    mesh_cloth.set_dt() = 0.2f;
    mesh_cloth.set_K_structural() = 5.0f;
    mesh_cloth.set_K_shearing() = 5.0f;
    mesh_cloth.set_K_bending() = 5.0f;
    mesh_cloth.set_wind() = 0.01f;
    //mesh_cloth.vertex(0,0)-=vec3(0.2f,0.0f,0.0f);
    //mesh_cloth.vertex(0,29)-= vec3(-0.2f,0.0f,0.0f);
    mesh_cloth.fill_empty_field_by_default();
    mesh_cloth_opengl.fill_vbo(mesh_cloth);

    //***************************************//
    //Build Cat
    //***************************************//
    mesh_cat = load_mesh_file("data/cat.obj");
    mesh_cat.transform_apply_auto_scale_and_center();
    mesh_cat.transform_apply_rotation(vec3(1,0,0),90*M_PI/180);
    mesh_cat.transform_apply_translation(vec3(0.6f,0.3f,-0.6f));
    mesh_cat_opengl.fill_vbo(mesh_cat);

    //***************************************//
    // Build cat hull
    //**************************************//
    centres.push_back(vec3(0.6f,0.5f,-0.55f));
    centres.push_back(vec3(0.6f,0.8f,-0.5f));
    centres.push_back(vec3(0.6f,-0.12f,-0.4f));//sphere
    radius.push_back(0.1f);
    radius.push_back(0.025f);
    radius.push_back(0.07f);
    length.push_back(0.5);
    length.push_back(0.5);
    length.push_back(0); //sphere

    hull_1 = build_cylinder(radius.at(0),length.at(0),30,20);
    hull_1.transform_apply_rotation(vec3(1,0,0),90*M_PI/180);
    hull_1.transform_apply_translation(centres.at(0));
    centres_bis.push_back(centres.at(0)+ vec3(0,-1,0)*length.at(0));
    hull_1_opengl.fill_vbo(hull_1);

    hull_2 = build_cylinder(radius.at(1),length.at(1),30,20);
    hull_2.transform_apply_rotation(vec3(1,0,0),90*M_PI/180);
    hull_2.transform_apply_translation(centres.at(1));
    centres_bis.push_back(centres.at(1)+ vec3(0,-1,0)*length.at(1));


    hull_3 = build_sphere(radius.at(2),centres.at(2));

    //hull_1_opengl.fill_vbo(hull_1); //debug
}

//2s 200000000

void scene::draw_scene()
{

    setup_shader_mesh(shader_mesh);

    // draw the ground
    glBindTexture(GL_TEXTURE_2D,texture_ground);                                                       PRINT_OPENGL_ERROR();
    mesh_ground_opengl.draw();
    // draw the sphere
    glBindTexture(GL_TEXTURE_2D,texture_default);                                                      PRINT_OPENGL_ERROR();
    mesh_sphere_opengl.draw();
    // Draw the cat
    glBindTexture(GL_TEXTURE_2D, texture_cat);
    mesh_cat_opengl.draw();
    glBindTexture(GL_TEXTURE_2D,texture_default);
    //hull_cat_opengl.draw();


    //try numerical integration (stop computation if divergence)
    try
    {
        if(divergence==false && time_integration.elapsed() > 5)
        {
            // compute-force / time integration
            mesh_cloth.update_force();
            mesh_cloth.integration_step();
            mesh_cloth.update_shpere_collision(mesh_sphere,centre_sphere,radius_sphere); //sphere original
            mesh_cloth.update_plan_collision(mesh_ground);
            mesh_cloth.update_cylinder_collision(hull_1,radius.at(0),centres.at(0),centres_bis.at(0));
            mesh_cloth.update_cylinder_collision(hull_2,radius.at(1),centres.at(1),centres_bis.at(1));
            mesh_cloth.update_shpere_collision(hull_3,centres.at(2),radius.at(2));

            // re-compute normals
            mesh_cloth.fill_normal();

            // update opengl container
            mesh_cloth_opengl.update_vbo_vertex(mesh_cloth);
            mesh_cloth_opengl.update_vbo_normal(mesh_cloth);

            time_integration.restart();
        }
    }
    catch(exception_divergence const& e)
    {
        if(divergence==false)
        {
            std::cout<<"\n\nDivergence, time integration stoped"<<std::endl;
            divergence = true;
        }
    }

    //draw the cloth
    glBindTexture(GL_TEXTURE_2D,texture_cloth);                                                       PRINT_OPENGL_ERROR();
    mesh_cloth_opengl.draw();


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

void scene::set_K_structural(int v)
{
    mesh_cloth.set_K_structural() = v; //K between 0 and 10
}
void scene::set_K_shearing(int v)
{
    mesh_cloth.set_K_shearing() = v; //K between 0 and 10
}
void scene::set_K_bending(int v)
{
    mesh_cloth.set_K_bending() = v; //K between 0 and 10
}

void scene::set_wind(int v)
{
    mesh_cloth.set_wind() = (float)v/100.0f; //wind between 0 and 0.1
}

void scene::set_dt(int v)
{
    mesh_cloth.set_dt() = (float)v/20.0f; //dt between 0 and 0.5
}

void scene::set_attache(int v)
{
    mesh_cloth.set_attache() = v;
}

static cpe::mesh build_ground(float const L,float const h)
{
    mesh m;
    m.add_vertex({ -L/2.0f, -L , h });
    m.add_vertex({ -L/2.0f,  L , h });
    m.add_vertex({ 1.5f*L,  L , h });
    m.add_vertex({ 1.5f*L, -L , h });

    m.add_texture_coord({  0.0f ,  0.0f });
    m.add_texture_coord({  0.0f ,  1.0f });
    m.add_texture_coord({  1.0f ,  1.0f });
    m.add_texture_coord({  1.0f ,  0.0f });

    m.add_triangle_index({0,2,1});
    m.add_triangle_index({0,3,2});


    return m;
}

static cpe::mesh build_sphere(float radius,vec3 center)
{
    mesh m;
    m.load("data/sphere.off");
    m.transform_apply_scale(radius);
    m.transform_apply_translation(center);
    return m;
}


static cpe::mesh build_ellipse(float rad1, float rad2, vec3 center)
{
    mesh m;
    m.load(("data/box.obj"));
    m.transform_apply_auto_scale_and_center();
    m.transform_apply_rotation(vec3(1,0,0),90*M_PI/180);
    return m;
}


static cpe::mesh build_cylinder(float radius, float length,
                            unsigned c_steps, unsigned l_steps)
{
    mesh m;
    // Creating the vertices
    for (unsigned i = 0; i <= l_steps; i++)
    {
        float z = length * i / l_steps;
        for (unsigned j = 0; j < c_steps; j++)
        {
            float angle = M_PI * 2.f * j / c_steps;
            float x = radius * cos(angle);
            float y = radius * sin(angle);
            m.add_vertex(vec3(x,y, z));
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
            m.add_triangle_index({i0,i1,i2});
            m.add_triangle_index({i1,i3,i2});
        }
    }
    m.fill_empty_field_by_default();
    return m;
}
