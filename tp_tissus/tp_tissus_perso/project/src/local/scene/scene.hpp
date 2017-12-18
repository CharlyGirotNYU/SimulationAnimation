
/** TP 5ETI - CPE Lyon - 2015/2016 */

#pragma once

#ifndef SCENE_HPP
#define SCENE_HPP

#include <GL/gl.h>
#include <GL/glew.h>

#include <QTime>

#include "../../lib/3d/mat3.hpp"
#include "../../lib/3d/vec3.hpp"
#include "../../lib/mesh/mesh.hpp"
#include "../../lib/opengl/mesh_opengl.hpp"
#include "../../lib/interface/camera_matrices.hpp"
#include "../../cloth/mesh_parametric_cloth.hpp"


#include <vector>


class myWidgetGL;

class scene
{
public:

    scene();



    /**  Method called only once at the beginning (load off files ...) */
    void load_scene();

    /**  Method called at every frame */
    void draw_scene();

    /** Set the pointer to the parent Widget */
    void set_widget(myWidgetGL* widget_param);

    /** Set Delta T called by the ui */
    void set_dt(int v);
    /** Set wind power value called by the ui*/
    void set_wind(int v);
    /** Set K values called by the ui */
    void set_K_structural(int v);
    void set_K_shearing(int v);
    void set_K_bending(int v);

private:


    /** Load a texture from a given file and returns its id */
    GLuint load_texture_file(std::string const& filename);

    /** Access to the parent object */
    myWidgetGL* pwidget;

    /** Default id for the texture (white texture) */
    GLuint texture_default;


    /** Ground mesh */
    cpe::mesh mesh_ground;
    /** Ground mesh for OpenGL drawing */
    cpe::mesh_opengl mesh_ground_opengl;


    /** Cloth mesh */
    cpe::mesh_parametric_cloth mesh_cloth;
    /** Cloth mesh for OpenGL drawing */
    cpe::mesh_opengl mesh_cloth_opengl;

    /** Parameter of a sphere */
    float radius_sphere = 0.198f ;
    //cpe::vec3 centre = {0.5f,0.5f,-0.5f};
    //cpe::vec3 centre = {0.3f,-0.0f,-0.3f};
    cpe::vec3 centre_sphere = {1.0f,1.0f,1.0f};
    /** Mesh of a sphere */
    cpe::mesh mesh_sphere;
    /** OpenGL VBO for the sphere */
    cpe::mesh_opengl mesh_sphere_opengl;

    /** Cat mesh */
    cpe::mesh mesh_cat;
    /** Cat mesh for openGl drawing */
    cpe::mesh_opengl mesh_cat_opengl;

    /** Cat hull */
    cpe::mesh hull_cat, hull_2, hull_3;
    /** Cat hull for OpenGL drawing/Debug */
    cpe::mesh_opengl hull_cat_opengl;

    /** all the centres, radius,  for the approximate hullS of the cat represented by cylinders and spheres (length + other side center for cylinders) */
    std::vector<cpe::vec3> centres;
    std::vector <float> radius;
    std::vector<float> length;
    std::vector<cpe::vec3> centres_bis;

    /** OpenGL ID for shader drawing meshes */
    GLuint shader_mesh;
    /** OpenGL ID for the texture of the cloth */
    GLuint texture_cloth;
    /** OpenGL ID for the texture of the ground */
    GLuint texture_ground;
    /** OpenGL ID for the texture of the cat */
    GLuint texture_cat;
    /** OpenGL ID for the texture of the cat's hull Debug Only*/
    GLuint texture_cat_hull;


    /** Time counter */
    QTime time_integration;

    /** Setup the shader for the mesh */
    void setup_shader_mesh(GLuint shader_id);

    /** The time interval for the numerical integration */
    float delta_t;
    /** Variable indicating if the system diverged (stop the time integration) */
    bool divergence;


};

#endif
