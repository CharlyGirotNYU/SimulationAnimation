/*
**    TP CPE Lyon
**    Copyright (C) 2015 Damien Rohmer
**
**    This program is free software: you can redistribute it and/or modify
**    it under the terms of the GNU General Public License as published by
**    the Free Software Foundation, either version 3 of the License, or
**    (at your option) any later version.
**
**   This program is distributed in the hope that it will be useful,
**    but WITHOUT ANY WARRANTY; without even the implied warranty of
**    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**    GNU General Public License for more details.
**
**    You should have received a copy of the GNU General Public License
**    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#ifndef MESH_PARAMETRIC_CLOTH_HPP
#define MESH_PARAMETRIC_CLOTH_HPP

#include "../lib/mesh/mesh_parametric.hpp"
#include "../lib/common/exception_cpe.hpp"
#include "../lib/mesh/mesh.hpp"

namespace cpe
{
class mesh_parametric_cloth : public mesh_parametric
{
public:
    using mesh_parametric::mesh_parametric;

    void set_plane_xy_unit(int const size_u_param,int const size_v_param);
    void set_plane_xy_unit(int const size_u_param,int const size_v_param, float offset_x, float offset_y);

    vec3 const& speed(int ku,int kv) const;
    vec3& speed(int ku,int kv);

    vec3 const& force(int ku,int kv) const;
    vec3& force(int ku,int kv);

    void update_force();
    void integration_step();

    /* Compute force exerced by structural springs */
    vec3 calcul_force_structural(vec3 p0,vec3 p1);
    /* Compute force exerced by shearing springs */
    vec3 calcul_force_shearing(vec3 p0,vec3 p1);
    /* Compute force exerced by bending springs */
    vec3 calcul_force_bending(vec3 p0,vec3 p1);

    /* Compute force exerced by wind */
    vec3 calcul_force_wind(vec3 n);


    /* Check collision with a plan given as parameter*/
    void update_plan_collision(mesh m);

    /* Check collision with a sphere given as parameter */
    void update_shpere_collision(mesh m, vec3 centre, float radius);

    void update_cat_collision(mesh m, float radius_cylindre, vec3 centre_cylindre, vec3 centre2);

    /** Set Delta T called by the ui */
    float& set_dt();
    /** Set wind power value called by the ui*/
    float& set_wind();
    /** Set K values called by the ui */
    float& set_K_structural();
    float& set_K_shearing();
    float& set_K_bending();
    /** Compute distance between two vec3 */
    float distance(vec3 A,vec3 B);
    float distance_xz(vec3 A, vec3 B);

private:

    std::vector<vec3> speed_data;
    std::vector<vec3> force_data;
    std::vector<bool> collision_plan_data;
    vec3 wind_direction;

    float K_structural, K_shearing, K_bending;
    float Kw; //wind
    float dt;

};

class exception_divergence : public exception_cpe
{
    using exception_cpe::exception_cpe;
};

}

#endif
