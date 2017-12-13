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

#include "mesh_parametric_cloth.hpp"

#include "../lib/common/error_handling.hpp"
#include <cmath>


namespace cpe
{

vec3 mesh_parametric_cloth::calcul_force_structural(vec3 p0,vec3 p1)
{
    static float const L_rest = 1.0f/30.0f; //modifier le L_rest pour calcul shearing springs
    //static float const K = 5.0f;
    float  L = norm(p0 - p1);
    return  K_structural * (L-L_rest) * (p0-p1)/norm(p0 - p1);
}

vec3 mesh_parametric_cloth::calcul_force_shearing(vec3 p0,vec3 p1)
{
    static float const L_rest = sqrt(2.0f/(30.0f*30.0f));
    float  L = norm(p0 - p1);
    return  K_shearing * (L-L_rest) * (p0-p1)/norm(p0 - p1);
}

vec3 mesh_parametric_cloth::calcul_force_bending(vec3 p0,vec3 p1)
{
    static float const L_rest = 2.0f/(30.0f);
    //static float const K = 5.0f;
    float  L = norm(p0 - p1);
    return  K_bending * (L-L_rest) * (p0-p1)/norm(p0 - p1);
}

vec3 mesh_parametric_cloth::calcul_force_wind(vec3 n)
{
    wind_direction = {1.0f,0.0f,0.0f};
    return Kw * dot(n,wind_direction) * n;
}

void mesh_parametric_cloth::update_force()
{

    int const Nu = size_u();
    int const Nv = size_v();
    int const N_total = Nu*Nv;
    ASSERT_CPE(static_cast<int>(force_data.size()) == Nu*Nv , "Error of size");




    static vec3 const g (0.0f,0.0f,-9.81f);
    vec3 const g_normalized = g/N_total;
    for(int ku=0 ; ku<Nu ; ++ku)
        for(int kv=0 ; kv<Nv ; ++kv)
        {

            //*******//
            // Gravity
            //*******//
            force(ku,kv) = g_normalized;

            //*************************************************************//
            //  Calculer les forces des ressorts s'appliquant sur chaque sommet
            //*************************************************************//

            //Fill structural (ku-1,kv),(ku,kv-1),(ku+1,kv),(ku,kv+1) haut gauche bas droite
            std::vector<vec3> line_structural;
            //point haut
            if(ku-1 >=0)
                line_structural.push_back( calcul_force_structural(vertex(ku,kv),vertex(ku-1,kv)) );
            else
                line_structural.push_back( vec3(-1.0f,-1.0f,-1.0f) );

            //point gauche
            if(kv-1>=0)
                line_structural.push_back( calcul_force_structural(vertex(ku,kv),vertex(ku,kv-1)) );
            else
                line_structural.push_back( vec3(-1.0f,-1.0f,-1.0f) );

            //point bas
            if(ku+1 <=Nu-1)
                line_structural.push_back( calcul_force_structural(vertex(ku,kv),vertex(ku+1,kv)) );
            else
                line_structural.push_back( vec3(-1.0f,-1.0f,-1.0f) );

            //point droit
            if(kv+1<=Nv-1)
                line_structural.push_back( calcul_force_structural(vertex(ku,kv),vertex(ku,kv+1)) );
            else
                line_structural.push_back( vec3(-1.0f,-1.0f,-1.0f) );

            for(auto f : line_structural )
            {
                if( f.x() != -1.0f && f[1] != -1.0f && f[2] != -1.0f)
                    force(ku,kv) -= f;
            }

            //fill shearing (ku-1,kv-1),(ku+1,kv-1),(ku+1,kv+1)(ku-1,kv+1) hg bg bd hd
            std::vector<vec3> line_shearing;
            //point haut gauche hg
            if(ku-1>=0 && kv-1 >= 0)
                line_shearing.push_back( calcul_force_shearing(vertex(ku,kv), vertex(ku-1,kv-1)));
            else
                line_shearing.push_back(vec3(-1.0f,-1.0f,-1.0f));
            //point bas gauche
            if(ku+1<=Nu-1 && kv-1 >= 0)
                line_shearing.push_back( calcul_force_shearing(vertex(ku,kv), vertex(ku+1,kv-1)));
            else
                line_shearing.push_back(vec3(-1.0f,-1.0f,-1.0f));
            //point bas droite
            if(ku+1<=Nu-1 && kv+1 <= Nv-1)
                line_shearing.push_back( calcul_force_shearing(vertex(ku,kv), vertex(ku+1,kv+1)));
            else
                line_shearing.push_back(vec3(-1.0f,-1.0f,-1.0f));
            //point haut droite
            if(ku-1>=0 && kv+1 <= Nv-1)
                line_shearing.push_back( calcul_force_shearing(vertex(ku,kv), vertex(ku-1,kv+1)));
            else
                line_shearing.push_back(vec3(-1.0f,-1.0f,-1.0f));

            for(auto f : line_shearing )
            {
                if( f.x() != -1.0f && f[1] != -1.0f && f[2] != -1.0f)
                    force(ku,kv) -= f;
            }

            //fill bending (ku-1,kv-1),(ku+1,kv-1),(ku+1,kv+1)(ku-1,kv+1) haut gauche bas droite
            std::vector<vec3> line_bending;
            //point haut
            if(ku-2 >=0)
                line_bending.push_back( calcul_force_bending(vertex(ku,kv),vertex(ku-2,kv)) );
            else
                line_bending.push_back( vec3(-1.0f,-1.0f,-1.0f) );

            //point gauche
            if(kv-2>=0)
                line_bending.push_back( calcul_force_bending(vertex(ku,kv),vertex(ku,kv-2)) );
            else
                line_bending.push_back( vec3(-1.0f,-1.0f,-1.0f) );

            //point bas
            if(ku+2 <=Nu-1)
                line_bending.push_back( calcul_force_bending(vertex(ku,kv),vertex(ku+2,kv)) );
            else
                line_bending.push_back( vec3(-1.0f,-1.0f,-1.0f) );

            //point droit
            if(kv+2<=Nv-1)
                line_bending.push_back( calcul_force_bending(vertex(ku,kv),vertex(ku,kv+2)) );
            else
                line_bending.push_back( vec3(-1.0f,-1.0f,-1.0f) );

            for(auto f : line_bending )
            {
                if( f.x() != -1.0f && f[1] != -1.0f && f[2] != -1.0f)
                    force(ku,kv) -= f;
            }


            //***********//
            //WIND
            //***********//
            force(ku,kv) += calcul_force_wind(normal(ku,kv));
        }



    force(0,0) = vec3(0,0,0);
    force(0,Nv-1) = vec3(0,0,0);


}

void mesh_parametric_cloth::integration_step()
{
    ASSERT_CPE(speed_data.size() == force_data.size(),"Incorrect size");
    ASSERT_CPE(static_cast<int>(speed_data.size()) == size_vertex(),"Incorrect size");


    int const Nu = size_u();
    int const Nv = size_v();

    static float const mu = 0.8f;
    //*************************************************************//
    // TO DO: Calculer l'integration numerique des positions au cours de l'intervalle de temps dt.
    //*************************************************************//
    for(int ku=0 ; ku<Nu ; ++ku)
    {
        for(int kv=0 ; kv<Nv ; ++kv)
        {
            speed(ku,kv) = (1-mu*dt)*speed(ku,kv) + force(ku,kv) * dt;
            vertex(ku,kv) = vertex(ku,kv) + dt*speed(ku,kv);
        }
    }

    //*************************************************************//


    //security check (throw exception if divergence is detected)
    static float const LIMIT=30.0f;
    for(int ku=0 ; ku<Nu ; ++ku)
    {
        for(int kv=0 ; kv<Nv ; ++kv)
        {
            vec3 const& p = vertex(ku,kv);

            if( norm(p) > LIMIT )
            {
                throw exception_divergence("Divergence of the system",EXCEPTION_PARAMETERS_CPE);
            }
        }
    }

}

void mesh_parametric_cloth::set_plane_xy_unit(int const size_u_param,int const size_v_param)
{
    mesh_parametric::set_plane_xy_unit(size_u_param,size_v_param);

    int const N = size_u()*size_v();
    speed_data.resize(N);
    force_data.resize(N);
    collision_plan_data.resize(N);

}
void mesh_parametric_cloth::set_plane_xy_unit(int const size_u_param,int const size_v_param, float offset_x , float offset_y)
{
    mesh_parametric::set_plane_xy_unit(size_u_param,size_v_param, offset_x, offset_y);

    int const N = size_u()*size_v();
    speed_data.resize(N);
    force_data.resize(N);
    collision_plan_data.resize(N);

}

vec3 const& mesh_parametric_cloth::speed(int const ku,int const kv) const
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(speed_data.size()),"Internal error");

    return speed_data[offset];
}

vec3& mesh_parametric_cloth::speed(int const ku,int const kv)
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(speed_data.size()),"Internal error");

    return speed_data[offset];
}

vec3 const& mesh_parametric_cloth::force(int const ku,int const kv) const
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(force_data.size()),"Internal error");

    return force_data[offset];
}

vec3& mesh_parametric_cloth::force(int const ku,int const kv)
{
    ASSERT_CPE(ku >= 0 , "Value ku ("+std::to_string(ku)+") should be >=0 ");
    ASSERT_CPE(ku < size_u() , "Value ku ("+std::to_string(ku)+") should be < size_u ("+std::to_string(size_u())+")");
    ASSERT_CPE(kv >= 0 , "Value kv ("+std::to_string(kv)+") should be >=0 ");
    ASSERT_CPE(kv < size_v() , "Value kv ("+std::to_string(kv)+") should be < size_v ("+std::to_string(size_v())+")");

    int const offset = ku + size_u()*kv;

    ASSERT_CPE(offset < static_cast<int>(force_data.size()),"Internal error");

    return force_data[offset];
}



void mesh_parametric_cloth::update_plan_collision(cpe::mesh m)
{
    float h = m.vertex(0).z();
    int const Nu = size_u();
    int const Nv = size_v();
    int const N_total = Nu*Nv;
    ASSERT_CPE(static_cast<int>(collision_plan_data.size()) == Nu*Nv , "Error of size");

    float epsilon=0.01f;

    for(int ku=0 ; ku<Nu ; ++ku)
    {
        for(int kv=0 ; kv<Nv ; ++kv)
        {
            if(this->vertex(ku,kv).z() < h+epsilon)
            {
                force(ku,kv) = vec3(force(ku,kv).x(),force(ku,kv).y(),0.0f);
                speed(ku,kv) = vec3(speed(ku,kv).x(), speed(ku,kv).y(),0.0f);
                vertex(ku,kv).z() = h +epsilon;
            }
        }
    }
}

void mesh_parametric_cloth::update_shpere_collision(mesh m, cpe::vec3 centre, float radius)
{
    int const N = m.size_vertex();

    int const Nu = size_u();
    int const Nv = size_v();
    int const N_total = Nu*Nv;
    ASSERT_CPE(static_cast<int>(collision_plan_data.size()) == Nu*Nv , "Error of size");

    float epsilon=0.0015f;

    for(int ku=0 ; ku<Nu ; ++ku)
    {
        for(int kv=0 ; kv<Nv ; ++kv)
        {
            for(int k=0; k<N; k++)
            {
                if(distance(vertex(ku,kv),centre) < radius+epsilon)
                {
                    float speed_n = dot(speed(ku,kv),m.normal(k));
                    float force_n = dot(force(ku,kv),m.normal(k));
                    force(ku,kv) -= force_n *m.normal(k);
                    speed(ku,kv) -= speed_n * m.normal(k);
                    vertex(ku,kv) += epsilon *m.normal(k);
                }
            }
        }
    }

}

float mesh_parametric_cloth::distance(vec3 A,vec3 B)
{
    return sqrt( (A.x()-B.x())*(A.x()-B.x()) + (A.y()-B.y())*(A.y()-B.y()) + (A.z()-B.z())*(A.z()-B.z()));
}



float& mesh_parametric_cloth::set_dt()
{
    return dt;
}

float &mesh_parametric_cloth::set_wind()
{
    return Kw;
}

float &mesh_parametric_cloth::set_K_structural()
{
    return K_structural;
}
float &mesh_parametric_cloth::set_K_shearing()
{
    return K_shearing;
}
float &mesh_parametric_cloth::set_K_bending()
{
    return K_bending;
}
}//Accolade du namespace
