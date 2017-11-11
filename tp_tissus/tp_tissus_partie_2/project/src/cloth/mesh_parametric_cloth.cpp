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

vec3 calcul_force(vec3 p0,vec3 p1)
{
    static float const L_rest = 1.0f/30.0f; //modifier le L_rest pour calcul shearing springs
    static float const K = 5.0f;
    float  L = norm(p0 - p1);
    return  K * (L-L_rest) * (p0-p1)/norm(p0 - p1);
}
vec3 calcul_force2(vec3 p0,vec3 p1)
{
    static float const L_rest = 1.0f/(2.0f*30.0f);
    static float const K = 5.0f;
    float  L = norm(p0 - p1);
    return  K * (L-L_rest) * (p0-p1)/norm(p0 - p1);
}

void mesh_parametric_cloth::update_force()
{

    int const Nu = size_u();
    int const Nv = size_v();
    int const N_total = Nu*Nv;
    ASSERT_CPE(static_cast<int>(force_data.size()) == Nu*Nv , "Error of size");



    //Gravity
    static vec3 const g (0.0f,0.0f,-9.81f);
    vec3 const g_normalized = g/N_total;
    for(int ku=0 ; ku<Nu ; ++ku)
    {
        for(int kv=0 ; kv<Nv ; ++kv)
        {

            force(ku,kv) = g_normalized;
        }
    }

    //*************************************************************//
    // TO DO, Calculer les forces s'appliquant sur chaque sommet
    //*************************************************************//



    // Gestion des points interieurs
    for(int ku=1 ; ku<Nu-1 ; ++ku)
    {
        for(int kv=1 ; kv<Nv-1 ; ++kv)
        {
            vec3  p = vertex(ku,kv);

            //Structural springs
            vec3  p_bas = vertex(ku+1,kv); vec3  p_haut = vertex(ku-1,kv);
            vec3  p_gauche = vertex(ku,kv-1); vec3  p_droite = vertex(ku,kv+1);
            force(ku,kv) -= calcul_force2(p,p_bas) + calcul_force2(p,p_haut) + calcul_force2(p,p_droite)  + calcul_force2(p,p_gauche);

            //Shearing springs
            vec3  p_bg = vertex(ku+1,kv-1); vec3  p_bd = vertex(ku+1,kv+1);
            vec3  p_hg = vertex(ku-1,kv-1); vec3  p_hd = vertex(ku-1,kv+1);
            force(ku,kv) -= calcul_force2(p,p_bg) + calcul_force2(p,p_hg) + calcul_force2(p,p_bd)  + calcul_force2(p,p_hd);

            //Bending Springs
            if(ku>=2 && ku<=Nu-3 && kv>=2 && kv<=Nv-3) //TOUT l'intérieur avec 4 points
            {
                vec3 pb = vertex(ku+2,kv); vec3  ph = vertex(ku-2,kv);
                vec3  pg = vertex(ku,kv-2); vec3  pd = vertex(ku,kv+2);
                force(ku,kv) -= calcul_force2(p,pg) + calcul_force2(p,ph) + calcul_force2(p,pb)  + calcul_force2(p,pd);
            }
            else if(ku==1 && kv>=2 && kv<=Nv-3)
            {
                vec3 pb = vertex(ku+2,kv);
                vec3  pg = vertex(ku,kv-2); vec3  pd = vertex(ku,kv+2);
                force(ku,kv) -= calcul_force2(p,pg)  + calcul_force2(p,pb)  + calcul_force2(p,pd);
            }
            else if(ku==Nu-2 && kv>=2 && kv<=Nv-3)
            {
                vec3  ph = vertex(ku-2,kv);
                vec3  pg = vertex(ku,kv-2); vec3  pd = vertex(ku,kv+2);
                force(ku,kv) -= calcul_force2(p,pg) + calcul_force2(p,ph)   + calcul_force2(p,pd);
            }
            else if(kv==1 && ku >=2 && ku<=Nu-3)
            {
                vec3 pb = vertex(ku+2,kv); vec3  ph = vertex(ku-2,kv);
                vec3  pd = vertex(ku,kv+2);
                force(ku,kv) -=   calcul_force2(p,ph) + calcul_force2(p,pb)  + calcul_force2(p,pd);
            }
            else if(kv==Nv-2 && ku >=2 && ku<=Nu-3)
            {
                vec3 pb = vertex(ku+2,kv); vec3  ph = vertex(ku-2,kv);
                vec3  pg = vertex(ku,kv-2);
                force(ku,kv) -= calcul_force2(p,pg) + calcul_force2(p,ph) + calcul_force2(p,pb);
            }
            else if(ku==1 && kv==1)
            {
                vec3  pd = vertex(ku,kv+2);
                vec3  pb = vertex(ku+2,kv);
                force(ku,kv) -= calcul_force2(p,pd) + calcul_force2(p,pb) ;
            }
            else if(ku==1 && kv==Nv-2)
            {
                vec3  pb = vertex(ku+2,kv);
                vec3  pg = vertex(ku,kv-2);
                force(ku,kv) -= calcul_force2(p,pb) + calcul_force2(p,pg) ;
            }
            else if(ku==Nu-2 && kv==Nv-2)
            {
                vec3  ph = vertex(ku-2,kv);
                vec3  pg = vertex(ku,kv-2);
                force(ku,kv) -= calcul_force2(p,ph) + calcul_force2(p,pg) ;
            }
            else if(ku==Nu-2 && kv==1)
            {
                vec3  ph = vertex(ku-2,kv);
                vec3  pd = vertex(ku,kv+2);
                force(ku,kv) -= calcul_force2(p,ph) + calcul_force2(p,pd) ;
            }


        }
    }

    // Gestion des bords
    for(int ku=1 ; ku<Nu-1 ; ++ku)
    {
        //Strucural springs
        vec3  p = vertex(ku,0);
        vec3  p_bas = vertex(ku+1,0); vec3  p_haut = vertex(ku-1,0); vec3  p_droite = vertex(ku,1);
        force(ku,0) -= calcul_force(p,p_bas) + calcul_force(p,p_haut) + calcul_force(p,p_droite);

        p = vertex(ku,Nv-1);
        p_bas = vertex(ku+1,Nv-1); p_haut = vertex(ku-1,Nv-1); vec3  p_gauche = vertex(ku,Nv-2);
        force(ku,Nv-1) -= calcul_force(p,p_bas) + calcul_force(p,p_haut) + calcul_force(p,p_gauche);

        //Shearing springs
        p = vertex(ku,0);
        vec3  p_bd = vertex(ku+1,1);vec3  p_hd = vertex(ku-1,1);
        force(ku,0) -= calcul_force(p,p_bd)  + calcul_force(p,p_hd);
        p = vertex(ku,Nv-1);
        vec3  p_hg = vertex(ku-1,Nv-2); vec3  p_bg = vertex(ku+1,Nv-2);
        force(ku,Nv-1)-= calcul_force(p,p_bg) + calcul_force(p,p_hg) ;

        //Bending Springs



        if(ku >=2 && ku<=Nu-3)
        {
            p = vertex(ku,0);
            vec3 pb = vertex(ku+2,0); vec3  ph = vertex(ku-2,0); vec3  pd = vertex(ku,2);
            force(ku,0) -=   calcul_force2(p,ph) + calcul_force2(p,pb)  + calcul_force2(p,pd);
            p = vertex(ku,Nv-1);
            pb = vertex(ku+2,Nv-1);   ph = vertex(ku-2,Nv-1); vec3  pg = vertex(ku,Nv-3);
            force(ku,Nv-1) -=   calcul_force2(p,ph) + calcul_force2(p,pb)  + calcul_force2(p,pg);
        }
        else if(ku ==1)
        {
            p = vertex(ku,0);
            vec3 pb = vertex(ku+2,0);  vec3  pd = vertex(ku,2);
            force(ku,0) -=    calcul_force2(p,pb)  + calcul_force2(p,pd);
            p = vertex(ku,Nv-1);
            pb = vertex(ku+2,Nv-1);  vec3  pg = vertex(ku,Nv-3);
            force(ku,Nv-1) -=    calcul_force2(p,pb)  + calcul_force2(p,pg);

        }
        else if(ku == Nu-2 )
        {
            p = vertex(ku,0);
            vec3  ph = vertex(ku-2,0); vec3  pd = vertex(ku,2);
            force(ku,0) -=   calcul_force2(p,ph) +  calcul_force2(p,pd);
            p = vertex(ku,Nv-1);
            ph = vertex(ku-2,Nv-1); vec3  pg = vertex(ku,Nv-3);
            force(ku,Nv-1) -=   calcul_force2(p,ph) +  calcul_force2(p,pg);
        }


    }

    for(int kv=1 ; kv<Nv-1 ; ++kv)
    {
        //Strucural springs
        vec3  p = vertex(0,kv);
        vec3  p_bas = vertex(1,kv);  vec3  p_gauche = vertex(0,kv-1); vec3  p_droite = vertex(0,kv+1);
        force(0,kv) -= calcul_force(p,p_bas) + calcul_force(p,p_gauche) + calcul_force(p,p_droite);

        p = vertex(Nu-1,kv);
        p_droite = vertex(Nu-1,kv+1); vec3 p_haut = vertex(Nu-2,kv); p_gauche = vertex(Nu-1,kv-1);
        force(Nu-1,kv) -= calcul_force(p,p_droite) + calcul_force(p,p_haut) + calcul_force(p,p_gauche);

        //Shearing springs
        p = vertex(0,kv);
        vec3  p_bg = vertex(1,kv-1); vec3  p_bd = vertex(1,kv+1);
        force(0,kv) -= calcul_force(p,p_bg)  + calcul_force(p,p_bd);
        p = vertex(Nu-1,kv);
        vec3  p_hg = vertex(Nu-2,kv-1); vec3  p_hd = vertex(Nu-2,kv+1);
        force(Nu-1,kv) -= calcul_force(p,p_hg) +  calcul_force(p,p_hd);

        //Bending Springs

        if(kv>= 2 && kv <= Nv-3)
        {
            p = vertex(0,kv);
            vec3  pb = vertex(2,kv);  vec3  pg = vertex(0,kv-2); vec3  pd = vertex(0,kv+2);
            force(0,kv) -= calcul_force2(p,pb) + calcul_force2(p,pg) + calcul_force2(p,pd);

            p = vertex(Nu-1,kv);
            pb = vertex(Nu-1,kv+2); vec3 ph = vertex(Nu-3,kv); pg = vertex(Nu-1,kv-2);
            force(Nu-1,kv) -= calcul_force2(p,pb) + calcul_force2(p,ph) + calcul_force2(p,pg);
        }
        else if(kv ==1)
        {
            p = vertex(0,kv);
            vec3  pb = vertex(2,kv);  vec3  pd = vertex(0,kv+2);
            force(0,kv) -= calcul_force2(p,pb) + calcul_force2(p,pd);

            p = vertex(Nu-1,kv);
            pd = vertex(Nu-1,kv+2); vec3 ph = vertex(Nu-3,kv);
            force(Nu-1,kv) -= calcul_force2(p,pd) + calcul_force2(p,ph);

        }
        else if (kv == Nv-2)
        {
            p = vertex(0,kv);
            vec3  pb = vertex(2,kv);  vec3  pg = vertex(0,kv-2);
            force(0,kv) -= calcul_force2(p,pb) + calcul_force2(p,pg);

            p = vertex(Nu-1,kv);
            pg = vertex(Nu-1,kv-2); vec3 ph = vertex(Nu-3,kv);
            force(Nu-1,kv) -= calcul_force2(p,pg) + calcul_force2(p,ph);
        }

    }
    // Gestion des 4 coins

    //Pour 2 sommets des extremités du maillage : force à 0
    force(0,0) = vec3(0.0f,0.0f,0.0f);
    force(0,Nv-1) = vec3(0.0f,0.0f,0.0f);

    //    vec3 p = vertex(0,0);
    //    vec3 p_bas = vertex(1,0);
    //    vec3 p_droite = vertex(0,1);

    //    force(0,0) += calcul_force(p,p_droite) + calcul_force(p,p_bas);

    vec3  p = vertex(Nu-1,0);
    vec3 p_haut = vertex(Nu-2,0);
    vec3 p_droite = vertex(Nu-1,1);
    vec3  p_hd = vertex(Nu-2,1);
    vec3 ph = vertex(Nu-3,0);
    vec3 pd = vertex(Nu-1,2);
    force(Nu-1,0) -= calcul_force(p,p_haut) + calcul_force(p,p_droite) + calcul_force(p,p_hd)
                     + calcul_force2(p,ph) + calcul_force2(p,pd);

    //    p = vertex(0,Nv-1);
    //    vec3 p_gauche = vertex(0,Nv-2);
    //    p_bas = vertex(1,Nv-1);
    //    force(0,Nv-1) += calcul_force(p,p_bas) + calcul_force(p,p_gauche);

    p = vertex(Nu-1,Nv-1);
    p_haut = vertex(Nu-2,Nv-1);
    vec3 p_gauche = vertex(Nu-1,Nv-2);
    vec3  p_hg = vertex(Nu-2,Nv-2);
    ph = vertex(Nu-3,Nv-1);
    vec3 pg = vertex(Nu-1,Nv-3);

    force(Nu-1,Nv-1) -= calcul_force(p,p_haut) + calcul_force(p,p_gauche) + calcul_force(p,p_hg)
                        + calcul_force2(p,ph) + calcul_force2(p,pg);

}

void mesh_parametric_cloth::integration_step(float const dt)
{
    ASSERT_CPE(speed_data.size() == force_data.size(),"Incorrect size");
    ASSERT_CPE(static_cast<int>(speed_data.size()) == size_vertex(),"Incorrect size");


    int const Nu = size_u();
    int const Nv = size_v();

    static float const mu = 0.2f;
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


}
