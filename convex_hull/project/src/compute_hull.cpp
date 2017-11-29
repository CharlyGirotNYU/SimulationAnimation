#include "compute_hull.hpp"
#include<algorithm>
#include<iostream>
#include "vec2.hpp"


/* Return true if det > 0 */
bool sign_det(vec2 A, vec2 B, vec2 X)
{
    return  ((B.x - A.x)*(X.y-A.y) - (X.x -A.x)*(B.y-A.y)) < 0;
}

/* Return true if all points are left */
bool allAtLeft(vec2 A, vec2 B, const std::vector<vec2>& V)
{
    for(auto X : V)
    {
        if(sign_det(A,B,X)) //si j'ai un point a droite
        {
            return false;
        }
    }
    return true;
}

std::vector<vec2> compute_hull(const std::vector<vec2>& V)
{
    std::vector<vec2> hull;

    //Initialisation du point A
    float xmin=1000;
    float ymin = 1000;

    for(auto v : V)
    {
        if(v.x < xmin)
        {
            xmin=v.x; //point le plus a gauche
            ymin=v.y;
        }
    }

    vec2 A; A.x=xmin; A.y = ymin;
    hull.push_back(A);

    // Choisir un point B
    vec2 B_tmp;
    while(B_tmp .x != hull[0].x && B_tmp.y != hull[0].y)
    {
        for( auto B : V)
        {
            if(allAtLeft(A,B,V)) //Si AB laisse tout les points a gauche
            {
                hull.push_back(B);
                A=B;
                B_tmp = B;
            }
        }
    }
    return {hull};
}
