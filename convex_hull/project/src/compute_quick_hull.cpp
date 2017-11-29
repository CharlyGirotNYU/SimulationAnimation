#include "compute_hull.hpp"
#include<algorithm>
#include<iostream>
#include "vec2.hpp"

bool sign_det(vec2 A, vec2 B, vec2 X)
{
    return  ((B.x - A.x)*(X.y-A.y) - (X.x -A.x)*(B.y-A.y)) < 0;
}


std::vector<vec2> remove_pointsin_box(const std::vector<vec2>& box, std::vector<vec2>& V)
{

    return V;
}


std::vector<vec2> compute_quick_hull(const std::vector<vec2>& V)
{
    std::vector<vec2> hull;

    //Initialisation d'un carré
    //    std::vector<vec2> box;
    //    for(int i=0;i<4;i++)
    //    {
    //        box.push_back(V[rand() % V.size()]);
    //    }

    //    //Ordonnner la box
    //    for(int i=0; )
    //Création d'une box de taille définit pour intialisier (triche)
    vec2 pt = V[rand() % V.size()];
    std::vector<vec2> box;

    float xmin=1000;
    float ymin = 1000;

    for(auto v : V)
    {
        if(v.x < xmin)
        {
            xmin=v.x; //point le plus a gauche
            ymin=v.y;
            vec2 A;
            A.x = xmin;
            A.y = ymin;
            break;
        }
    }
    box.push_back(A);
    for(auto v : V)
    {
        if(v.y > ymin)
        {
            xmin=v.x; //point le plus en haut
            ymin=v.y;
            vec2 A;
            A.x = xmin;
            A.y = ymin;
            break;
        }
    }




    for(auto b : box)
        std::cout << b.x << " " << b.y << std::endl;


    return {hull};
}











