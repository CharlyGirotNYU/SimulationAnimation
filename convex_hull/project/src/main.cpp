#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>

#include "compute_hull.hpp"
#include "compute_quick_hull.hpp"

void export_data(const std::vector<vec2>& V,const std::vector<vec2>& hull);

int main()
{
    // Generate N random points
    const int N = 1000000;
    std::vector<vec2> V(N);
    point_generator generator;
    for(int k=0; k<N; ++k)
        V[k] = generator();


    // call convex hull computation
    const auto time_start = std::chrono::system_clock::now();
    std::vector<vec2> hull = compute_quick_hull(V);
    const auto time_end = std::chrono::system_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
    std::cout<<"Elapsed time: "<<ms<<"ms"<<std::endl;


    // export data for visualization
    export_data(V,hull);

}



void export_data(const std::vector<vec2>& V,const std::vector<vec2>& hull)
{
    const std::string filename = "out.m";
    std::ofstream s(filename);
    if( !s.good() )
    {std::cout<<"Error opening file "+filename<<std::endl;abort();}


    s << "V=[";
    const size_t N = V.size();
    for(size_t k=0; k<N; ++k) {
        const vec2& p = V[k];
        s<<"["<<p.x<<";"<<p.y<<"],";
    }
    s << "];"<<std::endl;

    s << "hull=[";
    const size_t Nh = hull.size();
    for(size_t k=0; k<Nh; ++k) {
        const vec2& p = hull[k];
        s<<"["<<p.x<<";"<<p.y<<"],";
    }
    s << "];"<<std::endl;



    s.close();
}



