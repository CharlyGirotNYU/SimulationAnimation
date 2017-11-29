#pragma once

#include <random>

struct vec2
{
    float x,y;
    vec2 operator()(float a, float b):x(a),y(b){};
};

class point_generator
{

public:
    point_generator(int value = 0)
        : generator(std::default_random_engine(value)), distribution(std::normal_distribution<float>(0.0f,1.0f))
    {}

    vec2 operator()()
    {
        return {distribution(generator),distribution(generator)};
    }



private:
    std::default_random_engine generator;
    std::normal_distribution<float> distribution;

};
