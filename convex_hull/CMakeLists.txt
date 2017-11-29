
cmake_minimum_required(VERSION 2.8.11)

project(cgp)



set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -O2 -g")

file(
    GLOB_RECURSE
    source_files
    project/*.[ch]pp
)

add_executable(convex ${source_files})
