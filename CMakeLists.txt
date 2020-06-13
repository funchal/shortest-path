cmake_minimum_required(VERSION 3.16)

find_program(CMAKE_CXX_COMPILER clang++)
if (NOT CMAKE_CXX_COMPILER)
    message(FATAL_ERROR "clang++ not found")
endif()

project(shortestpath CXX)

set(CMAKE_CXX_STANDARD 17)

add_executable(shortestpath main.cpp)
#target_compile_options(shortestpath PRIVATE "-Weverything;-Werror;-Wno-c++98-compat;-Wno-c++98-compat-pedantic;-Wno-padded;-Wno-missing-prototypes;-Wno-sign-conversion")
