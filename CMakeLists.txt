cmake_minimum_required(VERSION 3.10)
project(bhs_game VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX23_STANDARD_COMPILE_OPTION "-std=c++23")
set(CMAKE_CXX23_EXTENSION_COMPILE_OPTION "-std=gnu++23")

include_directories(include/*)

add_subdirectory(src)
add_subdirectory(third_party/eigen)

enable_testing()
add_subdirectory(tests)
