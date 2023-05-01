cmake_minimum_required(VERSION 3.25 FATAL_ERROR)
project(bhs_game VERSION 1.0.0 LANGUAGES CXX C CUDA)
# COLORS:
if (NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(ColourBold "${Esc}[1m")
    set(Red "${Esc}[31m")
    set(Green "${Esc}[32m")
    set(Yellow "${Esc}[33m")
    set(Blue "${Esc}[34m")
    set(Magenta "${Esc}[35m")
    set(Cyan "${Esc}[36m")
    set(White "${Esc}[37m")
    set(BoldRed "${Esc}[1;31m")
    set(BoldGreen "${Esc}[1;32m")
    set(BoldYellow "${Esc}[1;33m")
    set(BoldBlue "${Esc}[1;34m")
    set(BoldMagenta "${Esc}[1;35m")
    set(BoldCyan "${Esc}[1;36m")
    set(BoldWhite "${Esc}[1;37m")
endif ()


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
#

#set(CMAKE_C_COMPILER /usr/bin/gcc-9)
#set(CMAKE_CXX_COMPILER /usr/bin/g++-9)

#set(CMAKE_CXX23_STANDARD_COMPILE_OPTION "-std=c++23")
#set(CMAKE_CXX23_EXTENSION_COMPILE_OPTION "-std=gnu++23")
set(THE_EXE_NAME bhs_game_executable)

include(ExternalProject)


if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("${Blue}Building in debug mode${ColourReset}")
    add_definitions(-DDEBUG -URELEASE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")


    # comment this if you want to remove the verbose output
    #    add_definitions(-DVERBOSE)
elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
    message("${Blue}Building in release mode${ColourReset}")
    add_definitions(-DRELEASE -UVERBOSE -UDEBUG)
    #add google sanatizers
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread -g")


    # comment this if you want to remove the verbose output
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    message("${Blue}Building in release mode${ColourReset}")
    add_definitions(-DRELEASE -DVERBOSE-UDEBUG)
    # comment this if you want to remove the verbose output
else ()
    message(WARNING "Building in something mode???????")
    add_definitions(-DRELEASE -UVERBOSE -UDEBUG)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

    # comment this if you want to remove the verbose output
    #    add_definitions(-DVERBOSE)
endif ()

# ======================
# Setting compile params:
# ======================

# allow for the use of the -pthread flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
# Make the LibTorch library work:
# CUDA PARAMS:
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-12.1)
set(CUDA_COMPILER /usr/local/cuda-12.1/bin/nvcc)

# Enable CUDNN
set(USE_CUDNN ON)
set(CAFFE2_USE_CUDNN ON)


set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost")

find_package(Torch REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch")
if (NOT TORCH_FOUND)
    message(STATUS "TORCH NOT FOUND")
    # The following needs to be set by whom ever is building this
    set(LIBTORCH_VERSION "2.0.0") # Change this to your desired version
    set(LIBTORCH_URL "https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}%2Bcu118.zip") # Update this with the URL you chose
    set(LIBTORCH_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch")

    message(STATUS "Downloading LibTorch to ${LIBTORCH_PATH}...")
    ExternalProject_Add(
            libtorch
            URL "${LIBTORCH_URL}"  # Update this with the URL you chose
            #            URL_HASH SHA256=83e43d63d0e7dc9d57ccbdac8a8d7edac6c9e18129bf3043be475486b769a9c2 # Replace <the_sha256_hash> with the calculated hash
            DOWNLOAD_NAME "libtorch.zip"
            DOWNLOAD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party"
            DOWNLOAD_NO_PROGRESS 0
            TIMEOUT 6000
            LOG_DOWNLOAD 1
            SOURCE_DIR "${LIBTORCH_PATH}"
            CONFIGURE_COMMAND ""
            BUILD_COMMAND ""
            INSTALL_COMMAND ""
            TEST_COMMAND ""
            DOWNLOAD_EXTRACT_TIMESTAMP true
    )
    message(STATUS "Downloading LibTorch to ${LIBTORCH_PATH}... Done!")
    set(CMAKE_PREFIX_PATH "${LIBTORCH_PATH}")
    find_package(Torch REQUIRED)
endif ()

find_package(yaml-cpp REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp")
if (NOT yaml-cpp_FOUND)
    message(WARNING "yaml-cpp NOT FOUND")
endif ()

find_package(Eigen3 REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen")
if (NOT Eigen3_FOUND)
    message(WARNING "Eigen3 NOT FOUND")
endif ()

find_package(Boost REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost")
if (NOT Boost_FOUND)
    message(WARNING "Boost NOT FOUND")
endif ()

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")


find_package(OpenMP)
if (OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found")
    message(STATUS "OpenMP_CXX_FLAGS: ${OpenMP_CXX_FLAGS}")
    message(STATUS "USED FOR Eigen multithreading")
else ()
    message(WARNING "OpenMP NOT found, so it will not be used for Eigen multithreading")
endif ()

# Find the TBB package
find_package(TBB REQUIRED)
if (TBB_FOUND)
    message(STATUS "TBB found")
    message(STATUS "TBB_INCLUDE_DIRS: ${TBB_INCLUDE_DIRS}")
    message(STATUS "TBB_LIBRARIES: ${TBB_LIBRARIES}")
    message(STATUS "USED FOR multithreading")
else ()
    message(WARNING "TBB NOT found, so it will not be used for multithreading")
endif ()

# Needed for boost
set(BoostRoot "${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost")


message(STATUS "TORCH_INCLUDE_DIRS: ${TORCH_INCLUDE_DIRS}")
message(STATUS "TORCH_LIBRARIES: ${TORCH_LIBRARIES}")

# This is for the executables
#include_directories(
#        ${CMAKE_CURRENT_SOURCE_DIR}/include
#        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen
#        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp/include
#        #        ${TORCH_INCLUDE_DIRS}
#        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch/include
#        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch/include/torch/csrc/api/include
#        #        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost
#        ${Boost_INCLUDE_DIRS} # Add this line
#)

# Making executables
add_executable(${THE_EXE_NAME} main.cpp)
add_executable(TorchTest TorchTest.cpp)

message(STATUS "${BoldBlue}Making bhs_game_lib${ColourReset}")
# Specify the source files for your library
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
# Create the library (change "SHARED" to "STATIC" if you want a static library)
add_library(bhs_game_lib STATIC ${SOURCES})
# Specify the include directories for your library
target_include_directories(bhs_game_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch/include/torch/csrc/api/include>
        )
## Set the relative paths as include directories
#target_include_directories(bhs_game_lib INTERFACE
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch/include>
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch/include/torch/csrc/api/include>
#        )

# Link third-party libraries to your library
#find_package(ThirdPartyLib1 REQUIRED)
#find_package(ThirdPartyLib2 REQUIRED)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)

target_link_libraries(bhs_game_lib PUBLIC
        #        ThirdPartyLib1::ThirdPartyLib1
        #        ThirdPartyLib2::ThirdPartyLib2
        yaml-cpp
        eigen
        dl
        OpenMP::OpenMP_CXX
        TBB::tbb
        )

# Install your library and export it for use in other projects
install(TARGETS bhs_game_lib EXPORT bhs_game_lib-config
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(DIRECTORY include/ DESTINATION include)

install(EXPORT bhs_game_lib-config
        FILE bhs_game_lib_config.cmake
        NAMESPACE BHS::
        DESTINATION lib/cmake/bhs_game_lib
        )

export(TARGETS bhs_game_lib
        FILE "${CMAKE_CURRENT_BINARY_DIR}/bhs_game_lib-config.cmake"
        NAMESPACE BHS::
        )

# Finishing the executables
target_link_libraries(${THE_EXE_NAME} PRIVATE
        bhs_game_lib
        yaml-cpp
        eigen
        ${TORCH_LIBRARIES}
        dl
        OpenMP::OpenMP_CXX)
#
##message(WARNING "TORCH_LIBRARIES: ${TORCH_LIBRARIES}")
target_link_libraries(TorchTest PRIVATE
        bhs_game_lib
        yaml-cpp
        eigen
        ${TORCH_LIBRARIES}
        dl
        OpenMP::OpenMP_CXX)