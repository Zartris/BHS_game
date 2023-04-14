cmake_minimum_required(VERSION 3.25 FATAL_ERROR)
project(bhs_game VERSION 1.0.0 LANGUAGES CXX C CUDA)

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
    message("Building in debug mode")
    add_definitions(-DDEBUG -URELEASE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
    # comment this if you want to remove the verbose output
#    add_definitions(-DVERBOSE)
elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
    message("Building in release mode")
    add_definitions(-DRELEASE -UVERBOSE -UDEBUG)
    # comment this if you want to remove the verbose output
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    message("Building in release mode")
    add_definitions(-DRELEASE -DVERBOSE-UDEBUG)
    # comment this if you want to remove the verbose output
else ()
    message("Building in something mode???????")

    # comment this if you want to remove the verbose output
    #    add_definitions(-DVERBOSE)
endif ()
# allow for the use of the -pthread flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

# The following needs to be set by whom ever is building this
set(LIBTORCH_VERSION "2.0.0") # Change this to your desired version
set(LIBTORCH_URL "https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}%2Bcu118.zip") # Update this with the URL you chose

set(LIBTORCH_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch")

if (NOT EXISTS ${LIBTORCH_PATH})
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
endif ()

# Make the LibTorch library work:
set(CMAKE_PREFIX_PATH "${LIBTORCH_PATH}")
# CUDA PARAMS:
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-12.1)
set(CUDA_COMPILER /usr/local/cuda-12.1/bin/nvcc)

# Enable CUDNN
set(USE_CUDNN ON)
set(CAFFE2_USE_CUDNN ON)

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
find_package(Torch REQUIRED COMPONENTS CUDA CUDNN)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")


# Needed for boost
set(BoostRoot "${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost")

# Clean up the LibTorch path from previous builds
#file(REMOVE_RECURSE "${LIBTORCH_PATH}.zip")
include_directories(
        ${CMAKE_SOURCE_DIR}/include
        #        ${CMAKE_SOURCE_DIR}/third_party/eigen
        ${CMAKE_SOURCE_DIR}/third_party/yaml-cpp/include
        ${TORCH_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/third_party/boost
)


set(SOURCE_FILES main.cpp)

add_executable(${THE_EXE_NAME} ${SOURCE_FILES})
add_executable(TorchTest TorchTest.cpp)

add_subdirectory(src)
#add_subdirectory(third_party/eigen)
add_subdirectory(third_party/yaml-cpp)
add_subdirectory(third_party/boost)

target_link_libraries(${THE_EXE_NAME} PRIVATE bhs_game_lib yaml-cpp ${TORCH_LIBRARIES} dl)
target_link_libraries(TorchTest PRIVATE bhs_game_lib yaml-cpp ${TORCH_LIBRARIES} dl)

enable_testing()
add_subdirectory(tests)
