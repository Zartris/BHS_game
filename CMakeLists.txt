cmake_minimum_required(VERSION 3.25 FATAL_ERROR)
project(bhs_game VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
#
#set(CMAKE_CUDA_FLAGS "-std=c++20")

#set(CMAKE_C_COMPILER /usr/bin/gcc-9)
#set(CMAKE_CXX_COMPILER /usr/bin/g++-9)

#set(CMAKE_CXX23_STANDARD_COMPILE_OPTION "-std=c++23")
#set(CMAKE_CXX23_EXTENSION_COMPILE_OPTION "-std=gnu++23")
set(THE_EXE_NAME bhs_game_executable)

include(ExternalProject)

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
#set(CMAKE_CUDA_HOST_COMPILER /usr/bin/clang-8)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-12.1)
set(CUDA_COMPILER /usr/local/cuda-12.1/bin/nvcc)
set(USE_CUDNN ON)

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
find_package(Torch REQUIRED COMPONENTS CUDA)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# Clean up the LibTorch path from previous builds
#file(REMOVE_RECURSE "${LIBTORCH_PATH}.zip")
include_directories(
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/third_party/eigen
        ${CMAKE_SOURCE_DIR}/third_party/yaml-cpp/include
)


set(SOURCE_FILES main.cpp)

add_executable(${THE_EXE_NAME} ${SOURCE_FILES})
add_dependencies(${THE_EXE_NAME} libtorch)

add_subdirectory(src)
add_subdirectory(third_party/eigen)
add_subdirectory(third_party/yaml-cpp)

target_link_libraries(${THE_EXE_NAME} PRIVATE bhs_game_lib yaml-cpp ${TORCH_LIBRARIES})

enable_testing()
add_subdirectory(tests)
