file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_executable(bhs_game ${SOURCES})
target_link_libraries(bhs_game PRIVATE Eigen3::Eigen)

target_include_directories(bhs_game PRIVATE ${CMAKE_SOURCE_DIR}/include)