file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(bhs_game_lib STATIC ${SOURCES})
#target_link_libraries(bhs_game_lib PRIVATE Eigen3::Eigen)

target_include_directories(bhs_game_lib PRIVATE ${CMAKE_SOURCE_DIR}/include)
