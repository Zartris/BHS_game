file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(bhs_game STATIC ${SOURCES})

target_include_directories(bhs_game PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
