#file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
#
#add_executable(test_bhs_game ${TEST_SOURCES})
#
#target_include_directories(test_bhs_game PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
#target_link_libraries(test_bhs_game bhs_game)
#
#add_test(NAME bhs_game_tests COMMAND test_bhs_game)
#
