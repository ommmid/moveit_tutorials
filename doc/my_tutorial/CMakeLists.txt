include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM
  ${Boost_INCLUDE_DIRS}
)

# the first arg of add_library is the name of the library in devel/lib folder. 
# we do not use this name for "#include". "#include" should use the name of the header file
add_library(misc_lib src/misc.cpp)
target_link_libraries(misc_lib  ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(my_planning_scene_tutorial src/my_planning_scene_tutorial.cpp)
# the first argument of this needs to be created by add_executable() or add_library()
target_link_libraries(my_planning_scene_tutorial  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# install the executable (my_planning_scene_tutorial) to the destination 
install(TARGETS my_planning_scene_tutorial 
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install the contents of DIRECTORY (launch) to the destination 
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(my_move_group_interface_tutorial src/my_move_group_interface_tutorial.cpp)
target_link_libraries(my_move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES} misc_lib)
install(TARGETS my_move_group_interface_tutorial 
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(collision_object src/collision_object.cpp)
target_link_libraries(collision_object  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS collision_object
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(test_misc src/test_misc.cpp)
target_link_libraries(test_misc ${catkin_LIBRARIES} ${Boost_LIBRARIES} misc_lib)
install(TARGETS test_misc 
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(trajopt_example_with_collision src/trajopt_example_with_collision.cpp)
target_link_libraries(trajopt_example_with_collision ${catkin_LIBRARIES} ${Boost_LIBRARIES} misc_lib)
install(TARGETS trajopt_example_with_collision 
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(check_collision src/check_collision.cpp)
target_link_libraries(check_collision ${catkin_LIBRARIES} ${Boost_LIBRARIES} misc_lib)
install(TARGETS check_collision
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(my_visualizing_collisions_tutorial src/my_visualizing_collisions_tutorial.cpp)
target_link_libraries(my_visualizing_collisions_tutorial
        ${interactive_markers_LIBRARIES}
        interactivity_utils
        ${catkin_LIBRARIES}
         ${Boost_LIBRARIES}
          misc_lib)
install(TARGETS my_visualizing_collisions_tutorial
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
