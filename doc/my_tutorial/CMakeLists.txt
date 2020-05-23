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
# set_target_properties(my_tutorial_misc  PROPERTIES  VERSION  "${my_tutorial_misc_VERSION}")

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

add_executable(map_pair src/map_pair.cpp)
target_link_libraries(map_pair  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS map_pair 
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(test_misc src/test_misc.cpp)
target_link_libraries(test_misc ${catkin_LIBRARIES} ${Boost_LIBRARIES} misc_lib)
install(TARGETS test_misc 
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})