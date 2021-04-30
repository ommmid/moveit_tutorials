add_executable(trajopt_example src/trajopt_example.cpp)
target_link_libraries(trajopt_example  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(publisher_to_panda src/publisher_to_panda.cpp)
target_link_libraries(publisher_to_panda  ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS trajopt_example DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
