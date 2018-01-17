
link_directories(${KTPROT_LINK_DIRECTORIES})
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${KTPROT_INCLUDE_DIRS}
)

########## Koala

add_executable(koala_camera_node koala_camera_node.cpp)
target_link_libraries(koala_camera_node 
  ${catkin_LIBRARIES}  ktprot
  )

add_executable(koala_camera_position_node koala_camera_position_node.cpp)
target_link_libraries(koala_camera_position_node 
  ${catkin_LIBRARIES}  ktprot
  )

add_executable(koala_wheels_node koala_wheels_node.cpp)
target_link_libraries(koala_wheels_node 
  ${catkin_LIBRARIES}  ktprot
  )

add_executable(koala_odom_node koala_odom_node.cpp)
target_link_libraries(koala_odom_node 
  ${catkin_LIBRARIES}  ktprot
  )

########## Khepera
add_executable(khepera_wheels_node khepera_wheels_node.cpp)
target_link_libraries(khepera_wheels_node 
  ${catkin_LIBRARIES}  ktprot
  )

# add_executable(khepera_ir_node khepera_ir_node.cpp)
# target_link_libraries(khepera_ir_node 
#   ${catkin_LIBRARIES}  ktprot
#   )
