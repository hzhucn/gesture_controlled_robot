
add_library(kteams_driver
  koala_video.cpp
  koala_wheels.cpp
  koala_camera_position.cpp
  khepera_wheels_ir.cpp
  )
target_link_libraries(kteams_driver ${catkin_LIBRARIES} jpeg ktprot)
