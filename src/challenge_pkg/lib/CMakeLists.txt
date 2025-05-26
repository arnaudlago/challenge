add_library(pure_pursuit_lib
  pure_pursuit.cpp
)

target_include_directories(pure_pursuit_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
  ${catkin_INCLUDE_DIRS}
)