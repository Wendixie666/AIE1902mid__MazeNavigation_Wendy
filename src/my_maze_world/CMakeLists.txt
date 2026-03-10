cmake_minimum_required(VERSION 2.8.3)
project(my_maze_world)

find_package(catkin REQUIRED)

catkin_package()

# Install scripts
install(PROGRAMS scripts/dynamic_obstacles.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install worlds
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)
