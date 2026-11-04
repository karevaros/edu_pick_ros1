cmake_minimum_required(VERSION 3.0.2)
project(edu_pick_task)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/pick_place.py scripts/go_pose.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
