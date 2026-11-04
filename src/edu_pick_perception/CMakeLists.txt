cmake_minimum_required(VERSION 3.0.2)
project(edu_pick_perception)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/object_detector.py scripts/fake_camera.py scripts/d405_camera.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
