cmake_minimum_required(VERSION 2.8.3)
project(uuv_trajectory_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/cascaded_pid_dp_controller.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest)
  foreach(T
      test/test_all.launch
      )
     add_rostest(${T})
  endforeach()

  catkin_add_nosetests(test/test_waypoint.py)
  catkin_add_nosetests(test/test_trajectory_point.py)
  catkin_add_nosetests(test/test_waypoint_set.py)
endif()
