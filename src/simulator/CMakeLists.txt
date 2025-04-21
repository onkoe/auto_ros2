cmake_minimum_required(VERSION 3.8)
project(simulator)

# build the package
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)

# install a bunch of stuff
install(DIRECTORY
    launch #...launch files
    resource # ...resources (used for sim models/etc.)
    params # ...launch parameter files
    DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(${PROJECT_NAME})

# add a script to act like a binary for the cmake build
install(PROGRAMS
  scripts/simulator
  DESTINATION lib/${PROJECT_NAME}
)

# test stuff
if(BUILD_TESTING)
    find_package(launch_testing_ament_cmake)
    add_launch_test(test/test_launch_sim_works.py)
endif()

ament_package()
