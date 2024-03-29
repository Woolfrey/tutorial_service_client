cmake_minimum_required(VERSION 3.8)
project(tutorial_service_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################################################################################################
#                                           Dependencies                                           #
####################################################################################################
find_package(ament_cmake REQUIRED)                                                                  # Used to build the package
find_package(rclcpp REQUIRED)                                                                       # Standard ROS2 C++ libraries
find_package(tutorial_service_definition REQUIRED)                                                  # Custom built

if(BUILD_TESTING)
     find_package(ament_lint_auto REQUIRED)
     set(ament_cmake_copyright_FOUND TRUE)
     set(ament_cmake_cpplint_FOUND TRUE)
     ament_lint_auto_find_test_dependencies()
endif()

####################################################################################################
#                                      Generate Services                                           #
####################################################################################################

add_executable(haiku_server src/haiku_server.cpp)
ament_target_dependencies(haiku_server
                          "rclcpp"
                          "tutorial_service_definition")
                          
add_executable(haiku_client src/haiku_client.cpp)
ament_target_dependencies(haiku_client
                          "rclcpp"
                          "tutorial_service_definition")

# This is needed so ROS can find the package
install(TARGETS
        haiku_server
        haiku_client
        DESTINATION lib/${PROJECT_NAME})
        
        
ament_package()

