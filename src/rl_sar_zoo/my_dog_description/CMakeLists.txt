cmake_minimum_required(VERSION 3.5)
project(my_dog_description VERSION 3.0.0)
if(POLICY CMP0009)
  cmake_policy(SET CMP0009 NEW)
endif()

if($ENV{ROS_DISTRO} MATCHES "noetic")
	find_package(catkin REQUIRED)
	catkin_package()
	find_package(roslaunch)
	foreach(dir config launch meshes urdf xacro)
        if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
            install(
                DIRECTORY ${dir}/
                DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
            )
        endif()
	endforeach(dir)
elseif($ENV{ROS_DISTRO} MATCHES "foxy|humble")
    find_package(ament_cmake REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    find_package(urdf REQUIRED)
    find_package(xacro REQUIRED)
    find_package(robot_state_publisher REQUIRED)
    message(STATUS "正在处理目录: ${dir}")
    message(STATUS "源目录存在: ${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    message(STATUS "目标目录: share/${PROJECT_NAME}/${dir}")
    foreach(dir config launch meshes urdf xacro)
        if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
            install(
                DIRECTORY ${dir}/
                DESTINATION share/${PROJECT_NAME}/${dir}

            )
        endif()
    endforeach()
    ament_package()
endif()