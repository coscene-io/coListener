cmake_minimum_required(VERSION 3.10.2)
project(colistener)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 11)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

macro(enable_strict_compiler_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /WX /W4)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror -Wold-style-cast -Wfloat-equal
                -Wmost -Wunused-exception-parameter)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror -Wold-style-cast -Wfloat-equal)
    endif()
endmacro()

find_package(catkin QUIET)
find_package(ament_cmake QUIET)

message(STATUS "ROS_VERSION: " $ENV{ROS_VERSION})
message(STATUS "ROS_DISTRO: " $ENV{ROS_DISTRO})
message(STATUS "ROS_ROOT: " $ENV{ROS_ROOT})

find_program(GIT_SCM git DOC "Git version control")
if(GIT_SCM)
    execute_process(
            COMMAND ${GIT_SCM} describe --always --dirty --exclude="*"
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            OUTPUT_VARIABLE GIT_HASH
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()
set(COLISTENER_VERSION "${CMAKE_PROJECT_VERSION}")
configure_file(version.cpp.in
        version.cpp @ONLY)

add_library(colistener_base SHARED
        ${CMAKE_CURRENT_BINARY_DIR}/version.cpp
)

target_include_directories(colistener_base
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

find_package(nlohmann_json QUIET)
if(nlohmann_json_FOUND)
    message(STATUS "nlohmann_json was found")
    target_link_libraries(colistener_base nlohmann_json::nlohmann_json)
else()
    message(STATUS "nlohmann_json not found, will search at compile time")
endif()

find_package(CURL REQUIRED)

if("$ENV{ROS_VERSION}" STREQUAL "1")
    if(catkin_FOUND)
        message(STATUS "Building with catkin")
        set(ROS_BUILD_TYPE "catkin")

        find_package(catkin REQUIRED COMPONENTS
            roslib roscpp topic_tools)
        find_package(Boost REQUIRED)

        catkin_package(
                INCLUDE_DIRS ros1_listener/include
                LIBRARIES colistener_base ros1_listener
                CATKIN_DEPENDS roslib roscpp topic_tools
                DEPENDS Boost
        )

        add_library(ros1_listener
            ros1_listener/src/listener.cpp
        )

        message(STATUS "CMAKE_CURRENT_SOURCE_DIR: " ${CMAKE_CURRENT_SOURCE_DIR})
        target_include_directories(ros1_listener
                SYSTEM PRIVATE
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ros1_listener/include>
                $<INSTALL_INTERFACE:include>
                ${catkin_INCLUDE_DIRS}
        )

        target_link_libraries(ros1_listener colistener_base ${CURL_LIBRARIES} ${catkin_LIBRARIES})
        enable_strict_compiler_warnings(ros1_listener)

        add_executable(colistener ros1_listener/src/main.cpp)

        target_include_directories(colistener
                SYSTEM PRIVATE
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ros1_listener/include>
                ${catkin_INCLUDE_DIRS})
        target_link_libraries(colistener ros1_listener ${catkin_LIBRARIES})

        enable_strict_compiler_warnings(colistener)

    else()
        message(FATAL_ERROR "Could not find catkin")
    endif()
elseif("$ENV{ROS_VERSION}" STREQUAL "2")

    set(CMAKE_CXX_STANDARD 14)

    if("$ENV{ROS_DISTRO}" STREQUAL "foxy")
        add_compile_definitions(ROS2_VERSION_FOXY)
    else()
        add_compile_definitions(ROS2_VERSION_HUMBLE)
    endif()

    if(ament_cmake_FOUND)
        message(STATUS "Building with ament_cmake")
        set(ROS_BUILD_TYPE "ament_cmake")

        find_package(ament_cmake REQUIRED)
        find_package(rosgraph_msgs REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(rclcpp_components REQUIRED)
        find_package(resource_retriever REQUIRED)


        add_library(ros2_listener SHARED
                ros2_listener/src/typesupport_helpers.cpp
                ros2_listener/src/listener.cpp
                ros2_listener/src/generic_subscription.cpp
        )

        target_include_directories(ros2_listener
                PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ros2_listener/include>
                $<INSTALL_INTERFACE:include>
        )

        ament_target_dependencies(ros2_listener rclcpp rclcpp_components resource_retriever rosgraph_msgs)
        target_link_libraries(ros2_listener colistener_base ${CURL_LIBRARIES})
        enable_strict_compiler_warnings(ros2_listener)
        add_executable(colistener
                ros2_listener/src/main.cpp
        )
        target_include_directories(colistener
                SYSTEM PRIVATE
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ros2_listener/include>
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                ${rclcpp_INCLUDE_DIRS}
        )
        target_link_libraries(colistener ros2_listener)
        ament_target_dependencies(colistener rclcpp)
        enable_strict_compiler_warnings(colistener)

    else()
        message(FATAL_ERROR "Could not find ament_cmake")
    endif()
else()
    message(FATAL_ERROR "ROS_VERSION environment variable must be 1 or 2")
endif()


if(ROS_BUILD_TYPE STREQUAL "catkin")
    if(CATKIN_ENABLE_TESTING)
        message(STATUS "Building tests with catkin")
    endif()
elseif(ROS_BUILD_TYPE STREQUAL "ament_cmake")
    if(BUILD_TESTING)
        message(STATUS "Building tests with ament_cmake")
    endif()
endif()


if(ROS_BUILD_TYPE STREQUAL "catkin")
    message(STATUS "catkin install")
    install(TARGETS colistener
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    install(TARGETS colistener_base ros1_listener
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )
    install(DIRECTORY ros1_listener/launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
elseif(ROS_BUILD_TYPE STREQUAL "ament_cmake")
    message(STATUS "ament install")
    install(FILES ros2_listener/include/listener.hpp
            DESTINATION include/${PROJECT_NAME}/
    )
    install(TARGETS colistener
            DESTINATION lib/${PROJECT_NAME}
    )
    install(TARGETS colistener_base ros2_listener
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
    )
    install(DIRECTORY ros2_listener/launch
            DESTINATION share/${PROJECT_NAME}/
    )
    ament_export_libraries(colistener_base ros2_listener)
    ament_package()
endif()
