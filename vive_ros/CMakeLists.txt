cmake_minimum_required(VERSION 3.8)
project(vive_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_definitions(-DLINUX -DLINUX64 -DPOSIX)
set(OPENVR "$ENV{HOME}/libraries/openvr")

add_definitions(-DOPENVR_PATH="${OPENVR}")
set(THIRDPARTY_DIR ${OPENVR}/samples/thirdparty)
set(SHARED_SRC_DIR ${OPENVR}/samples/shared)
#set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++17 -include ${SHARED_SRC_DIR}/compat.h")
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pedantic -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")


find_library(OPENVR_LIBRARIES
  NAMES
    openvr_api
  PATHS
    # ${OPENVR}/bin
    /usr/local/lib
  PATH_SUFFIXES
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)

set(OPENVR_INCLUDE_DIR /usr/local/include/openvr)


set(SDL_REQUIRED_LIBRARIES SDL2)
foreach(lib ${SDL_REQUIRED_LIBRARIES})
  find_library(${lib}_LIBRARY
    NAMES
      ${lib}
    PATHS
      ${THIRDPARTY_DIR}/sdl2-2.0.3/bin
    PATH_SUFFIXES
      linux64
    )
  list(APPEND SDL2_LIBRARIES ${${lib}_LIBRARY})
endforeach()
set(SDL2_INCLUDE_DIR ${THIRDPARTY_DIR}/sdl2-2.0.3/include)

file(GLOB SHARED_SRC_FILES
  ${SHARED_SRC_DIR}/*.cpp
  ${SHARED_SRC_DIR}/*.h
)


include_directories(
  ${SDL2_INCLUDE_DIR}
  ${OPENVR_INCLUDE_DIR}
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)


set(CMAKE_BUILD_TYPE "Release")

install(
        DIRECTORY include/
        DESTINATION include
)

add_executable(vive_node
  # ${SHARED_SRC_FILES}
  src/vive_node.cpp
  src/vr_interface.cpp
)

ament_target_dependencies(
  vive_node
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  std_srvs
)

#target_link_directories(vive_node PUBLIC ${OPENVR}/lib/linux64)

target_include_directories(vive_node PUBLIC ${OPENVR_INCLUDE_DIR})

target_link_libraries(vive_node
  #${SDL2_LIBRARIES}
  ${OPENVR_LIBRARIES}
  ${CMAKE_DL_LIBS}
  ${EXTRA_LIBS}
  ${rmw_implementation_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES})


install(TARGETS 
  vive_node
  DESTINATION lib/${PROJECT_NAME}
)
#install(
#  DIRECTORY ${OPENVR}/lib/linux64/
#  DESTINATION lib
#)
# install(
#   DIRECTORY ${OPENVR}/lib/linux64/
#   DESTINATION lib
# )

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS vive_node
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
