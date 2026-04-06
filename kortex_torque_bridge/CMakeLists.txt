cmake_minimum_required(VERSION 3.14)
include(FetchContent)
FetchContent_Declare(
  kinova_binary_api
  URL https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.5.0/linux_x86-64_x86_gcc.zip
  URL_HASH MD5=64bd86e7ab8bda90ef1fc7d6a356e080
)
FetchContent_MakeAvailable(kinova_binary_api)

project(kortex_torque_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Required by the Kortex API on Linux
add_definitions(-D_OS_UNIX=1)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kortex_api REQUIRED)

if(NOT (CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64"))
  message(FATAL_ERROR "Unsupported System: currently support is for Linux x86_64. Detected ${CMAKE_SYSTEM_NAME} and ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Import the prebuilt Kortex static library (same pattern as kortex_driver)
add_library(KortexApiCpp STATIC IMPORTED)
set_target_properties(KortexApiCpp PROPERTIES
  IMPORTED_LOCATION ${kinova_binary_api_SOURCE_DIR}/lib/release/libKortexApiCpp.a
  INTERFACE_LINK_LIBRARIES KortexApiCpp
)
target_link_libraries(KortexApiCpp INTERFACE pthread)
add_dependencies(KortexApiCpp kortex_api)

# The node executable
add_executable(kortex_torque_node src/kortex_torque_node.cpp)

target_include_directories(kortex_torque_node PRIVATE include)

target_link_libraries(kortex_torque_node KortexApiCpp)

ament_target_dependencies(kortex_torque_node
  SYSTEM kortex_api
  rclcpp
  std_msgs
)

install(TARGETS kortex_torque_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
endif()

ament_package()
