#  ----------------------------------------------------------------------------
#  2020 Bernd Pfrommer bernd.pfrommer@gmail.com
#

cmake_minimum_required(VERSION 3.5)
project(basalt_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# for libraries with missing cmake files
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/"
  ${CMAKE_MODULE_PATH})
  
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release)
endif()

if(NOT CXX_MARCH)
  set(CXX_MARCH native)
endif()



# Flags used for CHECK_CXX_SOURCE_COMPILES
set(CMAKE_REQUIRED_FLAGS "-Wno-error")


# save flags passed by user
set(BASALT_PASSED_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")          # cmake default: "-g"
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")  # cmake default: "-O2 -g -DNDEBUG"
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")                                      # cmake default: "-O3 -DNDEBUG"
set(CMAKE_CXX_FLAGS_CIDEBUG  "-O0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN")          # CI version with no debug symbols
set(CMAKE_CXX_FLAGS_CIRELWITHDEBINFO "-O3 -DEIGEN_INITIALIZE_MATRICES_BY_NAN")  # CI version with no debug symbols

# base set of compile flags
set(BASALT_CXX_FLAGS "-Wall -Wextra -Werror -Wno-error=unused-parameter -ftemplate-backtrace-limit=0")


# OpenMP option and compile flags
option(USE_OPENMP "Use OpenMP (e.g. for parallel computation in Eigen)" ${USE_OPENMP_DEFAULT})
if(USE_OPENMP)
  message(STATUS "OpenMP Enabled")
  set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -fopenmp")
else()
  message(STATUS "OpenMP Disabled")
endif()


# setup combined compiler flags
set(CMAKE_CXX_FLAGS "-std=c++17 ${BASALT_CXX_FLAGS} -march=${CXX_MARCH} ${BASALT_PASSED_CXX_FLAGS}")

# message(WARN "cmake flags: ${CMAKE_CXX_FLAGS}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(basalt_vio_ros2_msgs REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED core imgproc calib3d highgui)
find_package(Basalt CONFIG REQUIRED)

#
# Unless explicitly set otherwise, use Eigen headers
# provided by Basalt. Note that mixing different versions
# of Eigen leads to nasty memory corruption bugs.
#
if(NOT EIGEN_ROOT)
  set(EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../basalt/thirdparty/basalt-headers/thirdparty/eigen")
endif()
set(EIGEN_INCLUDE_DIR_HINTS ${EIGEN_ROOT})
find_package(Eigen3 3.3.7 EXACT REQUIRED MODULE)

include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")
if(NOT EIGEN3_INCLUDE_DIR MATCHES "^${EIGEN_ROOT}")
  message(STATUS "Found Eigen headers in  ${EIGEN3_INCLUDE_DIR} outside of specified EIGEN_ROOT '${EIGEN_ROOT}'")
endif()

set(ROS2_DEPENDENCIES
  "rclcpp"
  "rclcpp_components"
  "nav_msgs"
  "sensor_msgs"
  "message_filters"
  "image_transport"
  "message_filters"
  "tf2"
  "tf2_ros"
  "basalt_vio_ros2_msgs"
  "std_msgs"
)

ament_auto_find_build_dependencies(REQUIRED ${ROS2_DEPENDENCIES})

include_directories(${TBB_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${BASALT_INCLUDE_DIRS})

# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#     message(WARNING "VARIABLE: ${_variableName}=${${_variableName}}")
# endforeach()

include_directories(include)

include_directories(../basalt/thirdparty/basalt-headers/thirdparty/eigen/include)
include_directories(../basalt/thirdparty/basalt-headers/thirdparty/Sophus)
include_directories(../basalt/thirdparty/basalt-headers/include)


ament_auto_add_executable(basalt_vio_node
src/vio_frontend.cpp
src/vio_backend.cpp
src/vio_node.cpp
src/vio_publisher.cpp
src/image_subscriber.cpp
src/imu_subscriber.cpp
src/imu_publisher.cpp
src/optical_flow_publisher.cpp
src/optical_flow_subscriber.cpp
)

target_link_libraries(basalt_vio_node ${BASALT_LIBRARIES} ${TBB_LIBRARIES})

ament_auto_add_executable(basalt_vio_backend_node
src/vio_backend_node.cpp
src/vio_backend.cpp
src/vio_publisher.cpp
src/imu_subscriber.cpp
src/imu_publisher.cpp
src/optical_flow_subscriber.cpp
)

target_link_libraries(basalt_vio_backend_node ${BASALT_LIBRARIES} ${TBB_LIBRARIES})

ament_auto_add_executable(test_subscriber
src/test_subscriber.cpp
src/imu_subscriber.cpp
)

target_link_libraries(test_subscriber ${BASALT_LIBRARIES} ${TBB_LIBRARIES})


install(TARGETS
    basalt_vio_node
    basalt_vio_backend_node
    test_subscriber
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)


ament_package()

