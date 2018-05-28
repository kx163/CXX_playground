# add support for C++ 11 standard
set(CMAKE_CXX_FLAGS "-std=c++11")

# find OpenCV package
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENT common io)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# compile program
add_executable(imageBasics imageBasics.cpp)
target_link_libraries(imageBasics ${OpenCV_LIBRARIES})

add_executable(joinMap joinMap.cpp)
target_link_libraries(joinMap ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})