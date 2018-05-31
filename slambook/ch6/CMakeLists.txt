# add support for C++ 11 standard
set(CMAKE_CXX_FLAGS "-std=c++11")

# add cmake module for G2O
list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules )

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})

add_executable(ceres_curve_fitting ceres_curve_fitting.cpp)
target_link_libraries(ceres_curve_fitting ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(g2o_curve_fitting g2o_curve_fitting.cpp)
target_link_libraries(g2o_curve_fitting ${OpenCV_LIBRARIES} g2o_core g2o_stuff)

add_executable(ceres_exercice ceres_exercice.cpp)
target_link_libraries(ceres_exercice ${CERES_LIBRARIES})

add_executable(g2o_exercice g2o_exercice.cpp)
target_link_libraries(g2o_exercice g2o_core g2o_stuff)