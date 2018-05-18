find_package( Eigen3 3.3 REQUIRED NO_MODULE )

add_executable( play play.cpp )
target_link_libraries( play Eigen3::Eigen )