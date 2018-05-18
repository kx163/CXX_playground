add_executable( helloSlambook helloSlambook.cpp )

add_library( HelloSlambook SHARED libHelloSlambook.cpp )
add_executable( useHello useHello.cpp )
target_link_libraries( useHello HelloSlambook )