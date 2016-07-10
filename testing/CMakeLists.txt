
find_package(Boost REQUIRED COMPONENTS
  thread
)

add_executable(collaborative_test
  src/collaborative_test.cc
)

target_link_libraries(collaborative_test
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)