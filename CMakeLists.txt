cmake_minimum_required(VERSION 3.20)

project(FRCSimPhysicsLibrary VERSION 0.1.0 LANGUAGES CXX)

option(FRCSIM_BUILD_EXAMPLES "Build examples" ON)
option(FRCSIM_BUILD_TESTS "Build tests" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(core/physics-core)

if(FRCSIM_BUILD_EXAMPLES)
	add_executable(frcsim_minimal_world examples/cpp/minimal_world.cpp)
	target_link_libraries(frcsim_minimal_world PRIVATE frcsim_physics)
endif()

if(FRCSIM_BUILD_TESTS)
	add_executable(frcsim_rigidbody_test vendordep/tests/rigidbody_test.cpp)
	target_link_libraries(frcsim_rigidbody_test PRIVATE frcsim_physics)

	add_executable(frcsim_magnus_test vendordep/tests/magnus_test.cpp)
	target_link_libraries(frcsim_magnus_test PRIVATE frcsim_physics)
endif()
