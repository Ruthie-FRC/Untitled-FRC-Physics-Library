cmake_minimum_required(VERSION 3.20)

project(FRCSimPhysicsLibrary VERSION 2026.04.03 LANGUAGES CXX)

include(CTest)

option(FRCSIM_BUILD_EXAMPLES "Build examples" ON)
option(FRCSIM_BUILD_TESTS "Build tests" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(frcsim_physics
	core/driver/src/physics_world.cpp
	core/driver/src/rigid_assembly.cpp
	core/driver/src/joints.cpp
)

target_include_directories(frcsim_physics
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/core/driver/include
)

target_compile_features(frcsim_physics PUBLIC cxx_std_20)

if(FRCSIM_BUILD_EXAMPLES)
add_executable(frcsim_minimal_world examples/cpp/minimal_world.cpp)
target_link_libraries(frcsim_minimal_world PRIVATE frcsim_physics)

add_executable(frcsim_modular_bodies examples/cpp/modular_bodies_demo.cpp)
target_link_libraries(frcsim_modular_bodies PRIVATE frcsim_physics)

add_executable(frcsim_robot_assembly examples/cpp/robot_assembly_demo.cpp)
target_link_libraries(frcsim_robot_assembly PRIVATE frcsim_physics)
endif()

if(FRCSIM_BUILD_TESTS)
add_executable(frcsim_rigidbody_test vendordep/tests/rigidbody_test.cpp)
target_link_libraries(frcsim_rigidbody_test PRIVATE frcsim_physics)
add_test(NAME frcsim_rigidbody_test COMMAND frcsim_rigidbody_test)

add_executable(frcsim_magnus_test vendordep/tests/magnus_test.cpp)
target_link_libraries(frcsim_magnus_test PRIVATE frcsim_physics)
add_test(NAME frcsim_magnus_test COMMAND frcsim_magnus_test)

add_executable(frcsim_modular_body_test vendordep/tests/modular_body_test.cpp)
target_link_libraries(frcsim_modular_body_test PRIVATE frcsim_physics)
add_test(NAME frcsim_modular_body_test COMMAND frcsim_modular_body_test)

# New comprehensive test suites
add_executable(frcsim_math_test vendordep/tests/math_test.cpp)
target_link_libraries(frcsim_math_test PRIVATE frcsim_physics)
add_test(NAME frcsim_math_test COMMAND frcsim_math_test)

add_executable(frcsim_integration_test vendordep/tests/integration_test.cpp)
target_link_libraries(frcsim_integration_test PRIVATE frcsim_physics)
add_test(NAME frcsim_integration_test COMMAND frcsim_integration_test)

add_executable(frcsim_assembly_test vendordep/tests/assembly_test.cpp)
target_link_libraries(frcsim_assembly_test PRIVATE frcsim_physics)
add_test(NAME frcsim_assembly_test COMMAND frcsim_assembly_test)

add_executable(frcsim_boundary_test vendordep/tests/boundary_test.cpp)
target_link_libraries(frcsim_boundary_test PRIVATE frcsim_physics)
add_test(NAME frcsim_boundary_test COMMAND frcsim_boundary_test)

add_executable(frcsim_deformable_test vendordep/tests/deformable_test.cpp)
target_link_libraries(frcsim_deformable_test PRIVATE frcsim_physics)
add_test(NAME frcsim_deformable_test COMMAND frcsim_deformable_test)

add_executable(frcsim_forces_test vendordep/tests/forces_test.cpp)
target_link_libraries(frcsim_forces_test PRIVATE frcsim_physics)
add_test(NAME frcsim_forces_test COMMAND frcsim_forces_test)
endif()
