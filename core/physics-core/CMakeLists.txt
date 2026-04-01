add_library(frcsim_physics
    src/physics_world.cpp
)

target_include_directories(frcsim_physics
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(frcsim_physics PUBLIC cxx_std_20)
