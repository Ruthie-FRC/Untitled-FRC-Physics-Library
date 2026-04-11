// Copyright (c) Jsim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void c_doThing(void);

uint64_t c_rsCreateWorld(double fixed_dt_s, int enable_gravity);
void c_rsDestroyWorld(uint64_t world_handle);

int c_rsCreateBody(uint64_t world_handle, double mass_kg);
int c_rsSetBodyPosition(uint64_t world_handle, int body_index,
                        double x_m, double y_m, double z_m);
int c_rsSetBodyLinearVelocity(uint64_t world_handle, int body_index,
                              double vx_mps, double vy_mps, double vz_mps);
int c_rsSetBodyGravityEnabled(uint64_t world_handle, int body_index,
                              int enabled);

int c_rsStepWorld(uint64_t world_handle, int steps);
int c_rsSetWorldGravity(uint64_t world_handle, double gx_mps2,
                        double gy_mps2, double gz_mps2);

int c_rsGetBodyPosition(uint64_t world_handle, int body_index,
                        double* x_m, double* y_m, double* z_m);
int c_rsGetBodyLinearVelocity(uint64_t world_handle, int body_index,
                              double* vx_mps, double* vy_mps, double* vz_mps);

#ifdef __cplusplus
}  // extern "C"
#endif
