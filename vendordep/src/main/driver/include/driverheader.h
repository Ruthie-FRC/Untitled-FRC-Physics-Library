#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void c_doThing();

unsigned long long c_rsCreateWorld(double fixed_dt_s, int enable_gravity);
void c_rsDestroyWorld(unsigned long long world_handle);

int c_rsCreateBody(unsigned long long world_handle, double mass_kg);
int c_rsSetBodyPosition(unsigned long long world_handle, int body_index, double x_m, double y_m, double z_m);
int c_rsSetBodyLinearVelocity(unsigned long long world_handle, int body_index, double vx_mps, double vy_mps,
							  double vz_mps);
int c_rsSetBodyGravityEnabled(unsigned long long world_handle, int body_index, int enabled);

int c_rsStepWorld(unsigned long long world_handle, int steps);
int c_rsSetWorldGravity(unsigned long long world_handle, double gx_mps2, double gy_mps2, double gz_mps2);

int c_rsGetBodyPosition(unsigned long long world_handle, int body_index, double* x_m, double* y_m, double* z_m);
int c_rsGetBodyLinearVelocity(unsigned long long world_handle, int body_index, double* vx_mps, double* vy_mps,
							  double* vz_mps);

#ifdef __cplusplus
}  // extern "C"
#endif
