// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <stdint.h>

/**
 * @file driverheader.h
 * @brief Stable C ABI for constructing, configuring, stepping, and querying JSim worlds.
 *
 * This header is intentionally C-compatible so it can be consumed from JNI,
 * Python/ctypes, C#, Rust FFI, and other foreign-function interfaces without
 * requiring C++ name mangling support.
 *
 * General conventions:
 * - All distances are meters.
 * - All velocities are meters per second.
 * - All angular quantities are radians or radians/second.
 * - All masses are kilograms.
 * - All time values are seconds.
 * - A world handle value of 0 is invalid.
 * - Functions returning `int` use 0 for success and non-zero for failure.
 * - Indices are zero-based and must reference an existing body.
 * - Pointer out-parameters must be non-null unless explicitly documented as optional.
 *
 * Error model:
 * - Invalid handle, invalid body index, invalid pointer, or invalid numeric input
 *   results in a non-zero error code.
 * - APIs are best-effort deterministic and do not throw C++ exceptions across this ABI.
 *
 * Threading model:
 * - World mutation functions are not re-entrant for the same world handle.
 * - Callers should serialize writes and step operations per world.
 * - Query-only calls should not race with mutating calls unless external locking is used.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief No-op sanity probe for link/load validation.
 *
 * This function exists primarily as a simple symbol to validate library load
 * paths in integration tests and downstream wrappers.
 */
void c_doThing(void);

/**
 * @brief Creates a new physics world and returns its opaque handle.
 * @param fixed_dt_s Fixed simulation timestep in seconds. Typical FRC values are 0.005-0.02.
 * @param enable_gravity Non-zero enables gravity by default; zero disables it.
 * @return Opaque non-zero world handle on success; 0 on failure.
 */
uint64_t c_rsCreateWorld(double fixed_dt_s, int enable_gravity);

/**
 * @brief Destroys a previously created world and releases all associated resources.
 * @param world_handle Opaque world handle returned by c_rsCreateWorld().
 *
 * Safe behavior:
 * - Passing an invalid handle is ignored or treated as an error internally,
 *   but callers should not rely on double-destroy semantics.
 * - After destruction, all body indices and references from that world are invalid.
 */
void c_rsDestroyWorld(uint64_t world_handle);

/**
 * @brief Creates a new rigid body in a world.
 * @param world_handle Target world handle.
 * @param mass_kg Body mass in kilograms.
 * @return Non-negative body index on success; negative value on failure.
 */
int c_rsCreateBody(uint64_t world_handle, double mass_kg);

/**
 * @brief Sets a body's world-space position.
 * @param world_handle Target world handle.
 * @param body_index Zero-based body index returned by c_rsCreateBody().
 * @param x_m Position x in meters.
 * @param y_m Position y in meters.
 * @param z_m Position z in meters.
 * @return 0 on success, non-zero on failure.
 */
int c_rsSetBodyPosition(uint64_t world_handle, int body_index,
                        double x_m, double y_m, double z_m);

/**
 * @brief Sets a body's world-space linear velocity.
 * @param world_handle Target world handle.
 * @param body_index Zero-based body index.
 * @param vx_mps Velocity x component in meters/second.
 * @param vy_mps Velocity y component in meters/second.
 * @param vz_mps Velocity z component in meters/second.
 * @return 0 on success, non-zero on failure.
 */
int c_rsSetBodyLinearVelocity(uint64_t world_handle, int body_index,
                              double vx_mps, double vy_mps, double vz_mps);

/**
 * @brief Enables or disables gravity for a single body.
 * @param world_handle Target world handle.
 * @param body_index Zero-based body index.
 * @param enabled Non-zero to enable gravity for this body, zero to disable.
 * @return 0 on success, non-zero on failure.
 */
int c_rsSetBodyGravityEnabled(uint64_t world_handle, int body_index,
                              int enabled);

/**
 * @brief Sets per-body material coefficients used in contact response.
 * @param world_handle Target world handle.
 * @param body_index Zero-based body index.
 * @param restitution Coefficient of restitution, generally in [0, 1].
 * @param friction_kinetic Kinetic friction coefficient, typically >= 0.
 * @param friction_static Static friction coefficient, typically >= 0.
 * @param collision_damping Additional collision damping term, typically >= 0.
 * @return 0 on success, non-zero on failure.
 */
int c_rsSetBodyMaterial(uint64_t world_handle, int body_index,
                        double restitution, double friction_kinetic,
                        double friction_static, double collision_damping);

/**
 * @brief Assigns a numeric material id to a body.
 * @param world_handle Target world handle.
 * @param body_index Zero-based body index.
 * @param material_id Application-defined material identifier.
 * @return 0 on success, non-zero on failure.
 *
 * The material id is used by world-level material interaction tables to override
 * restitution/friction for specific material pairs.
 */
int c_rsSetBodyMaterialId(uint64_t world_handle, int body_index,
                          int32_t material_id);

/**
 * @brief Sets broad-phase collision filtering for a body.
 * @param world_handle Target world handle.
 * @param body_index Zero-based body index.
 * @param collision_layer_bits Bitmask describing layers this body belongs to.
 * @param collision_mask_bits Bitmask describing layers this body can interact with.
 * @return 0 on success, non-zero on failure.
 *
 * A pair (A, B) is considered eligible only if both expressions are non-zero:
 * - (A.layer_bits & B.mask_bits)
 * - (B.layer_bits & A.mask_bits)
 */
int c_rsSetBodyCollisionFilter(uint64_t world_handle, int body_index,
                               uint32_t collision_layer_bits,
                               uint32_t collision_mask_bits);

/**
 * @brief Configures spherical aerodynamic metadata for a body.
 * @param world_handle Target world handle.
 * @param body_index Zero-based body index.
 * @param radius_m Sphere radius in meters.
 * @param drag_coefficient Dimensionless drag coefficient (Cd).
 * @return 0 on success, non-zero on failure.
 */
int c_rsSetBodyAerodynamicSphere(uint64_t world_handle, int body_index,
                                 double radius_m, double drag_coefficient);

/**
 * @brief Configures box aerodynamic metadata for a body.
 * @param world_handle Target world handle.
 * @param body_index Zero-based body index.
 * @param x_m Box size along body-local x in meters.
 * @param y_m Box size along body-local y in meters.
 * @param z_m Box size along body-local z in meters.
 * @param drag_coefficient Dimensionless drag coefficient (Cd).
 * @return 0 on success, non-zero on failure.
 */
int c_rsSetBodyAerodynamicBox(uint64_t world_handle, int body_index,
                              double x_m, double y_m, double z_m,
                              double drag_coefficient);

/**
 * @brief Configures world-level aerodynamic constants and feature toggle.
 * @param world_handle Target world handle.
 * @param enabled Non-zero enables aerodynamic forces; zero disables them.
 * @param air_density_kgpm3 Air density in kg/m^3.
 * @param linear_drag_coefficient_n_per_mps Linear drag coefficient in N/(m/s).
 * @param magnus_coefficient Magnus lift scale factor.
 * @param default_drag_coefficient Default dimensionless Cd when body value is unset.
 * @param default_drag_reference_area_m2 Default area in m^2 when body geometry is unset.
 * @return 0 on success, non-zero on failure.
 */
int c_rsSetWorldAerodynamics(uint64_t world_handle, int enabled,
                             double air_density_kgpm3,
                             double linear_drag_coefficient_n_per_mps,
                             double magnus_coefficient,
                             double default_drag_coefficient,
                             double default_drag_reference_area_m2);

/**
 * @brief Adds or updates a material-pair interaction override.
 * @param world_handle Target world handle.
 * @param material_a_id First material id in the pair.
 * @param material_b_id Second material id in the pair.
 * @param restitution Pair-specific restitution coefficient.
 * @param friction Pair-specific friction coefficient.
 * @param enabled Non-zero enables this interaction entry; zero disables it.
 * @return 0 on success, non-zero on failure.
 *
 * Pair lookup is symmetric: (A, B) and (B, A) resolve to the same entry.
 */
int c_rsSetMaterialInteraction(uint64_t world_handle, int32_t material_a_id,
                               int32_t material_b_id, double restitution,
                               double friction, int enabled);

/**
 * @brief Advances the world by `steps` fixed timesteps.
 * @param world_handle Target world handle.
 * @param steps Number of fixed-dt integration steps; must be >= 0.
 * @return 0 on success, non-zero on failure.
 */
int c_rsStepWorld(uint64_t world_handle, int steps);

/**
 * @brief Sets world gravity acceleration vector.
 * @param world_handle Target world handle.
 * @param gx_mps2 Gravity x component in m/s^2.
 * @param gy_mps2 Gravity y component in m/s^2.
 * @param gz_mps2 Gravity z component in m/s^2.
 * @return 0 on success, non-zero on failure.
 */
int c_rsSetWorldGravity(uint64_t world_handle, double gx_mps2,
                        double gy_mps2, double gz_mps2);

/**
 * @brief Reads a body's world-space position.
 * @param world_handle Target world handle.
 * @param body_index Zero-based body index.
 * @param x_m Output pointer for x position in meters.
 * @param y_m Output pointer for y position in meters.
 * @param z_m Output pointer for z position in meters.
 * @return 0 on success, non-zero on failure.
 */
int c_rsGetBodyPosition(uint64_t world_handle, int body_index,
                        double* x_m, double* y_m, double* z_m);

/**
 * @brief Reads a body's world-space linear velocity.
 * @param world_handle Target world handle.
 * @param body_index Zero-based body index.
 * @param vx_mps Output pointer for x velocity in meters/second.
 * @param vy_mps Output pointer for y velocity in meters/second.
 * @param vz_mps Output pointer for z velocity in meters/second.
 * @return 0 on success, non-zero on failure.
 */
int c_rsGetBodyLinearVelocity(uint64_t world_handle, int body_index,
                              double* vx_mps, double* vy_mps, double* vz_mps);

/**
 * @brief Exports body poses as a tightly packed 7-tuple array.
 * @param world_handle Target world handle.
 * @param out_pose7 Output buffer of length at least (7 * max_bodies).
 * @param max_bodies Maximum number of bodies to write.
 * @return Number of bodies written on success (>= 0), negative on failure.
 *
 * Layout per body i:
 * - out_pose7[i * 7 + 0] = x (m)
 * - out_pose7[i * 7 + 1] = y (m)
 * - out_pose7[i * 7 + 2] = z (m)
 * - out_pose7[i * 7 + 3] = qw
 * - out_pose7[i * 7 + 4] = qx
 * - out_pose7[i * 7 + 5] = qy
 * - out_pose7[i * 7 + 6] = qz
 */
int c_rsGetBodyPose7Array(uint64_t world_handle, double* out_pose7,
                          int max_bodies);

/**
 * @brief Exports body velocities as a tightly packed 6-tuple array.
 * @param world_handle Target world handle.
 * @param out_velocity6 Output buffer of length at least (6 * max_bodies).
 * @param max_bodies Maximum number of bodies to write.
 * @return Number of bodies written on success (>= 0), negative on failure.
 *
 * Layout per body i:
 * - out_velocity6[i * 6 + 0] = vx (m/s)
 * - out_velocity6[i * 6 + 1] = vy (m/s)
 * - out_velocity6[i * 6 + 2] = vz (m/s)
 * - out_velocity6[i * 6 + 3] = wx (rad/s)
 * - out_velocity6[i * 6 + 4] = wy (rad/s)
 * - out_velocity6[i * 6 + 5] = wz (rad/s)
 */
int c_rsGetBodyVelocity6Array(uint64_t world_handle, double* out_velocity6,
                              int max_bodies);

/**
 * @brief Exports full body state as a tightly packed 13-tuple array.
 * @param world_handle Target world handle.
 * @param out_state13 Output buffer of length at least (13 * max_bodies).
 * @param max_bodies Maximum number of bodies to write.
 * @return Number of bodies written on success (>= 0), negative on failure.
 *
 * Layout per body i:
 * - out_state13[i * 13 + 0] = x (m)
 * - out_state13[i * 13 + 1] = y (m)
 * - out_state13[i * 13 + 2] = z (m)
 * - out_state13[i * 13 + 3] = qw
 * - out_state13[i * 13 + 4] = qx
 * - out_state13[i * 13 + 5] = qy
 * - out_state13[i * 13 + 6] = qz
 * - out_state13[i * 13 + 7] = vx (m/s)
 * - out_state13[i * 13 + 8] = vy (m/s)
 * - out_state13[i * 13 + 9] = vz (m/s)
 * - out_state13[i * 13 + 10] = wx (rad/s)
 * - out_state13[i * 13 + 11] = wy (rad/s)
 * - out_state13[i * 13 + 12] = wz (rad/s)
 */
int c_rsGetBodyState13Array(uint64_t world_handle, double* out_state13,
                            int max_bodies);

#ifdef __cplusplus
}  // extern "C"
#endif
