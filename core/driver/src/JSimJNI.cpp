// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#include "jni.h"
#include "jsim_jni_JSimJNI.h"

#include <cstdint>

#include "driverheader.h"

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
  // Check to ensure the JNI version is valid

  JNIEnv* env;
  if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
    return JNI_ERR;

  // In here is also where you store things like class references
  // if they are ever needed

  return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM* vm, void* reserved) {}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    initialize
 * Signature: ()I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_initialize
  (JNIEnv*, jclass)
{
  c_doThing();
  return 1;
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    createWorld
 * Signature: (DZ)J
 */
JNIEXPORT jlong JNICALL
Java_jsim_jni_JSimJNI_createWorld
  (JNIEnv*, jclass, jdouble fixed_dt_seconds, jboolean enable_gravity)
{
  return static_cast<jlong>(
      c_rsCreateWorld(fixed_dt_seconds, enable_gravity ? 1 : 0));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    destroyWorld
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_jsim_jni_JSimJNI_destroyWorld
  (JNIEnv*, jclass, jlong world_handle)
{
  c_rsDestroyWorld(static_cast<uint64_t>(world_handle));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    createBody
 * Signature: (JD)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_createBody
  (JNIEnv*, jclass, jlong world_handle, jdouble mass_kg)
{
  return static_cast<jint>(
      c_rsCreateBody(static_cast<uint64_t>(world_handle), mass_kg));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    setBodyPosition
 * Signature: (JIDDD)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_setBodyPosition
  (JNIEnv*, jclass, jlong world_handle, jint body_index, jdouble x_m,
   jdouble y_m, jdouble z_m)
{
  return static_cast<jint>(
      c_rsSetBodyPosition(static_cast<uint64_t>(world_handle),
                          body_index, x_m, y_m, z_m));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    setBodyLinearVelocity
 * Signature: (JIDDD)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_setBodyLinearVelocity
  (JNIEnv*, jclass, jlong world_handle, jint body_index, jdouble vx_mps,
   jdouble vy_mps, jdouble vz_mps)
{
  return static_cast<jint>(
      c_rsSetBodyLinearVelocity(static_cast<uint64_t>(world_handle),
                                body_index, vx_mps, vy_mps, vz_mps));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    setBodyGravityEnabled
 * Signature: (JIZ)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_setBodyGravityEnabled
  (JNIEnv*, jclass, jlong world_handle, jint body_index, jboolean enabled)
{
  return static_cast<jint>(
      c_rsSetBodyGravityEnabled(static_cast<uint64_t>(world_handle),
                                body_index, enabled ? 1 : 0));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    setBodyMaterial
 * Signature: (JIDDDD)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_setBodyMaterial
  (JNIEnv*, jclass, jlong world_handle, jint body_index,
   jdouble restitution, jdouble friction_kinetic, jdouble friction_static,
   jdouble collision_damping)
{
  return static_cast<jint>(
      c_rsSetBodyMaterial(static_cast<uint64_t>(world_handle), body_index,
                          restitution, friction_kinetic, friction_static,
                          collision_damping));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    setBodyMaterialId
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_setBodyMaterialId
  (JNIEnv*, jclass, jlong world_handle, jint body_index, jint material_id)
{
  return static_cast<jint>(
      c_rsSetBodyMaterialId(static_cast<uint64_t>(world_handle), body_index,
                            static_cast<int32_t>(material_id)));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    setBodyCollisionFilter
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_setBodyCollisionFilter
  (JNIEnv*, jclass, jlong world_handle, jint body_index,
   jint collision_layer_bits, jint collision_mask_bits)
{
  return static_cast<jint>(
      c_rsSetBodyCollisionFilter(static_cast<uint64_t>(world_handle),
                                 body_index,
                                 static_cast<uint32_t>(collision_layer_bits),
                                 static_cast<uint32_t>(collision_mask_bits)));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    setBodyAerodynamicSphere
 * Signature: (JIDD)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_setBodyAerodynamicSphere
  (JNIEnv*, jclass, jlong world_handle, jint body_index, jdouble radius_m,
   jdouble drag_coefficient)
{
  return static_cast<jint>(
      c_rsSetBodyAerodynamicSphere(static_cast<uint64_t>(world_handle),
                                   body_index, radius_m, drag_coefficient));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    setBodyAerodynamicBox
 * Signature: (JIDDDD)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_setBodyAerodynamicBox
  (JNIEnv*, jclass, jlong world_handle, jint body_index, jdouble x_m,
   jdouble y_m, jdouble z_m, jdouble drag_coefficient)
{
  return static_cast<jint>(
      c_rsSetBodyAerodynamicBox(static_cast<uint64_t>(world_handle),
                                body_index, x_m, y_m, z_m, drag_coefficient));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    setWorldGravity
 * Signature: (JDDD)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_setWorldGravity
  (JNIEnv*, jclass, jlong world_handle, jdouble gx_mps2, jdouble gy_mps2,
   jdouble gz_mps2)
{
  return static_cast<jint>(
      c_rsSetWorldGravity(static_cast<uint64_t>(world_handle),
                          gx_mps2, gy_mps2, gz_mps2));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    setWorldAerodynamics
 * Signature: (JZDDDDD)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_setWorldAerodynamics
  (JNIEnv*, jclass, jlong world_handle, jboolean enabled,
   jdouble air_density_kgpm3, jdouble linear_drag_n_per_mps,
   jdouble magnus_coefficient, jdouble default_drag_coefficient,
   jdouble default_drag_reference_area_m2)
{
  return static_cast<jint>(
      c_rsSetWorldAerodynamics(static_cast<uint64_t>(world_handle),
                               enabled ? 1 : 0, air_density_kgpm3,
                               linear_drag_n_per_mps, magnus_coefficient,
                               default_drag_coefficient,
                               default_drag_reference_area_m2));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    setMaterialInteraction
 * Signature: (JIIDDZ)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_setMaterialInteraction
  (JNIEnv*, jclass, jlong world_handle, jint material_a_id,
   jint material_b_id, jdouble restitution, jdouble friction,
   jboolean enabled)
{
  return static_cast<jint>(
      c_rsSetMaterialInteraction(static_cast<uint64_t>(world_handle),
                                 static_cast<int32_t>(material_a_id),
                                 static_cast<int32_t>(material_b_id),
                                 restitution, friction, enabled ? 1 : 0));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    stepWorld
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_stepWorld
  (JNIEnv*, jclass, jlong world_handle, jint steps)
{
  return static_cast<jint>(
      c_rsStepWorld(static_cast<uint64_t>(world_handle), steps));
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    getBodyPosition
 * Signature: (JI[D)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_getBodyPosition
  (JNIEnv* env, jclass, jlong world_handle, jint body_index,
   jdoubleArray out_xyz)
{
  if (out_xyz == nullptr || env->GetArrayLength(out_xyz) < 3) {
    return -1;
  }

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  const int rc = c_rsGetBodyPosition(
      static_cast<uint64_t>(world_handle), body_index, &x, &y, &z);
  if (rc != 0) {
    return rc;
  }

  jdouble values[3] = {x, y, z};
  env->SetDoubleArrayRegion(out_xyz, 0, 3, values);
  return 0;
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    getBodyLinearVelocity
 * Signature: (JI[D)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_getBodyLinearVelocity
  (JNIEnv* env, jclass, jlong world_handle, jint body_index,
   jdoubleArray out_vxyz)
{
  if (out_vxyz == nullptr || env->GetArrayLength(out_vxyz) < 3) {
    return -1;
  }

  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  const int rc = c_rsGetBodyLinearVelocity(
      static_cast<uint64_t>(world_handle), body_index, &vx, &vy, &vz);
  if (rc != 0) {
    return rc;
  }

  jdouble values[3] = {vx, vy, vz};
  env->SetDoubleArrayRegion(out_vxyz, 0, 3, values);
  return 0;
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    getBodyPose7Array
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_getBodyPose7Array
  (JNIEnv* env, jclass, jlong world_handle, jdoubleArray out_pose7)
{
  if (out_pose7 == nullptr) {
    return -1;
  }

  const jsize len = env->GetArrayLength(out_pose7);
  if (len < 7) {
    return -1;
  }

  const int max_bodies = static_cast<int>(len / 7);
  jdouble* data = env->GetDoubleArrayElements(out_pose7, nullptr);
  if (data == nullptr) {
    return -1;
  }

  const int rc = c_rsGetBodyPose7Array(static_cast<uint64_t>(world_handle),
                                       data, max_bodies);
  env->ReleaseDoubleArrayElements(out_pose7, data, 0);
  return static_cast<jint>(rc);
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    getBodyVelocity6Array
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_getBodyVelocity6Array
  (JNIEnv* env, jclass, jlong world_handle, jdoubleArray out_velocity6)
{
  if (out_velocity6 == nullptr) {
    return -1;
  }

  const jsize len = env->GetArrayLength(out_velocity6);
  if (len < 6) {
    return -1;
  }

  const int max_bodies = static_cast<int>(len / 6);
  jdouble* data = env->GetDoubleArrayElements(out_velocity6, nullptr);
  if (data == nullptr) {
    return -1;
  }

  const int rc = c_rsGetBodyVelocity6Array(static_cast<uint64_t>(world_handle),
                                           data, max_bodies);
  env->ReleaseDoubleArrayElements(out_velocity6, data, 0);
  return static_cast<jint>(rc);
}

/*
 * Class:     jsim_jni_JSimJNI
 * Method:    getBodyState13Array
 * Signature: (J[D)I
 */
JNIEXPORT jint JNICALL
Java_jsim_jni_JSimJNI_getBodyState13Array
  (JNIEnv* env, jclass, jlong world_handle, jdoubleArray out_state13)
{
  if (out_state13 == nullptr) {
    return -1;
  }

  const jsize len = env->GetArrayLength(out_state13);
  if (len < 13) {
    return -1;
  }

  const int max_bodies = static_cast<int>(len / 13);
  jdouble* data = env->GetDoubleArrayElements(out_state13, nullptr);
  if (data == nullptr) {
    return -1;
  }

  const int rc = c_rsGetBodyState13Array(static_cast<uint64_t>(world_handle),
                                         data, max_bodies);
  env->ReleaseDoubleArrayElements(out_state13, data, 0);
  return static_cast<jint>(rc);
}
