// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#include "jni.h"
#include "rensim_jni_RenSimJNI.h"

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
 * Class:     rensim_jni_RenSimJNI
 * Method:    initialize
 * Signature: ()I
 */
JNIEXPORT jint JNICALL
Java_rensim_jni_RenSimJNI_initialize
  (JNIEnv*, jclass)
{
  c_doThing();
  return 1;
}

/*
 * Class:     rensim_jni_RenSimJNI
 * Method:    createWorld
 * Signature: (DZ)J
 */
JNIEXPORT jlong JNICALL
Java_rensim_jni_RenSimJNI_createWorld
  (JNIEnv*, jclass, jdouble fixed_dt_seconds, jboolean enable_gravity)
{
  return static_cast<jlong>(
      c_rsCreateWorld(fixed_dt_seconds, enable_gravity ? 1 : 0));
}

/*
 * Class:     rensim_jni_RenSimJNI
 * Method:    destroyWorld
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_rensim_jni_RenSimJNI_destroyWorld
  (JNIEnv*, jclass, jlong world_handle)
{
  c_rsDestroyWorld(static_cast<unsigned long long>(world_handle));
}

/*
 * Class:     rensim_jni_RenSimJNI
 * Method:    createBody
 * Signature: (JD)I
 */
JNIEXPORT jint JNICALL
Java_rensim_jni_RenSimJNI_createBody
  (JNIEnv*, jclass, jlong world_handle, jdouble mass_kg)
{
  return static_cast<jint>(
      c_rsCreateBody(static_cast<unsigned long long>(world_handle), mass_kg));
}

/*
 * Class:     rensim_jni_RenSimJNI
 * Method:    setBodyPosition
 * Signature: (JIDDD)I
 */
JNIEXPORT jint JNICALL
Java_rensim_jni_RenSimJNI_setBodyPosition
  (JNIEnv*, jclass, jlong world_handle, jint body_index, jdouble x_m,
   jdouble y_m, jdouble z_m)
{
  return static_cast<jint>(
      c_rsSetBodyPosition(static_cast<unsigned long long>(world_handle),
                          body_index, x_m, y_m, z_m));
}

/*
 * Class:     rensim_jni_RenSimJNI
 * Method:    setBodyLinearVelocity
 * Signature: (JIDDD)I
 */
JNIEXPORT jint JNICALL
Java_rensim_jni_RenSimJNI_setBodyLinearVelocity
  (JNIEnv*, jclass, jlong world_handle, jint body_index, jdouble vx_mps,
   jdouble vy_mps, jdouble vz_mps)
{
  return static_cast<jint>(
      c_rsSetBodyLinearVelocity(static_cast<unsigned long long>(world_handle),
                                body_index, vx_mps, vy_mps, vz_mps));
}

/*
 * Class:     rensim_jni_RenSimJNI
 * Method:    setBodyGravityEnabled
 * Signature: (JIZ)I
 */
JNIEXPORT jint JNICALL
Java_rensim_jni_RenSimJNI_setBodyGravityEnabled
  (JNIEnv*, jclass, jlong world_handle, jint body_index, jboolean enabled)
{
  return static_cast<jint>(
      c_rsSetBodyGravityEnabled(static_cast<unsigned long long>(world_handle),
                                body_index, enabled ? 1 : 0));
}

/*
 * Class:     rensim_jni_RenSimJNI
 * Method:    setWorldGravity
 * Signature: (JDDD)I
 */
JNIEXPORT jint JNICALL
Java_rensim_jni_RenSimJNI_setWorldGravity
  (JNIEnv*, jclass, jlong world_handle, jdouble gx_mps2, jdouble gy_mps2,
   jdouble gz_mps2)
{
  return static_cast<jint>(
      c_rsSetWorldGravity(static_cast<unsigned long long>(world_handle),
                          gx_mps2, gy_mps2, gz_mps2));
}

/*
 * Class:     rensim_jni_RenSimJNI
 * Method:    stepWorld
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL
Java_rensim_jni_RenSimJNI_stepWorld
  (JNIEnv*, jclass, jlong world_handle, jint steps)
{
  return static_cast<jint>(
      c_rsStepWorld(static_cast<unsigned long long>(world_handle), steps));
}

/*
 * Class:     rensim_jni_RenSimJNI
 * Method:    getBodyPosition
 * Signature: (JI[D)I
 */
JNIEXPORT jint JNICALL
Java_rensim_jni_RenSimJNI_getBodyPosition
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
      static_cast<unsigned long long>(world_handle), body_index, &x, &y, &z);
  if (rc != 0) {
    return rc;
  }

  jdouble values[3] = {x, y, z};
  env->SetDoubleArrayRegion(out_xyz, 0, 3, values);
  return 0;
}

/*
 * Class:     rensim_jni_RenSimJNI
 * Method:    getBodyLinearVelocity
 * Signature: (JI[D)I
 */
JNIEXPORT jint JNICALL
Java_rensim_jni_RenSimJNI_getBodyLinearVelocity
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
      static_cast<unsigned long long>(world_handle), body_index, &vx, &vy, &vz);
  if (rc != 0) {
    return rc;
  }

  jdouble values[3] = {vx, vy, vz};
  env->SetDoubleArrayRegion(out_vxyz, 0, 3, values);
  return 0;
}
