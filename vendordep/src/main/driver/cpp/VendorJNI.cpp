#include "jni.h"
#include "rensim_jni_VendorJNI.h"

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

JNIEXPORT jint JNICALL Java_rensim_jni_VendorJNI_initialize
  (JNIEnv *, jclass) {
  c_doThing();
  return 1;
}

JNIEXPORT jlong JNICALL Java_rensim_jni_VendorJNI_createWorld
  (JNIEnv *, jclass, jdouble fixed_dt_seconds, jboolean enable_gravity) {
  return static_cast<jlong>(c_rsCreateWorld(fixed_dt_seconds, enable_gravity ? 1 : 0));
}

JNIEXPORT void JNICALL Java_rensim_jni_VendorJNI_destroyWorld
  (JNIEnv *, jclass, jlong world_handle) {
  c_rsDestroyWorld(static_cast<unsigned long long>(world_handle));
}

JNIEXPORT jint JNICALL Java_rensim_jni_VendorJNI_createBody
  (JNIEnv *, jclass, jlong world_handle, jdouble mass_kg) {
  return static_cast<jint>(c_rsCreateBody(static_cast<unsigned long long>(world_handle), mass_kg));
}

JNIEXPORT jint JNICALL Java_rensim_jni_VendorJNI_setBodyPosition
  (JNIEnv *, jclass, jlong world_handle, jint body_index, jdouble x_m, jdouble y_m, jdouble z_m) {
  return static_cast<jint>(c_rsSetBodyPosition(static_cast<unsigned long long>(world_handle), body_index, x_m, y_m, z_m));
}

JNIEXPORT jint JNICALL Java_rensim_jni_VendorJNI_setBodyLinearVelocity
  (JNIEnv *, jclass, jlong world_handle, jint body_index, jdouble vx_mps, jdouble vy_mps, jdouble vz_mps) {
  return static_cast<jint>(
      c_rsSetBodyLinearVelocity(static_cast<unsigned long long>(world_handle), body_index, vx_mps, vy_mps, vz_mps));
}

JNIEXPORT jint JNICALL Java_rensim_jni_VendorJNI_setBodyGravityEnabled
  (JNIEnv *, jclass, jlong world_handle, jint body_index, jboolean enabled) {
  return static_cast<jint>(
      c_rsSetBodyGravityEnabled(static_cast<unsigned long long>(world_handle), body_index, enabled ? 1 : 0));
}

JNIEXPORT jint JNICALL Java_rensim_jni_VendorJNI_setWorldGravity
  (JNIEnv *, jclass, jlong world_handle, jdouble gx_mps2, jdouble gy_mps2, jdouble gz_mps2) {
  return static_cast<jint>(
      c_rsSetWorldGravity(static_cast<unsigned long long>(world_handle), gx_mps2, gy_mps2, gz_mps2));
}

JNIEXPORT jint JNICALL Java_rensim_jni_VendorJNI_stepWorld
  (JNIEnv *, jclass, jlong world_handle, jint steps) {
  return static_cast<jint>(c_rsStepWorld(static_cast<unsigned long long>(world_handle), steps));
}

JNIEXPORT jint JNICALL Java_rensim_jni_VendorJNI_getBodyPosition
  (JNIEnv *env, jclass, jlong world_handle, jint body_index, jdoubleArray out_xyz) {
  if (out_xyz == nullptr || env->GetArrayLength(out_xyz) < 3) {
    return -1;
  }

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  const int rc = c_rsGetBodyPosition(static_cast<unsigned long long>(world_handle), body_index, &x, &y, &z);
  if (rc != 0) {
    return rc;
  }

  jdouble values[3] = {x, y, z};
  env->SetDoubleArrayRegion(out_xyz, 0, 3, values);
  return 0;
}

JNIEXPORT jint JNICALL Java_rensim_jni_VendorJNI_getBodyLinearVelocity
  (JNIEnv *env, jclass, jlong world_handle, jint body_index, jdoubleArray out_vxyz) {
  if (out_vxyz == nullptr || env->GetArrayLength(out_vxyz) < 3) {
    return -1;
  }

  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  const int rc = c_rsGetBodyLinearVelocity(static_cast<unsigned long long>(world_handle), body_index, &vx, &vy, &vz);
  if (rc != 0) {
    return rc;
  }

  jdouble values[3] = {vx, vy, vz};
  env->SetDoubleArrayRegion(out_vxyz, 0, 3, values);
  return 0;
}
