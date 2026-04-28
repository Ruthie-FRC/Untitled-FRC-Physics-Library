#pragma once
#include <jni.h>
#include "frcsim/math/vector.hpp"
#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/matrix.hpp"

#ifdef __cplusplus
extern "C" {
#endif

// Vector3 JNI
JNIEXPORT jlong JNICALL Java_api_Vector3_nativeCreate(JNIEnv*, jobject, jdouble, jdouble, jdouble);
JNIEXPORT jdouble JNICALL Java_api_Vector3_nativeNorm(JNIEnv*, jobject, jlong);
JNIEXPORT void JNICALL Java_api_Vector3_nativeDelete(JNIEnv*, jobject, jlong);

// Quaternion JNI
JNIEXPORT jlong JNICALL Java_api_Quaternion_nativeCreate(JNIEnv*, jobject, jdouble, jdouble, jdouble, jdouble);
JNIEXPORT jlong JNICALL Java_api_Quaternion_nativeFromAxisAngle(JNIEnv*, jclass, jlong, jdouble);
JNIEXPORT jlong JNICALL Java_api_Quaternion_nativeMultiply(JNIEnv*, jobject, jlong, jlong);
JNIEXPORT jlong JNICALL Java_api_Quaternion_nativeRotate(JNIEnv*, jobject, jlong, jlong);
JNIEXPORT void JNICALL Java_api_Quaternion_nativeDelete(JNIEnv*, jobject, jlong);

// Matrix3 JNI
JNIEXPORT jlong JNICALL Java_api_Matrix3_nativeCreate(JNIEnv*, jobject);
JNIEXPORT jlong JNICALL Java_api_Matrix3_nativeMultiply(JNIEnv*, jobject, jlong, jlong);
JNIEXPORT void JNICALL Java_api_Matrix3_nativeDelete(JNIEnv*, jobject, jlong);

#ifdef __cplusplus
}
#endif
