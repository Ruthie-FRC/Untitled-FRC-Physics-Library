#include "math_jni.h"
#include <memory>

// Helper macros for pointer conversion
#define PTR(T, ptr) reinterpret_cast<T*>(ptr)
#define JPTR(obj) reinterpret_cast<jlong>(obj)

// Vector3 JNI
JNIEXPORT jlong JNICALL Java_api_Vector3_nativeCreate(JNIEnv*, jobject, jdouble x, jdouble y, jdouble z) {
    return JPTR(new frcsim::Vector3(x, y, z));
}
JNIEXPORT jdouble JNICALL Java_api_Vector3_nativeNorm(JNIEnv*, jobject, jlong ptr) {
    return PTR(frcsim::Vector3, ptr)->norm();
}
JNIEXPORT void JNICALL Java_api_Vector3_nativeDelete(JNIEnv*, jobject, jlong ptr) {
    delete PTR(frcsim::Vector3, ptr);
}

// Quaternion JNI
JNIEXPORT jlong JNICALL Java_api_Quaternion_nativeCreate(JNIEnv*, jobject, jdouble w, jdouble x, jdouble y, jdouble z) {
    return JPTR(new frcsim::Quaternion(w, x, y, z));
}
JNIEXPORT jlong JNICALL Java_api_Quaternion_nativeFromAxisAngle(JNIEnv*, jclass, jlong axisPtr, jdouble angle) {
    auto axis = *PTR(frcsim::Vector3, axisPtr);
    return JPTR(new frcsim::Quaternion(frcsim::Quaternion::fromAxisAngle(axis, angle)));
}
JNIEXPORT jlong JNICALL Java_api_Quaternion_nativeMultiply(JNIEnv*, jobject, jlong ptrA, jlong ptrB) {
    auto a = *PTR(frcsim::Quaternion, ptrA);
    auto b = *PTR(frcsim::Quaternion, ptrB);
    return JPTR(new frcsim::Quaternion(a * b));
}
JNIEXPORT jlong JNICALL Java_api_Quaternion_nativeRotate(JNIEnv*, jobject, jlong qPtr, jlong vPtr) {
    auto q = *PTR(frcsim::Quaternion, qPtr);
    auto v = *PTR(frcsim::Vector3, vPtr);
    auto result = q.rotate(v);
    return JPTR(new frcsim::Vector3(result));
}
JNIEXPORT void JNICALL Java_api_Quaternion_nativeDelete(JNIEnv*, jobject, jlong ptr) {
    delete PTR(frcsim::Quaternion, ptr);
}

// Matrix3 JNI
JNIEXPORT jlong JNICALL Java_api_Matrix3_nativeCreate(JNIEnv*, jobject) {
    return JPTR(new frcsim::Matrix3());
}
JNIEXPORT jlong JNICALL Java_api_Matrix3_nativeMultiply(JNIEnv*, jobject, jlong mPtr, jlong vPtr) {
    auto m = *PTR(frcsim::Matrix3, mPtr);
    auto v = *PTR(frcsim::Vector3, vPtr);
    auto result = m * v;
    return JPTR(new frcsim::Vector3(result));
}
JNIEXPORT void JNICALL Java_api_Matrix3_nativeDelete(JNIEnv*, jobject, jlong ptr) {
    delete PTR(frcsim::Matrix3, ptr);
}
