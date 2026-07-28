#pragma once
#include <android/log.h>

#define BC_TAG "BulletCamNative"
#define BC_LOGI(...) __android_log_print(ANDROID_LOG_INFO, BC_TAG, __VA_ARGS__)
#define BC_LOGW(...) __android_log_print(ANDROID_LOG_WARN, BC_TAG, __VA_ARGS__)
#define BC_LOGE(...) __android_log_print(ANDROID_LOG_ERROR, BC_TAG, __VA_ARGS__)
