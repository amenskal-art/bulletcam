// Host stand-in for <android/log.h>.
#pragma once
#include <stdarg.h>
enum { ANDROID_LOG_INFO = 4, ANDROID_LOG_WARN = 5, ANDROID_LOG_ERROR = 6 };
#ifdef __cplusplus
extern "C"
#endif
int __android_log_print(int priority, const char* tag, const char* format, ...);
