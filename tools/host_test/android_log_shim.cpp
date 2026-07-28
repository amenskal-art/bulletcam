#include <android/log.h>

#include <cstdio>

extern "C" int __android_log_print(int priority, const char* tag, const char* format, ...) {
    va_list args;
    va_start(args, format);
    std::fprintf(stderr, "[%s] ", tag);
    const int written = std::vfprintf(stderr, format, args);
    std::fprintf(stderr, "\n");
    va_end(args);
    (void)priority;
    return written;
}
