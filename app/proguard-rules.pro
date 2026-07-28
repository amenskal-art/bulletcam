# JNI entry points are resolved by name at runtime.
-keepclasseswithmembernames class * { native <methods>; }
-keep class com.bulletcam.capture.nativebridge.** { *; }
-keep class com.google.ar.core.** { *; }
