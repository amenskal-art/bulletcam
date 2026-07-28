// JNI bridge between the Kotlin capture layer and the native reconstruction
// core. Image planes cross the boundary as direct ByteBuffers and are read
// through GetDirectBufferAddress, so a captured frame is converted straight
// from the camera's own memory into native storage with no copy in between.
#include <jni.h>

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>

#include "core/frame_store.h"
#include "core/log.h"
#include "core/progress.h"
#include "pipeline.h"

using namespace bc;

namespace {

struct Session {
    std::unique_ptr<FrameStore> store;
    Progress progress;
    std::mutex mutex;
    std::string lastMessage;
};

Session* asSession(jlong handle) { return reinterpret_cast<Session*>(handle); }

std::string toStdString(JNIEnv* env, jstring value) {
    if (!value) return {};
    const char* chars = env->GetStringUTFChars(value, nullptr);
    std::string result = chars ? chars : "";
    if (chars) env->ReleaseStringUTFChars(value, chars);
    return result;
}

// Option array layout, mirrored by NativeReconstructor.Options in Kotlin.
enum OptionIndex {
    kOptStereoMaxEdge = 0,
    kOptTextureMaxEdge,
    kOptPatchMatchIterations,
    kOptSourceViews,
    kOptPoissonResolution,
    kOptAtlasResolution,
    kOptTargetTriangles,
    kOptMinConsistentViews,
    kOptionCount
};

ReconstructionOptions readOptions(JNIEnv* env, jintArray array) {
    ReconstructionOptions options;
    if (!array) return options;
    const jsize length = env->GetArrayLength(array);
    if (length < kOptionCount) return options;
    jint values[kOptionCount];
    env->GetIntArrayRegion(array, 0, kOptionCount, values);
    options.stereoImageMaxEdge = std::max(160, values[kOptStereoMaxEdge]);
    options.textureImageMaxEdge = std::max(320, values[kOptTextureMaxEdge]);
    options.patchMatchIterations = std::max(1, values[kOptPatchMatchIterations]);
    options.sourceViewsPerRef = std::max(2, values[kOptSourceViews]);
    options.poissonResolution = std::max(48, values[kOptPoissonResolution]);
    options.atlasResolution = std::max(256, values[kOptAtlasResolution]);
    // Zero means "keep the full resolution mesh"; anything else is a hard cap.
    options.targetTriangles =
        values[kOptTargetTriangles] <= 0 ? 0 : std::max(5000, values[kOptTargetTriangles]);
    options.minConsistentViews = std::max(1, values[kOptMinConsistentViews]);
    return options;
}

}  // namespace

extern "C" {

JNIEXPORT jlong JNICALL Java_com_bulletcam_capture_nativebridge_NativeReconstructor_nativeCreateSession(
    JNIEnv* env, jobject, jstring workDir, jintArray optionArray) {
    const std::string dir = toStdString(env, workDir);
    if (dir.empty()) return 0;
    auto* session = new Session();
    session->store = std::make_unique<FrameStore>(dir, readOptions(env, optionArray));
    BC_LOGI("Capture session created at %s", dir.c_str());
    return reinterpret_cast<jlong>(session);
}

JNIEXPORT void JNICALL Java_com_bulletcam_capture_nativebridge_NativeReconstructor_nativeDestroySession(
    JNIEnv*, jobject, jlong handle) {
    Session* session = asSession(handle);
    if (!session) return;
    delete session;
}

JNIEXPORT jfloat JNICALL Java_com_bulletcam_capture_nativebridge_NativeReconstructor_nativeAddFrame(
    JNIEnv* env, jobject, jlong handle, jobject yBuffer, jobject uBuffer, jobject vBuffer,
    jint width, jint height, jint yRowStride, jint uvRowStride, jint uvPixelStride,
    jfloatArray cameraToWorld, jfloatArray intrinsics, jfloat minSharpness) {
    Session* session = asSession(handle);
    if (!session || !session->store) return -1.f;

    // Zero-copy: these point straight at the camera image's own memory.
    const auto* y = static_cast<const uint8_t*>(env->GetDirectBufferAddress(yBuffer));
    const auto* u = static_cast<const uint8_t*>(env->GetDirectBufferAddress(uBuffer));
    const auto* v = static_cast<const uint8_t*>(env->GetDirectBufferAddress(vBuffer));
    if (!y || !u || !v) {
        BC_LOGE("Camera planes are not direct ByteBuffers");
        return -1.f;
    }
    if (width <= 0 || height <= 0 || yRowStride < width) return -1.f;

    jfloat pose[16];
    jfloat k[4];
    if (env->GetArrayLength(cameraToWorld) < 16 || env->GetArrayLength(intrinsics) < 4) return -1.f;
    env->GetFloatArrayRegion(cameraToWorld, 0, 16, pose);
    env->GetFloatArrayRegion(intrinsics, 0, 4, k);

    return session->store->addFrame(y, u, v, width, height, yRowStride, uvRowStride, uvPixelStride,
                                    pose, k, minSharpness);
}

/**
 * Sets the region of interest around the subject. Radius <= 0 clears the mask
 * and reconstructs the whole scene. Safe to call between captures; it only
 * affects the reconstruction pass.
 */
JNIEXPORT void JNICALL
Java_com_bulletcam_capture_nativebridge_NativeReconstructor_nativeSetSubjectVolume(
    JNIEnv*, jobject, jlong handle, jfloat centerX, jfloat centerY, jfloat centerZ,
    jfloat radius) {
    Session* session = asSession(handle);
    if (!session || !session->store) return;
    SubjectVolume volume;
    volume.center = {centerX, centerY, centerZ};
    volume.radius = radius;
    session->store->mutableOptions().subject = volume;
}

JNIEXPORT jint JNICALL Java_com_bulletcam_capture_nativebridge_NativeReconstructor_nativeFrameCount(
    JNIEnv*, jobject, jlong handle) {
    Session* session = asSession(handle);
    return session && session->store ? session->store->frameCount() : 0;
}

JNIEXPORT void JNICALL Java_com_bulletcam_capture_nativebridge_NativeReconstructor_nativeReset(
    JNIEnv*, jobject, jlong handle) {
    Session* session = asSession(handle);
    if (session && session->store) session->store->clear();
}

JNIEXPORT void JNICALL Java_com_bulletcam_capture_nativebridge_NativeReconstructor_nativeCancel(
    JNIEnv*, jobject, jlong handle) {
    Session* session = asSession(handle);
    if (session) session->progress.requestCancel();
}

JNIEXPORT jstring JNICALL Java_com_bulletcam_capture_nativebridge_NativeReconstructor_nativeLastMessage(
    JNIEnv* env, jobject, jlong handle) {
    Session* session = asSession(handle);
    if (!session) return env->NewStringUTF("");
    std::lock_guard<std::mutex> lock(session->mutex);
    return env->NewStringUTF(session->lastMessage.c_str());
}

JNIEXPORT jint JNICALL Java_com_bulletcam_capture_nativebridge_NativeReconstructor_nativeReconstruct(
    JNIEnv* env, jobject, jlong handle, jstring objPath, jstring glbPath, jobject listener) {
    Session* session = asSession(handle);
    if (!session || !session->store) return -1;

    jmethodID onProgress = nullptr;
    if (listener) {
        jclass listenerClass = env->GetObjectClass(listener);
        onProgress = env->GetMethodID(listenerClass, "onProgress", "(Ljava/lang/String;F)V");
        env->DeleteLocalRef(listenerClass);
    }

    // The sink is only ever invoked from this thread, so capturing env is safe.
    session->progress.reset([env, listener, onProgress](const std::string& stage, float value) {
        if (!listener || !onProgress) return;
        jstring text = env->NewStringUTF(stage.c_str());
        env->CallVoidMethod(listener, onProgress, text, static_cast<jfloat>(value));
        env->DeleteLocalRef(text);
        if (env->ExceptionCheck()) env->ExceptionClear();
    });

    const ReconstructionResult result = runReconstruction(
        *session->store, toStdString(env, objPath), toStdString(env, glbPath), session->progress);

    {
        std::lock_guard<std::mutex> lock(session->mutex);
        session->lastMessage = result.message;
    }
    return result.success ? 0 : -1;
}

}  // extern "C"
