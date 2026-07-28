package com.bulletcam.capture.nativebridge

import java.nio.ByteBuffer

/**
 * Kotlin face of the native reconstruction core.
 *
 * Camera planes are handed over as direct [ByteBuffer]s and read on the native
 * side through `GetDirectBufferAddress`, so a captured frame never gets copied
 * into a JVM byte array on the way to C++.
 */
class NativeReconstructor : AutoCloseable {

    fun interface ProgressListener {
        /** Invoked from the reconstruction thread. [fraction] runs 0f..1f. */
        fun onProgress(stage: String, fraction: Float)
    }

    /**
     * Tuning knobs, ordered exactly as the native `OptionIndex` enum expects.
     *
     * [targetTriangles] of 0 keeps the full-resolution surface; use
     * [poissonResolution] as the primary quality control instead.
     */
    data class Options(
        val stereoImageMaxEdge: Int = 512,
        val textureImageMaxEdge: Int = 1600,
        val patchMatchIterations: Int = 5,
        val sourceViews: Int = 6,
        val poissonResolution: Int = 160,
        val atlasResolution: Int = 2048,
        val targetTriangles: Int = 0,
        val minConsistentViews: Int = 2,
    ) {
        fun toIntArray() = intArrayOf(
            stereoImageMaxEdge,
            textureImageMaxEdge,
            patchMatchIterations,
            sourceViews,
            poissonResolution,
            atlasResolution,
            targetTriangles,
            minConsistentViews,
        )

        companion object {
            /** Roughly 2 minutes on a recent phone for 40 photos. */
            val Fast = Options(
                stereoImageMaxEdge = 384,
                patchMatchIterations = 3,
                sourceViews = 4,
                poissonResolution = 128,
                atlasResolution = 1024,
            )

            val Balanced = Options()

            /** Best detail; expect several minutes and a warm phone. */
            val High = Options(
                stereoImageMaxEdge = 768,
                textureImageMaxEdge = 2048,
                patchMatchIterations = 6,
                sourceViews = 8,
                poissonResolution = 224,
                atlasResolution = 4096,
                minConsistentViews = 3,
            )
        }
    }

    private var handle: Long = 0L

    val isOpen: Boolean get() = handle != 0L

    fun open(workDir: String, options: Options) {
        check(handle == 0L) { "Session already open" }
        handle = nativeCreateSession(workDir, options.toIntArray())
        check(handle != 0L) { "Could not create the native capture session" }
    }

    /**
     * Converts and stores one pose-tagged photo.
     *
     * @param cameraToWorld 4x4 row-major camera-to-world matrix in
     *   computer-vision axes: +X right, +Y down, +Z along the viewing direction.
     * @param intrinsics `[fx, fy, cx, cy]` for the supplied image size.
     * @return the frame's sharpness score, or -1f if it was rejected as blurry.
     */
    fun addFrame(
        y: ByteBuffer,
        u: ByteBuffer,
        v: ByteBuffer,
        width: Int,
        height: Int,
        yRowStride: Int,
        uvRowStride: Int,
        uvPixelStride: Int,
        cameraToWorld: FloatArray,
        intrinsics: FloatArray,
        minSharpness: Float,
    ): Float {
        if (handle == 0L) return -1f
        return nativeAddFrame(
            handle, y, u, v, width, height, yRowStride, uvRowStride, uvPixelStride,
            cameraToWorld, intrinsics, minSharpness,
        )
    }

    /**
     * Restricts the reconstruction to a ball around the subject, in ARCore world
     * coordinates. A radius of 0 clears the mask and reconstructs everything.
     *
     * This is not just a crop of the result. Dense stereo skips pixels whose ray
     * misses the ball entirely, and for the ones that hit it searches only the
     * depth interval between the two intersections rather than the whole scene
     * range - so the mask makes the expensive stage both faster and better
     * conditioned. It also keeps the Poisson grid and the voxel size scaled to
     * the object instead of the room.
     */
    fun setSubjectVolume(centerX: Float, centerY: Float, centerZ: Float, radius: Float) {
        if (handle == 0L) return
        nativeSetSubjectVolume(handle, centerX, centerY, centerZ, radius)
    }

    fun clearSubjectVolume() {
        if (handle != 0L) nativeSetSubjectVolume(handle, 0f, 0f, 0f, 0f)
    }

    fun frameCount(): Int = if (handle == 0L) 0 else nativeFrameCount(handle)

    fun discardAll() {
        if (handle != 0L) nativeReset(handle)
    }

    /** Blocking. Run it off the main thread. Returns true when both files exist. */
    fun reconstruct(objPath: String, glbPath: String, listener: ProgressListener?): Boolean {
        if (handle == 0L) return false
        return nativeReconstruct(handle, objPath, glbPath, listener) == 0
    }

    /** Asks the running reconstruction to stop at the next checkpoint. */
    fun cancel() {
        if (handle != 0L) nativeCancel(handle)
    }

    /** Result summary, or the reason the last run stopped. */
    fun lastMessage(): String = if (handle == 0L) "" else nativeLastMessage(handle)

    override fun close() {
        if (handle != 0L) {
            nativeDestroySession(handle)
            handle = 0L
        }
    }

    private external fun nativeCreateSession(workDir: String, options: IntArray): Long
    private external fun nativeDestroySession(handle: Long)
    private external fun nativeAddFrame(
        handle: Long,
        y: ByteBuffer,
        u: ByteBuffer,
        v: ByteBuffer,
        width: Int,
        height: Int,
        yRowStride: Int,
        uvRowStride: Int,
        uvPixelStride: Int,
        cameraToWorld: FloatArray,
        intrinsics: FloatArray,
        minSharpness: Float,
    ): Float

    private external fun nativeSetSubjectVolume(
        handle: Long,
        centerX: Float,
        centerY: Float,
        centerZ: Float,
        radius: Float,
    )

    private external fun nativeFrameCount(handle: Long): Int
    private external fun nativeReset(handle: Long)
    private external fun nativeCancel(handle: Long)
    private external fun nativeLastMessage(handle: Long): String
    private external fun nativeReconstruct(
        handle: Long,
        objPath: String,
        glbPath: String,
        listener: ProgressListener?,
    ): Int

    companion object {
        init {
            System.loadLibrary("bulletcam")
        }
    }
}
