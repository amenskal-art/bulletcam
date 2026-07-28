package com.bulletcam.capture.capture

import android.media.Image
import android.opengl.Matrix
import android.util.Log
import com.bulletcam.capture.nativebridge.NativeReconstructor
import com.google.ar.core.Camera
import com.google.ar.core.Frame
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.NotYetAvailableException
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.abs

/**
 * Turns a stream of ARCore frames into a pose-tagged photo set.
 *
 * ARCore's visual-inertial odometry already knows where the phone is to within
 * a few millimetres, so every photo arrives with a known extrinsic. That is what
 * lets the native side skip structure-from-motion and go straight to dense
 * stereo - the expensive part of photogrammetry is already solved by the time
 * the shutter fires.
 */
class CaptureController(
    private val reconstructor: NativeReconstructor,
    private val coverage: OrbitCoverage,
) {

    /** Snapshot handed to the UI once per frame. Immutable so Compose can diff it. */
    data class Overlay(
        val tracking: Boolean = false,
        val subjectPlaced: Boolean = false,
        val frameCount: Int = 0,
        val coverageFraction: Float = 0f,
        val azimuthCoverage: Float = 0f,
        val hint: String = "Point at the subject and tap to lock it in",
        val distanceMeters: Float = 0f,
        val ringPoints: FloatArray = FloatArray(0),
        val ringCovered: BooleanArray = BooleanArray(0),
        val subjectScreen: FloatArray? = null,
        val motionBlurRisk: Boolean = false,
        val lastRejectReason: String? = null,
        /** Three great circles of the capture volume, projected to the screen. */
        val volumePoints: FloatArray = FloatArray(0),
        val subjectRadius: Float = 0f,
    ) {
        // Data class equality on arrays compares identity, which is what we want
        // here: a new frame always allocates new arrays, so state always differs.
        override fun equals(other: Any?): Boolean = this === other
        override fun hashCode(): Int = System.identityHashCode(this)
    }

    /** World-space point the operator is orbiting. */
    private var subject: FloatArray? = null
    private var pendingSubjectTap: FloatArray? = null
    private val manualShutter = AtomicBoolean(false)

    var autoCapture: Boolean = true
    var minSeparationDegrees: Float = 8f
    var minSharpness: Float = 12f

    /**
     * Radius of the ball the reconstruction is restricted to. Anything outside
     * it is never matched, never fused and never surfaced.
     */
    var subjectRadius: Float = DEFAULT_SUBJECT_RADIUS
        private set

    fun adjustRadius(delta: Float) {
        subjectRadius = (subjectRadius + delta).coerceIn(MIN_RADIUS, MAX_RADIUS)
        pushSubjectVolume()
    }

    /** Hands the current volume to the native side. Safe to call repeatedly. */
    fun pushSubjectVolume() {
        val target = subject
        if (target == null) {
            reconstructor.clearSubjectVolume()
        } else {
            reconstructor.setSubjectVolume(target[0], target[1], target[2], subjectRadius)
        }
    }

    private var lastPosition: FloatArray? = null
    private var lastTimestamp = 0L
    private var speedMetersPerSecond = 0f
    private var lastReject: String? = null

    private val glPose = FloatArray(16)
    private val cvPose = FloatArray(16)
    private val intrinsics = FloatArray(4)
    private val viewMatrix = FloatArray(16)
    private val projectionMatrix = FloatArray(16)
    private val viewProjection = FloatArray(16)
    private val worldPoint = FloatArray(4)
    private val clipPoint = FloatArray(4)

    fun requestShutter() {
        manualShutter.set(true)
    }

    /** Screen tap in pixels; resolved against the scene on the next frame. */
    fun requestSubjectAt(x: Float, y: Float) {
        pendingSubjectTap = floatArrayOf(x, y)
    }

    fun clearSubject() {
        subject = null
        pushSubjectVolume()
    }

    fun reset() {
        coverage.reset()
        subject = null
        lastPosition = null
        lastReject = null
        pushSubjectVolume()
    }

    /**
     * Runs on the GL thread once per rendered frame. Returns the overlay state
     * for the UI to draw.
     */
    fun onFrame(frame: Frame, session: Session, viewportWidth: Int, viewportHeight: Int): Overlay {
        val camera = frame.camera
        val tracking = camera.trackingState == TrackingState.TRACKING
        if (!tracking) {
            manualShutter.set(false)
            return Overlay(
                tracking = false,
                subjectPlaced = subject != null,
                frameCount = reconstructor.frameCount(),
                coverageFraction = coverage.coverage(),
                azimuthCoverage = coverage.azimuthCoverage(),
                hint = "Move the phone slowly to recover tracking",
            )
        }

        camera.pose.toMatrix(glPose, 0)
        val position = floatArrayOf(glPose[12], glPose[13], glPose[14])
        updateSpeed(position, frame.timestamp)

        pendingSubjectTap?.let { tap ->
            subject = resolveSubject(frame, camera, tap)
            pendingSubjectTap = null
            pushSubjectVolume()
        }

        camera.getViewMatrix(viewMatrix, 0)
        camera.getProjectionMatrix(projectionMatrix, 0, NEAR_PLANE, FAR_PLANE)
        Matrix.multiplyMM(viewProjection, 0, projectionMatrix, 0, viewMatrix, 0)

        val target = subject
        var hint = "Point at the subject and tap to lock it in"
        var distance = 0f
        var ringPoints = FloatArray(0)
        var ringCovered = BooleanArray(0)
        var subjectScreen: FloatArray? = null
        var volumePoints = FloatArray(0)

        if (target != null) {
            val toCamera = OrbitCoverage.normalise(
                floatArrayOf(
                    position[0] - target[0],
                    position[1] - target[1],
                    position[2] - target[2],
                )
            )
            distance = OrbitCoverage.distance(position, target)

            val shouldCapture = shouldCapture(toCamera)
            if (shouldCapture) {
                if (ingest(frame, camera, toCamera)) {
                    lastReject = null
                } else {
                    lastReject = "Frame skipped: too blurry"
                }
            }

            val projection = projectRing(target, distance, viewportWidth, viewportHeight)
            ringPoints = projection.first
            ringCovered = projection.second
            subjectScreen = projectPoint(target, viewportWidth, viewportHeight)
            volumePoints = projectVolume(target, viewportWidth, viewportHeight)

            hint = when {
                reconstructor.frameCount() < MIN_FRAMES ->
                    "Orbit the subject - ${MIN_FRAMES - reconstructor.frameCount()} more to go"
                speedMetersPerSecond > MAX_SPEED -> "Slow down"
                else -> coverage.nextHint()
            }
        }

        manualShutter.set(false)

        return Overlay(
            tracking = true,
            subjectPlaced = target != null,
            frameCount = reconstructor.frameCount(),
            coverageFraction = coverage.coverage(),
            azimuthCoverage = coverage.azimuthCoverage(),
            hint = hint,
            distanceMeters = distance,
            ringPoints = ringPoints,
            ringCovered = ringCovered,
            subjectScreen = subjectScreen,
            motionBlurRisk = speedMetersPerSecond > MAX_SPEED,
            lastRejectReason = lastReject,
            volumePoints = volumePoints,
            subjectRadius = subjectRadius,
        )
    }

    private fun shouldCapture(toCamera: FloatArray): Boolean {
        if (manualShutter.get()) return true
        if (!autoCapture) return false
        if (speedMetersPerSecond > MAX_SPEED) return false
        return coverage.angleToNearestCapture(toCamera) >= minSeparationDegrees
    }

    private fun updateSpeed(position: FloatArray, timestampNanos: Long) {
        val previous = lastPosition
        if (previous != null && lastTimestamp != 0L && timestampNanos > lastTimestamp) {
            val seconds = (timestampNanos - lastTimestamp) / 1_000_000_000f
            if (seconds > 1e-4f) {
                val instant = OrbitCoverage.distance(position, previous) / seconds
                // Light smoothing so a single jittery frame cannot block capture.
                speedMetersPerSecond = 0.7f * speedMetersPerSecond + 0.3f * instant
            }
        }
        lastPosition = position
        lastTimestamp = timestampNanos
    }

    /**
     * Places the subject where the operator tapped: an ARCore hit if the scene
     * offers one, otherwise a fixed distance along the tapped ray, which is the
     * common case for a small object on a plain table.
     */
    private fun resolveSubject(frame: Frame, camera: Camera, tap: FloatArray): FloatArray {
        try {
            val hits = frame.hitTest(tap[0], tap[1])
            val hit = hits.firstOrNull()
            if (hit != null) {
                val pose = hit.hitPose
                return floatArrayOf(pose.tx(), pose.ty(), pose.tz())
            }
        } catch (t: Throwable) {
            Log.d(TAG, "Hit test unavailable, using a fixed offset", t)
        }
        camera.pose.toMatrix(glPose, 0)
        // ARCore's camera looks down -Z, so step forward along the negative axis.
        return floatArrayOf(
            glPose[12] - glPose[8] * DEFAULT_SUBJECT_DISTANCE,
            glPose[13] - glPose[9] * DEFAULT_SUBJECT_DISTANCE,
            glPose[14] - glPose[10] * DEFAULT_SUBJECT_DISTANCE,
        )
    }

    private fun ingest(frame: Frame, camera: Camera, toCamera: FloatArray): Boolean {
        val image: Image = try {
            frame.acquireCameraImage()
        } catch (e: NotYetAvailableException) {
            return false
        } catch (t: Throwable) {
            Log.w(TAG, "Could not acquire the camera image", t)
            return false
        }

        image.use { cpuImage ->
            if (cpuImage.format != android.graphics.ImageFormat.YUV_420_888) {
                Log.w(TAG, "Unexpected camera image format ${cpuImage.format}")
                return false
            }
            val planes = cpuImage.planes
            if (planes.size < 3) return false

            camera.pose.toMatrix(glPose, 0)
            toComputerVisionPose(glPose, cvPose)

            val k = camera.imageIntrinsics
            val focal = k.focalLength
            val principal = k.principalPoint
            val dimensions = k.imageDimensions
            // ARCore reports intrinsics for its own CPU image size; rescale if the
            // delivered image differs so fx/cx always match the pixels we send.
            val scaleX = cpuImage.width.toFloat() / dimensions[0].toFloat()
            val scaleY = cpuImage.height.toFloat() / dimensions[1].toFloat()
            intrinsics[0] = focal[0] * scaleX
            intrinsics[1] = focal[1] * scaleY
            intrinsics[2] = principal[0] * scaleX
            intrinsics[3] = principal[1] * scaleY

            val score = reconstructor.addFrame(
                y = planes[0].buffer,
                u = planes[1].buffer,
                v = planes[2].buffer,
                width = cpuImage.width,
                height = cpuImage.height,
                yRowStride = planes[0].rowStride,
                uvRowStride = planes[1].rowStride,
                uvPixelStride = planes[1].pixelStride,
                cameraToWorld = cvPose,
                intrinsics = intrinsics,
                minSharpness = minSharpness,
            )
            if (score < 0f) return false
            coverage.record(toCamera)
            return true
        }
    }

    private fun projectRing(
        target: FloatArray,
        radius: Float,
        viewportWidth: Int,
        viewportHeight: Int,
    ): Pair<FloatArray, BooleanArray> {
        val samples = RING_SAMPLES
        val points = FloatArray(samples * 2)
        val covered = BooleanArray(samples)
        val ringRadius = radius.coerceIn(0.15f, 2.0f)
        val directions = coverage.ringDirections(RING_ELEVATION_DEGREES, samples)

        for (i in 0 until samples) {
            val d = directions[i]
            val world = floatArrayOf(
                target[0] + d[0] * ringRadius,
                target[1] + d[1] * ringRadius,
                target[2] + d[2] * ringRadius,
            )
            val screen = projectPoint(world, viewportWidth, viewportHeight)
            if (screen == null) {
                points[i * 2] = Float.NaN
                points[i * 2 + 1] = Float.NaN
            } else {
                points[i * 2] = screen[0]
                points[i * 2 + 1] = screen[1]
            }
            covered[i] = coverage.isAzimuthCovered(coverage.sectorOfRingSample(i, samples))
        }
        return points to covered
    }

    /**
     * Three orthogonal great circles of the capture volume. Drawing the actual
     * masked region beats an abstract indicator: the operator can see whether
     * the object is fully enclosed before spending two minutes finding out.
     */
    private fun projectVolume(target: FloatArray, width: Int, height: Int): FloatArray {
        val samples = VOLUME_SAMPLES
        val out = FloatArray(3 * samples * 2)
        for (circle in 0 until 3) {
            for (i in 0 until samples) {
                val a = 2.0 * Math.PI * i / samples
                val c = (kotlin.math.cos(a) * subjectRadius).toFloat()
                val s = (kotlin.math.sin(a) * subjectRadius).toFloat()
                val world = when (circle) {
                    0 -> floatArrayOf(target[0] + c, target[1] + s, target[2])
                    1 -> floatArrayOf(target[0], target[1] + c, target[2] + s)
                    else -> floatArrayOf(target[0] + c, target[1], target[2] + s)
                }
                val screen = projectPoint(world, width, height)
                val base = (circle * samples + i) * 2
                if (screen == null) {
                    out[base] = Float.NaN
                    out[base + 1] = Float.NaN
                } else {
                    out[base] = screen[0]
                    out[base + 1] = screen[1]
                }
            }
        }
        return out
    }

    private fun projectPoint(world: FloatArray, width: Int, height: Int): FloatArray? {
        worldPoint[0] = world[0]
        worldPoint[1] = world[1]
        worldPoint[2] = world[2]
        worldPoint[3] = 1f
        Matrix.multiplyMV(clipPoint, 0, viewProjection, 0, worldPoint, 0)
        val w = clipPoint[3]
        if (w <= 1e-5f) return null
        val ndcX = clipPoint[0] / w
        val ndcY = clipPoint[1] / w
        if (abs(ndcX) > 4f || abs(ndcY) > 4f) return null
        return floatArrayOf(
            (ndcX * 0.5f + 0.5f) * width,
            (1f - (ndcY * 0.5f + 0.5f)) * height,
        )
    }

    companion object {
        private const val TAG = "CaptureController"
        private const val NEAR_PLANE = 0.05f
        private const val FAR_PLANE = 20f
        private const val RING_SAMPLES = 96
        private const val RING_ELEVATION_DEGREES = 18f
        private const val DEFAULT_SUBJECT_DISTANCE = 0.45f
        private const val DEFAULT_SUBJECT_RADIUS = 0.15f
        private const val MIN_RADIUS = 0.03f
        private const val MAX_RADIUS = 1.2f
        const val RADIUS_STEP = 0.02f
        const val VOLUME_SAMPLES = 48
        private const val MAX_SPEED = 0.45f
        const val MIN_FRAMES = 18

        /**
         * ARCore hands out an OpenGL-style pose: +X right, +Y up, +Z out of the
         * screen. The reconstruction core works in the computer-vision frame:
         * +X right, +Y down, +Z along the viewing direction. Flipping the last
         * two axes converts between them, and the matrix is transposed from
         * column-major into the row-major layout the JNI bridge expects.
         */
        fun toComputerVisionPose(glColumnMajor: FloatArray, outRowMajor: FloatArray) {
            outRowMajor[0] = glColumnMajor[0]
            outRowMajor[1] = -glColumnMajor[4]
            outRowMajor[2] = -glColumnMajor[8]
            outRowMajor[3] = glColumnMajor[12]

            outRowMajor[4] = glColumnMajor[1]
            outRowMajor[5] = -glColumnMajor[5]
            outRowMajor[6] = -glColumnMajor[9]
            outRowMajor[7] = glColumnMajor[13]

            outRowMajor[8] = glColumnMajor[2]
            outRowMajor[9] = -glColumnMajor[6]
            outRowMajor[10] = -glColumnMajor[10]
            outRowMajor[11] = glColumnMajor[14]

            outRowMajor[12] = 0f
            outRowMajor[13] = 0f
            outRowMajor[14] = 0f
            outRowMajor[15] = 1f
        }
    }
}
