package com.bulletcam.capture.ar

import android.app.Activity
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.Build
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import com.google.ar.core.CameraConfig
import com.google.ar.core.CameraConfigFilter
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import java.util.EnumSet
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

/**
 * Owns the ARCore session and the GL preview.
 *
 * Two things matter for photogrammetry and both are set up here: the camera
 * config with the largest CPU image ARCore will give us, and - where the device
 * allows it - a Camera2 interop path that pins exposure and focus so the photo
 * set stays photometrically consistent.
 */
class ArSessionController(
    private val activity: Activity,
    private val listener: Listener,
) : GLSurfaceView.Renderer {

    interface Listener {
        /** Called on the GL thread once per rendered frame. */
        fun onArFrame(frame: Frame, session: Session, viewportWidth: Int, viewportHeight: Int)

        /** Called on the main thread. */
        fun onSessionMessage(message: String)

        fun onExposureLockChanged(locked: Boolean, supported: Boolean)
    }

    var session: Session? = null
        private set

    private val background = BackgroundRenderer()
    private var cameraLock: SharedCameraLock? = null
    private var cameraThread: HandlerThread? = null
    private var cameraHandler: Handler? = null

    private var viewportWidth = 1
    private var viewportHeight = 1
    private var viewportChanged = false
    private var displayRotation = 0
    private var sharedCameraActive = false
    private var arCoreResumed = false
    private var glReady = false

    /** Image size of the frames handed to the reconstruction core. */
    var captureImageSize: Pair<Int, Int> = 0 to 0
        private set

    var exposureLocked: Boolean = false
        private set

    fun createSession(useSharedCamera: Boolean): Boolean {
        return try {
            val features =
                if (useSharedCamera) EnumSet.of(Session.Feature.SHARED_CAMERA) else EnumSet.noneOf(
                    Session.Feature::class.java
                )
            val created = Session(activity, features)
            configure(created)
            session = created
            sharedCameraActive = useSharedCamera
            true
        } catch (t: Throwable) {
            Log.e(TAG, "Session creation failed", t)
            listener.onSessionMessage(t.message ?: "ARCore session could not start")
            false
        }
    }

    private fun configure(target: Session) {
        // Pick the config with the most CPU pixels: texture detail is bounded by
        // this, and plane detection or depth would only cost us frame time.
        val filter = CameraConfigFilter(target)
            .setFacingDirection(CameraConfig.FacingDirection.BACK)
            .setTargetFps(EnumSet.of(CameraConfig.TargetFps.TARGET_FPS_30))
        val configs = target.getSupportedCameraConfigs(filter)
        val best = configs.maxByOrNull { it.imageSize.width.toLong() * it.imageSize.height }
        if (best != null) {
            target.cameraConfig = best
            captureImageSize = best.imageSize.width to best.imageSize.height
        }

        val config = Config(target).apply {
            // Fixed focus keeps the intrinsics constant across the whole orbit.
            focusMode = Config.FocusMode.FIXED
            planeFindingMode = Config.PlaneFindingMode.DISABLED
            lightEstimationMode = Config.LightEstimationMode.DISABLED
            updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
            if (target.isDepthModeSupported(Config.DepthMode.AUTOMATIC)) {
                depthMode = Config.DepthMode.AUTOMATIC
            }
        }
        target.configure(config)
    }

    fun resume() {
        val target = session ?: return
        updateDisplayRotation()
        if (sharedCameraActive) {
            // The shared path resumes ARCore only once Camera2 has a live
            // capture session, otherwise ARCore and the app fight over the device.
            startCameraThread()
            if (glReady) openSharedCamera(target)
        } else {
            resumeArCore(target)
        }
    }

    private fun resumeArCore(target: Session) {
        if (arCoreResumed) return
        try {
            target.resume()
            arCoreResumed = true
        } catch (t: Throwable) {
            Log.e(TAG, "Session resume failed", t)
            listener.onSessionMessage("Could not start the camera: ${t.message}")
        }
    }

    private fun openSharedCamera(target: Session) {
        if (cameraLock != null) return
        val handler = cameraHandler ?: return
        val lock = SharedCameraLock(
            context = activity,
            session = target,
            handler = handler,
            onCaptureSessionActive = {
                activity.runOnUiThread {
                    resumeArCore(target)
                    listener.onExposureLockChanged(exposureLocked, true)
                }
            },
            onFailure = { throwable ->
                Log.w(TAG, "Shared camera unavailable, falling back", throwable)
                activity.runOnUiThread { fallBackToPlainSession() }
            },
        )
        if (!lock.deviceSupportsLock()) {
            Log.i(TAG, "Device is LEGACY; exposure lock is unavailable")
            fallBackToPlainSession()
            return
        }
        cameraLock = lock
        lock.open()
    }

    private fun fallBackToPlainSession() {
        cameraLock?.close()
        cameraLock = null
        session?.let {
            try {
                it.pause()
            } catch (ignored: Throwable) {
            }
            it.close()
        }
        session = null
        arCoreResumed = false
        sharedCameraActive = false
        listener.onExposureLockChanged(locked = false, supported = false)
        if (createSession(useSharedCamera = false)) {
            if (glReady) session?.setCameraTextureName(background.textureId)
            resume()
        }
    }

    /** Freezes exposure, white balance and focus for the rest of the session. */
    fun setExposureLocked(locked: Boolean) {
        exposureLocked = locked
        cameraLock?.setLocked(locked)
        listener.onExposureLockChanged(locked, cameraLock != null)
    }

    fun pause() {
        try {
            session?.pause()
        } catch (t: Throwable) {
            Log.w(TAG, "Session pause failed", t)
        }
        arCoreResumed = false
        cameraLock?.close()
        cameraLock = null
        stopCameraThread()
    }

    fun close() {
        pause()
        session?.close()
        session = null
    }

    private fun startCameraThread() {
        if (cameraThread != null) return
        cameraThread = HandlerThread("BulletCamCamera").apply {
            start()
            cameraHandler = Handler(looper)
        }
    }

    private fun stopCameraThread() {
        cameraThread?.quitSafely()
        cameraThread = null
        cameraHandler = null
    }

    private fun updateDisplayRotation() {
        displayRotation = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            activity.display?.rotation ?: 0
        } else {
            @Suppress("DEPRECATION")
            activity.windowManager.defaultDisplay.rotation
        }
        viewportChanged = true
    }

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.04f, 0.05f, 0.07f, 1f)
        background.createOnGlThread()
        glReady = true
        val target = session
        if (target != null) {
            target.setCameraTextureName(background.textureId)
            if (sharedCameraActive) {
                activity.runOnUiThread { openSharedCamera(target) }
            }
        }
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        viewportWidth = width
        viewportHeight = height
        updateDisplayRotation()
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        val target = session ?: return
        if (!arCoreResumed) return

        if (viewportChanged) {
            target.setDisplayGeometry(displayRotation, viewportWidth, viewportHeight)
            viewportChanged = false
        }

        val frame = try {
            target.setCameraTextureName(background.textureId)
            target.update()
        } catch (t: Throwable) {
            Log.w(TAG, "Frame update failed", t)
            return
        }

        background.draw(frame)
        if (frame.camera.trackingState == TrackingState.TRACKING || frame.timestamp != 0L) {
            listener.onArFrame(frame, target, viewportWidth, viewportHeight)
        }
    }

    private companion object {
        const val TAG = "ArSessionController"
    }
}
