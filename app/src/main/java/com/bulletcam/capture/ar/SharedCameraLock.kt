package com.bulletcam.capture.ar

import android.annotation.SuppressLint
import android.content.Context
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.CaptureResult
import android.hardware.camera2.TotalCaptureResult
import android.os.Handler
import android.util.Log
import com.google.ar.core.Session

/**
 * Holds the camera open through ARCore's shared-camera interop so exposure,
 * white balance and focus can be pinned for the whole capture session.
 *
 * ARCore has no API of its own for auto-exposure lock, and unlocked AE is the
 * single biggest cause of blotchy photogrammetry textures: the same surface
 * comes back a different brightness in every photo. Camera2 keys are the only
 * way to stop that, which is what this class exists for.
 *
 * Every step is guarded - if any device refuses the interop path, [onFailure]
 * fires and the caller falls back to a plain ARCore session.
 */
class SharedCameraLock(
    private val context: Context,
    private val session: Session,
    private val handler: Handler,
    private val onCaptureSessionActive: () -> Unit,
    private val onFailure: (Throwable) -> Unit,
) {

    private val cameraManager =
        context.getSystemService(Context.CAMERA_SERVICE) as CameraManager

    private var cameraDevice: CameraDevice? = null
    private var captureSession: CameraCaptureSession? = null
    private var requestBuilder: CaptureRequest.Builder? = null

    /** Focus distance reported by the last auto-focus result, in dioptres. */
    private var lastFocusDistance: Float? = null
    private var locked = false

    val isActive: Boolean get() = captureSession != null

    @SuppressLint("MissingPermission")
    fun open() {
        try {
            val sharedCamera = session.sharedCamera
            val cameraId = session.cameraConfig.cameraId
            val wrapped = sharedCamera.createARDeviceStateCallback(deviceCallback, handler)
            cameraManager.openCamera(cameraId, wrapped, handler)
        } catch (t: Throwable) {
            onFailure(t)
        }
    }

    fun close() {
        try {
            captureSession?.close()
        } catch (t: Throwable) {
            Log.w(TAG, "Capture session close failed", t)
        }
        captureSession = null
        try {
            cameraDevice?.close()
        } catch (t: Throwable) {
            Log.w(TAG, "Camera close failed", t)
        }
        cameraDevice = null
        requestBuilder = null
        locked = false
    }

    /**
     * Freezes exposure, white balance and focus at whatever the auto algorithms
     * have converged on, so the whole photo set shares one photometric
     * calibration.
     *
     * ARCore forbids calling setRepeatingRequest while it is active, so the
     * caller must pause the ARCore session around this call. ArSessionController
     * does that on the GL thread between frames.
     */
    fun applyLock(shouldLock: Boolean) {
        if (locked == shouldLock) return
        locked = shouldLock
        applyRequest()
    }

    /**
     * Hands the capture callback to ARCore once it takes over the repeating
     * request. Without this the app stops seeing capture metadata - including
     * the focus distance the lock needs to pin - the moment ARCore resumes.
     */
    fun onArCoreResumed() {
        try {
            session.sharedCamera.setCaptureCallback(captureCallback, handler)
        } catch (t: Throwable) {
            Log.w(TAG, "Could not register the ARCore capture callback", t)
        }
    }

    private fun applyRequest() {
        val builder = requestBuilder ?: return
        val target = captureSession ?: return
        builder.set(CaptureRequest.CONTROL_AE_LOCK, locked)
        builder.set(CaptureRequest.CONTROL_AWB_LOCK, locked)
        if (locked) {
            builder.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF)
            lastFocusDistance?.let { builder.set(CaptureRequest.LENS_FOCUS_DISTANCE, it) }
        } else {
            builder.set(
                CaptureRequest.CONTROL_AF_MODE,
                CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE,
            )
        }
        try {
            // Legal here only because ARCore is paused; see applyLock.
            target.setRepeatingRequest(builder.build(), captureCallback, handler)
        } catch (t: Throwable) {
            Log.w(TAG, "Could not update the repeating request", t)
        }
    }

    private val deviceCallback = object : CameraDevice.StateCallback() {
        override fun onOpened(device: CameraDevice) {
            cameraDevice = device
            try {
                // TEMPLATE_RECORD is what ARCore expects for shared sessions.
                val builder = device.createCaptureRequest(CameraDevice.TEMPLATE_RECORD)
                val surfaces = session.sharedCamera.arCoreSurfaces
                surfaces.forEach { builder.addTarget(it) }
                requestBuilder = builder
                val stateCallback =
                    session.sharedCamera.createARSessionStateCallback(sessionCallback, handler)
                @Suppress("DEPRECATION")
                device.createCaptureSession(surfaces, stateCallback, handler)
            } catch (t: Throwable) {
                onFailure(t)
            }
        }

        override fun onDisconnected(device: CameraDevice) {
            device.close()
            cameraDevice = null
        }

        override fun onError(device: CameraDevice, error: Int) {
            device.close()
            cameraDevice = null
            onFailure(IllegalStateException("Camera device error $error"))
        }
    }

    private val sessionCallback = object : CameraCaptureSession.StateCallback() {
        override fun onConfigured(target: CameraCaptureSession) {
            captureSession = target
            applyRequest()
            onCaptureSessionActive()
        }

        override fun onConfigureFailed(target: CameraCaptureSession) {
            onFailure(IllegalStateException("Camera capture session configuration failed"))
        }
    }

    private val captureCallback = object : CameraCaptureSession.CaptureCallback() {
        override fun onCaptureCompleted(
            target: CameraCaptureSession,
            request: CaptureRequest,
            result: TotalCaptureResult,
        ) {
            if (!locked) {
                // Remember where continuous AF settled so the lock can hold it.
                result.get(CaptureResult.LENS_FOCUS_DISTANCE)?.let { lastFocusDistance = it }
            }
        }
    }

    /** True when the device exposes the manual controls the lock depends on. */
    fun deviceSupportsLock(): Boolean = try {
        val id = session.cameraConfig.cameraId
        val characteristics = cameraManager.getCameraCharacteristics(id)
        val level = characteristics.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL)
        level != CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY
    } catch (t: Throwable) {
        false
    }

    private companion object {
        const val TAG = "SharedCameraLock"
    }
}
