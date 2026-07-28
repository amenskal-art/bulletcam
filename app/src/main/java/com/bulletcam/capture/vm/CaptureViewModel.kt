package com.bulletcam.capture.vm

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.bulletcam.capture.capture.CaptureController
import com.bulletcam.capture.capture.OrbitCoverage
import com.bulletcam.capture.nativebridge.NativeReconstructor
import com.google.ar.core.Frame
import com.google.ar.core.Session
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

sealed interface Screen {
    data object Capture : Screen
    data class Processing(val stage: String, val fraction: Float) : Screen
    data class Result(val objFile: File, val glbFile: File, val summary: String) : Screen
    data class Failed(val reason: String) : Screen
}

enum class Quality { Fast, Balanced, High }

class CaptureViewModel(application: Application) : AndroidViewModel(application) {

    private val reconstructor = NativeReconstructor()
    private val coverage = OrbitCoverage()
    val capture = CaptureController(reconstructor, coverage)

    private val _screen = MutableStateFlow<Screen>(Screen.Capture)
    val screen: StateFlow<Screen> = _screen.asStateFlow()

    private val _overlay = MutableStateFlow(CaptureController.Overlay())
    val overlay: StateFlow<CaptureController.Overlay> = _overlay.asStateFlow()

    private val _quality = MutableStateFlow(Quality.Balanced)
    val quality: StateFlow<Quality> = _quality.asStateFlow()

    private val _exposureLocked = MutableStateFlow(false)
    val exposureLocked: StateFlow<Boolean> = _exposureLocked.asStateFlow()

    private val _exposureLockSupported = MutableStateFlow(false)
    val exposureLockSupported: StateFlow<Boolean> = _exposureLockSupported.asStateFlow()

    private val _sessionMessage = MutableStateFlow<String?>(null)
    val sessionMessage: StateFlow<String?> = _sessionMessage.asStateFlow()

    private var workDir: File = File(application.cacheDir, "reconstruction").apply { mkdirs() }

    init {
        openSession()
    }

    private fun openSession() {
        if (reconstructor.isOpen) return
        workDir.deleteRecursively()
        workDir.mkdirs()
        reconstructor.open(workDir.absolutePath, optionsFor(_quality.value))
    }

    private fun optionsFor(quality: Quality) = when (quality) {
        Quality.Fast -> NativeReconstructor.Options.Fast
        Quality.Balanced -> NativeReconstructor.Options.Balanced
        Quality.High -> NativeReconstructor.Options.High
    }

    fun setQuality(value: Quality) {
        if (_quality.value == value) return
        _quality.value = value
        // Options are fixed for the lifetime of a native session, so restart it.
        // Photos already taken would have been converted at the old settings.
        reconstructor.close()
        coverage.reset()
        capture.reset()
        openSession()
    }

    /** Called from the GL thread. */
    fun onArFrame(frame: Frame, session: Session, width: Int, height: Int) {
        if (_screen.value !is Screen.Capture) return
        _overlay.value = capture.onFrame(frame, session, width, height)
    }

    fun onSessionMessage(message: String) {
        _sessionMessage.value = message
    }

    fun dismissSessionMessage() {
        _sessionMessage.value = null
    }

    fun onExposureLockChanged(locked: Boolean, supported: Boolean) {
        _exposureLocked.value = locked
        _exposureLockSupported.value = supported
    }

    fun discardCaptures() {
        reconstructor.discardAll()
        capture.reset()
        _overlay.value = CaptureController.Overlay()
    }

    fun startReconstruction() {
        if (_screen.value is Screen.Processing) return
        val frames = reconstructor.frameCount()
        if (frames < CaptureController.MIN_FRAMES) {
            _sessionMessage.value =
                "Capture at least ${CaptureController.MIN_FRAMES} photos before building the model"
            return
        }

        val stamp = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())
        val outputDir = File(getApplication<Application>().getExternalFilesDir(null), "models/$stamp")
        outputDir.mkdirs()
        val objFile = File(outputDir, "model.obj")
        val glbFile = File(outputDir, "model.glb")

        _screen.value = Screen.Processing("Starting", 0f)
        viewModelScope.launch {
            val success = withContext(Dispatchers.Default) {
                reconstructor.reconstruct(objFile.absolutePath, glbFile.absolutePath) { stage, value ->
                    _screen.value = Screen.Processing(stage, value)
                }
            }
            val message = reconstructor.lastMessage()
            _screen.value = if (success && objFile.exists()) {
                Screen.Result(objFile, glbFile, message.ifBlank { "Model ready" })
            } else {
                Screen.Failed(message.ifBlank { "Reconstruction failed" })
            }
        }
    }

    fun cancelReconstruction() {
        reconstructor.cancel()
    }

    fun backToCapture(freshSession: Boolean) {
        if (freshSession) {
            reconstructor.close()
            coverage.reset()
            capture.reset()
            openSession()
            _overlay.value = CaptureController.Overlay()
        }
        _screen.value = Screen.Capture
    }

    override fun onCleared() {
        reconstructor.cancel()
        reconstructor.close()
        super.onCleared()
    }
}
