package com.bulletcam.capture.vm

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.bulletcam.capture.capture.CaptureController
import com.bulletcam.capture.capture.OrbitCoverage
import com.bulletcam.capture.nativebridge.NativeReconstructor
import com.bulletcam.capture.recon.ReconstructionEngine
import com.bulletcam.capture.recon.ReconstructionService
import com.google.ar.core.Frame
import com.google.ar.core.Session
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.stateIn
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

    private val reconstructor: NativeReconstructor = ReconstructionEngine.reconstructor
    private val coverage = OrbitCoverage()
    val capture = CaptureController(reconstructor, coverage)

    /**
     * Screen state is derived from the engine rather than held here, so an
     * Activity recreated while a reconstruction is running - returning to the
     * app from recents, or after a rotation - lands straight back on the
     * progress screen instead of a blank capture view.
     */
    val screen: StateFlow<Screen> = ReconstructionEngine.state
        .map { state ->
            when (state) {
                ReconstructionEngine.State.Idle -> Screen.Capture
                is ReconstructionEngine.State.Running -> Screen.Processing(state.stage, state.fraction)
                is ReconstructionEngine.State.Done ->
                    Screen.Result(state.objFile, state.glbFile, state.summary)

                is ReconstructionEngine.State.Failed -> Screen.Failed(state.reason)
            }
        }
        .stateIn(viewModelScope, SharingStarted.Eagerly, Screen.Capture)

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

    private val workDir = File(application.cacheDir, "reconstruction")

    init {
        ReconstructionEngine.openSession(workDir, optionsFor(_quality.value))
    }

    private fun optionsFor(quality: Quality) = when (quality) {
        Quality.Fast -> NativeReconstructor.Options.Fast
        Quality.Balanced -> NativeReconstructor.Options.Balanced
        Quality.High -> NativeReconstructor.Options.High
    }

    fun setQuality(value: Quality) {
        if (_quality.value == value || ReconstructionEngine.isRunning) return
        _quality.value = value
        // Image sizes are baked into the frames already converted, so changing
        // them means starting the photo set over.
        ReconstructionEngine.resetSession(workDir, optionsFor(value))
        coverage.reset()
        capture.reset()
        _overlay.value = CaptureController.Overlay()
    }

    /** Called from the GL thread. */
    fun onArFrame(frame: Frame, session: Session, width: Int, height: Int) {
        if (screen.value !is Screen.Capture) return
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

    fun adjustSubjectRadius(delta: Float) {
        capture.adjustRadius(delta)
    }

    fun discardCaptures() {
        if (ReconstructionEngine.isRunning) return
        reconstructor.discardAll()
        capture.reset()
        _overlay.value = CaptureController.Overlay()
    }

    fun startReconstruction() {
        if (ReconstructionEngine.isRunning) return
        val frames = reconstructor.frameCount()
        if (frames < CaptureController.MIN_FRAMES) {
            _sessionMessage.value =
                "Capture at least ${CaptureController.MIN_FRAMES} photos before building the model"
            return
        }

        // Make sure the mask the operator was shown is the one that gets used.
        capture.pushSubjectVolume()

        val stamp = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())
        val outputDir = File(getApplication<Application>().getExternalFilesDir(null), "models/$stamp")
        outputDir.mkdirs()

        ReconstructionService.start(
            context = getApplication<Application>(),
            objFile = File(outputDir, "model.obj"),
            glbFile = File(outputDir, "model.glb"),
        )
    }

    fun cancelReconstruction() {
        ReconstructionEngine.cancel()
    }

    fun backToCapture(freshSession: Boolean) {
        ReconstructionEngine.acknowledge()
        if (freshSession) {
            ReconstructionEngine.resetSession(workDir, optionsFor(_quality.value))
            coverage.reset()
            capture.reset()
            _overlay.value = CaptureController.Overlay()
        }
    }

    // Deliberately no cleanup of the native session here: it outlives this
    // ViewModel so a reconstruction survives the Activity being destroyed.
}
