package com.bulletcam.capture.recon

import android.util.Log
import com.bulletcam.capture.nativebridge.NativeReconstructor
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import java.io.File

/**
 * Process-scoped owner of the native session and the reconstruction job.
 *
 * The work deliberately does not live in a ViewModel. A reconstruction runs for
 * minutes, and a ViewModel-scoped coroutine dies with the Activity while the
 * OS throttles plain background threads the moment the app stops being visible.
 * Keeping the job here lets [ReconstructionService] hold it up with a foreground
 * notification, and lets a freshly recreated Activity reattach to a run that is
 * already halfway through.
 */
object ReconstructionEngine {

    sealed interface State {
        data object Idle : State
        data class Running(val stage: String, val fraction: Float) : State
        data class Done(val objFile: File, val glbFile: File, val summary: String) : State
        data class Failed(val reason: String) : State
    }

    val reconstructor = NativeReconstructor()

    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Default)
    private var job: Job? = null

    private val _state = MutableStateFlow<State>(State.Idle)
    val state: StateFlow<State> = _state.asStateFlow()

    val isRunning: Boolean get() = _state.value is State.Running

    @Synchronized
    fun openSession(workDir: File, options: NativeReconstructor.Options) {
        if (reconstructor.isOpen) return
        workDir.deleteRecursively()
        workDir.mkdirs()
        reconstructor.open(workDir.absolutePath, options)
    }

    /** Discards the native session so the next open picks up new options. */
    @Synchronized
    fun resetSession(workDir: File, options: NativeReconstructor.Options) {
        if (isRunning) return
        reconstructor.close()
        _state.value = State.Idle
        openSession(workDir, options)
    }

    /**
     * Starts a run. Safe to call twice: a second call while one is in flight is
     * ignored rather than corrupting the native session.
     */
    @Synchronized
    fun start(objFile: File, glbFile: File) {
        if (isRunning) return
        _state.value = State.Running("Starting", 0f)
        job = scope.launch {
            val success = try {
                reconstructor.reconstruct(objFile.absolutePath, glbFile.absolutePath) { stage, value ->
                    _state.value = State.Running(stage, value)
                }
            } catch (t: Throwable) {
                Log.e(TAG, "Reconstruction threw", t)
                false
            }
            val message = reconstructor.lastMessage()
            _state.value = if (success && objFile.exists()) {
                State.Done(objFile, glbFile, message.ifBlank { "Model ready" })
            } else {
                State.Failed(message.ifBlank { "Reconstruction failed" })
            }
        }
    }

    fun cancel() {
        reconstructor.cancel()
    }

    /** Clears a terminal state once the UI has shown it. */
    @Synchronized
    fun acknowledge() {
        if (_state.value is State.Running) return
        _state.value = State.Idle
    }

    private const val TAG = "ReconstructionEngine"
}
