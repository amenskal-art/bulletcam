package com.bulletcam.capture

import android.Manifest
import android.content.pm.PackageManager
import android.opengl.GLSurfaceView
import android.os.Bundle
import android.view.WindowManager
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.core.content.ContextCompat
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.bulletcam.capture.ar.ArSessionController
import com.bulletcam.capture.ui.BulletCamTheme
import com.bulletcam.capture.ui.CaptureScreen
import com.bulletcam.capture.ui.FailureScreen
import com.bulletcam.capture.ui.PermissionScreen
import com.bulletcam.capture.ui.ProcessingScreen
import com.bulletcam.capture.ui.ViewerScreen
import com.bulletcam.capture.vm.CaptureViewModel
import com.bulletcam.capture.vm.Screen
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Frame
import com.google.ar.core.Session

class MainActivity : ComponentActivity(), ArSessionController.Listener {

    private val viewModel: CaptureViewModel by viewModels()

    private lateinit var glSurfaceView: GLSurfaceView
    private lateinit var arController: ArSessionController

    private var cameraGranted by mutableStateOf(false)
    private var arCoreInstallRequested = false
    private var sessionStarted = false

    private val permissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
            cameraGranted = granted
            if (granted) startArSession()
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        arController = ArSessionController(this, this)
        glSurfaceView = GLSurfaceView(this).apply {
            preserveEGLContextOnPause = true
            setEGLContextClientVersion(2)
            setEGLConfigChooser(8, 8, 8, 8, 16, 0)
            setRenderer(arController)
            renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
        }

        cameraGranted = ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED

        setContent {
            BulletCamTheme {
                val screen by viewModel.screen.collectAsStateWithLifecycle()
                val message by viewModel.sessionMessage.collectAsStateWithLifecycle()

                LaunchedEffect(message) {
                    message?.let {
                        Toast.makeText(this@MainActivity, it, Toast.LENGTH_LONG).show()
                        viewModel.dismissSessionMessage()
                    }
                }

                if (!cameraGranted) {
                    PermissionScreen(
                        rationale = "BulletCam needs the camera to photograph your subject. " +
                            "Nothing leaves the device: the app has no internet permission and " +
                            "every step of the reconstruction runs here.",
                        onRequest = { permissionLauncher.launch(Manifest.permission.CAMERA) },
                    )
                    return@BulletCamTheme
                }

                when (val current = screen) {
                    is Screen.Capture -> CaptureScreen(
                        viewModel = viewModel,
                        glSurfaceView = glSurfaceView,
                        onToggleExposureLock = { arController.setExposureLocked(it) },
                    )

                    is Screen.Processing -> ProcessingScreen(
                        stage = current.stage,
                        fraction = current.fraction,
                        onCancel = { viewModel.cancelReconstruction() },
                    )

                    is Screen.Result -> ViewerScreen(
                        objFile = current.objFile,
                        glbFile = current.glbFile,
                        summary = current.summary,
                        onNewScan = { viewModel.backToCapture(freshSession = true) },
                        onBack = { viewModel.backToCapture(freshSession = false) },
                    )

                    is Screen.Failed -> FailureScreen(
                        reason = current.reason,
                        onRetry = { viewModel.backToCapture(freshSession = false) },
                        onStartOver = { viewModel.backToCapture(freshSession = true) },
                    )
                }
            }
        }
    }

    override fun onResume() {
        super.onResume()
        if (cameraGranted) startArSession()
        glSurfaceView.onResume()
    }

    override fun onPause() {
        super.onPause()
        glSurfaceView.onPause()
        arController.pause()
        sessionStarted = false
    }

    override fun onDestroy() {
        arController.close()
        super.onDestroy()
    }

    private fun startArSession() {
        if (sessionStarted) return
        val installStatus = try {
            ArCoreApk.getInstance().requestInstall(this, !arCoreInstallRequested)
        } catch (t: Throwable) {
            viewModel.onSessionMessage(
                t.message ?: "ARCore is required and could not be installed on this device"
            )
            return
        }
        if (installStatus == ArCoreApk.InstallStatus.INSTALL_REQUESTED) {
            arCoreInstallRequested = true
            return
        }

        if (arController.session == null) {
            // Try the shared-camera path first: it is the only way to pin
            // exposure and focus, which is what keeps the texture even.
            if (!arController.createSession(useSharedCamera = true)) {
                if (!arController.createSession(useSharedCamera = false)) return
            }
        }
        arController.resume()
        sessionStarted = true
    }

    // --- ArSessionController.Listener ---------------------------------------

    override fun onArFrame(frame: Frame, session: Session, viewportWidth: Int, viewportHeight: Int) {
        viewModel.onArFrame(frame, session, viewportWidth, viewportHeight)
    }

    override fun onSessionMessage(message: String) {
        runOnUiThread { viewModel.onSessionMessage(message) }
    }

    override fun onExposureLockChanged(locked: Boolean, supported: Boolean) {
        runOnUiThread { viewModel.onExposureLockChanged(locked, supported) }
    }
}
