package com.bulletcam.capture.recon

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.IBinder
import android.os.PowerManager
import androidx.core.app.NotificationCompat
import androidx.core.content.ContextCompat
import com.bulletcam.capture.MainActivity
import com.bulletcam.capture.R
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch
import java.io.File
import kotlin.math.roundToInt

/**
 * Keeps a reconstruction alive while the app is in the background.
 *
 * Android will freeze ordinary background work within seconds of the app losing
 * visibility, and a multi-minute CPU job is exactly what the OS is looking to
 * kill. A foreground service with a progress notification is the supported way
 * to say "this is work the user asked for and can see", and the partial wake
 * lock stops the CPU idling out with the screen off.
 */
class ReconstructionService : Service() {

    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Main.immediate)
    private var wakeLock: PowerManager.WakeLock? = null
    private var observing = false

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        // Android gives a freshly started service only a few seconds to go
        // foreground or it kills the process, and that clock starts even for the
        // cancel intent, so this happens before anything else.
        startForegroundWith(buildProgressNotification("Starting", 0f))

        if (intent?.action == ACTION_CANCEL) {
            ReconstructionEngine.cancel()
            if (!ReconstructionEngine.isRunning) {
                releaseWakeLock()
                stopForegroundCompat()
                stopSelf()
            }
            return START_NOT_STICKY
        }

        acquireWakeLock()

        val objPath = intent?.getStringExtra(EXTRA_OBJ_PATH)
        val glbPath = intent?.getStringExtra(EXTRA_GLB_PATH)
        if (objPath != null && glbPath != null) {
            ReconstructionEngine.start(File(objPath), File(glbPath))
        }

        if (!observing) {
            observing = true
            scope.launch {
                ReconstructionEngine.state.collectLatest { state -> onState(state) }
            }
        }
        // Not STICKY: a restarted service with no intent has no job to resume,
        // and the native session would be gone with the process anyway.
        return START_NOT_STICKY
    }

    private fun onState(state: ReconstructionEngine.State) {
        val manager = getSystemService(NotificationManager::class.java)
        when (state) {
            is ReconstructionEngine.State.Running ->
                manager.notify(
                    NOTIFICATION_ID,
                    buildProgressNotification(state.stage, state.fraction),
                )

            is ReconstructionEngine.State.Done -> {
                finishWith("Model ready", state.summary)
            }

            is ReconstructionEngine.State.Failed -> {
                finishWith("Reconstruction stopped", state.reason)
            }

            ReconstructionEngine.State.Idle -> Unit
        }
    }

    private fun finishWith(title: String, body: String) {
        val manager = getSystemService(NotificationManager::class.java)
        releaseWakeLock()
        stopForegroundCompat()
        manager.notify(RESULT_NOTIFICATION_ID, buildResultNotification(title, body))
        stopSelf()
    }

    private fun startForegroundWith(notification: Notification) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.UPSIDE_DOWN_CAKE) {
            startForeground(
                NOTIFICATION_ID,
                notification,
                ServiceInfo.FOREGROUND_SERVICE_TYPE_DATA_SYNC,
            )
        } else {
            startForeground(NOTIFICATION_ID, notification)
        }
    }

    private fun stopForegroundCompat() {
        stopForeground(STOP_FOREGROUND_REMOVE)
    }

    private fun contentIntent(): PendingIntent {
        val intent = Intent(this, MainActivity::class.java).apply {
            flags = Intent.FLAG_ACTIVITY_SINGLE_TOP or Intent.FLAG_ACTIVITY_CLEAR_TOP
        }
        return PendingIntent.getActivity(
            this, 0, intent, PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE,
        )
    }

    private fun buildProgressNotification(stage: String, fraction: Float): Notification {
        val percent = (fraction.coerceIn(0f, 1f) * 100f).roundToInt()
        val cancelIntent = PendingIntent.getService(
            this,
            1,
            Intent(this, ReconstructionService::class.java).setAction(ACTION_CANCEL),
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE,
        )
        return NotificationCompat.Builder(this, CHANNEL_ID)
            .setSmallIcon(R.drawable.ic_notification)
            .setContentTitle("Building model - $percent%")
            .setContentText(stage)
            .setProgress(100, percent, false)
            .setOngoing(true)
            .setOnlyAlertOnce(true)
            .setForegroundServiceBehavior(NotificationCompat.FOREGROUND_SERVICE_IMMEDIATE)
            .setPriority(NotificationCompat.PRIORITY_LOW)
            .setContentIntent(contentIntent())
            .addAction(0, "Cancel", cancelIntent)
            .build()
    }

    private fun buildResultNotification(title: String, body: String): Notification =
        NotificationCompat.Builder(this, CHANNEL_ID)
            .setSmallIcon(R.drawable.ic_notification)
            .setContentTitle(title)
            .setContentText(body)
            .setStyle(NotificationCompat.BigTextStyle().bigText(body))
            .setAutoCancel(true)
            .setPriority(NotificationCompat.PRIORITY_DEFAULT)
            .setContentIntent(contentIntent())
            .build()

    private fun acquireWakeLock() {
        if (wakeLock?.isHeld == true) return
        val power = getSystemService(Context.POWER_SERVICE) as PowerManager
        wakeLock = power.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "BulletCam:reconstruction")
            .apply {
                setReferenceCounted(false)
                // Generous but bounded: a very high quality run on a slow device
                // should still finish, and the lock is released on completion.
                acquire(60L * 60L * 1000L)
            }
    }

    private fun releaseWakeLock() {
        wakeLock?.let { if (it.isHeld) it.release() }
        wakeLock = null
    }

    override fun onDestroy() {
        releaseWakeLock()
        scope.cancel()
        super.onDestroy()
    }

    companion object {
        private const val NOTIFICATION_ID = 4201
        private const val RESULT_NOTIFICATION_ID = 4202
        const val CHANNEL_ID = "reconstruction"
        private const val ACTION_CANCEL = "com.bulletcam.capture.CANCEL_RECONSTRUCTION"
        private const val EXTRA_OBJ_PATH = "obj"
        private const val EXTRA_GLB_PATH = "glb"

        fun createChannel(context: Context) {
            val channel = NotificationChannel(
                CHANNEL_ID,
                "Model reconstruction",
                NotificationManager.IMPORTANCE_LOW,
            ).apply {
                description = "Progress while a captured object is turned into a 3D model"
                setShowBadge(false)
            }
            context.getSystemService(NotificationManager::class.java)
                .createNotificationChannel(channel)
        }

        fun start(context: Context, objFile: File, glbFile: File) {
            val intent = Intent(context, ReconstructionService::class.java).apply {
                putExtra(EXTRA_OBJ_PATH, objFile.absolutePath)
                putExtra(EXTRA_GLB_PATH, glbFile.absolutePath)
            }
            ContextCompat.startForegroundService(context, intent)
        }
    }
}
