package com.bulletcam.capture

import android.app.Application
import com.bulletcam.capture.recon.ReconstructionService

class BulletCamApp : Application() {
    override fun onCreate() {
        super.onCreate()
        // Registered up front so the service can go foreground the instant it
        // starts, which Android requires.
        ReconstructionService.createChannel(this)
    }
}
