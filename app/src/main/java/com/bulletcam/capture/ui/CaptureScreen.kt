package com.bulletcam.capture.ui

import android.opengl.GLSurfaceView
import android.view.ViewGroup
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.bulletcam.capture.capture.CaptureController
import com.bulletcam.capture.vm.CaptureViewModel
import com.bulletcam.capture.vm.Quality
import kotlin.math.roundToInt

@Composable
fun CaptureScreen(
    viewModel: CaptureViewModel,
    glSurfaceView: GLSurfaceView,
    onToggleExposureLock: (Boolean) -> Unit,
) {
    val overlay by viewModel.overlay.collectAsStateWithLifecycle()
    val quality by viewModel.quality.collectAsStateWithLifecycle()
    val exposureLocked by viewModel.exposureLocked.collectAsStateWithLifecycle()
    val exposureSupported by viewModel.exposureLockSupported.collectAsStateWithLifecycle()

    Box(modifier = Modifier.fillMaxSize().background(Ink.Graphite)) {
        AndroidView(
            modifier = Modifier
                .fillMaxSize()
                .pointerInput(Unit) {
                    detectTapGestures { offset ->
                        viewModel.capture.requestSubjectAt(offset.x, offset.y)
                    }
                },
            factory = { _ ->
                (glSurfaceView.parent as? ViewGroup)?.removeView(glSurfaceView)
                glSurfaceView
            },
        )

        OrbitOverlay(overlay)

        Column(
            modifier = Modifier
                .align(Alignment.TopCenter)
                .fillMaxWidth()
                .padding(horizontal = 20.dp, vertical = 18.dp),
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.Top,
            ) {
                Column {
                    Text("PHOTOS", style = Label)
                    Text("%03d".format(overlay.frameCount), style = ReadoutLarge, color = Ink.Bone)
                }
                Column(horizontalAlignment = Alignment.End) {
                    Text("ANGULAR COVERAGE", style = Label)
                    Text(
                        "${(overlay.coverageFraction * 100).roundToInt()}%",
                        style = ReadoutLarge,
                        color = if (overlay.coverageFraction > 0.6f) Ink.Phosphor else Ink.Amber,
                    )
                }
            }

            Spacer(Modifier.height(12.dp))
            CoverageStrip(overlay)
            Spacer(Modifier.height(14.dp))

            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                StatusChip(
                    text = if (overlay.tracking) "TRACKING" else "TRACKING LOST",
                    tint = if (overlay.tracking) Ink.Phosphor else Ink.Alert,
                )
                if (overlay.subjectPlaced) {
                    StatusChip(
                        text = "%.2f m".format(overlay.distanceMeters),
                        tint = if (overlay.distanceMeters in 0.25f..1.2f) Ink.Phosphor else Ink.Amber,
                    )
                }
                if (overlay.motionBlurRisk) StatusChip(text = "TOO FAST", tint = Ink.Amber)
                if (exposureSupported) {
                    StatusChip(
                        text = if (exposureLocked) "AE/AF LOCKED" else "AE/AF AUTO",
                        tint = if (exposureLocked) Ink.Phosphor else Ink.Steel,
                        onClick = { onToggleExposureLock(!exposureLocked) },
                    )
                }
            }
        }

        Column(
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .fillMaxWidth()
                .padding(horizontal = 20.dp, vertical = 26.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
        ) {
            if (overlay.subjectPlaced) {
                VolumeControl(
                    radius = overlay.subjectRadius,
                    onAdjust = { viewModel.adjustSubjectRadius(it) },
                )
                Spacer(Modifier.height(14.dp))
            }

            Text(
                overlay.lastRejectReason ?: overlay.hint,
                style = Readout,
                color = if (overlay.lastRejectReason != null) Ink.Amber else Ink.Bone,
                textAlign = TextAlign.Center,
                modifier = Modifier.fillMaxWidth(),
            )
            Spacer(Modifier.height(18.dp))

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically,
            ) {
                QualitySelector(quality) { viewModel.setQuality(it) }

                Box(
                    modifier = Modifier
                        .size(76.dp)
                        .clip(CircleShape)
                        .background(Ink.Slate)
                        .border(2.dp, Ink.Phosphor, CircleShape)
                        .clickable { viewModel.capture.requestShutter() },
                    contentAlignment = Alignment.Center,
                ) {
                    Box(
                        modifier = Modifier
                            .size(54.dp)
                            .clip(CircleShape)
                            .background(Ink.Phosphor),
                    )
                }

                Column(horizontalAlignment = Alignment.End) {
                    ActionButton(
                        label = "BUILD",
                        enabled = overlay.frameCount >= CaptureController.MIN_FRAMES,
                        onClick = { viewModel.startReconstruction() },
                    )
                    Spacer(Modifier.height(8.dp))
                    Text(
                        "DISCARD",
                        style = Label,
                        color = Ink.Steel,
                        modifier = Modifier.clickable { viewModel.discardCaptures() },
                    )
                }
            }
        }
    }
}

/** Live AR guide: a ring around the subject that fills in as angles are covered. */
@Composable
private fun OrbitOverlay(overlay: CaptureController.Overlay) {
    val density = LocalDensity.current
    Canvas(modifier = Modifier.fillMaxSize()) {
        // Capture volume first, so the coverage ring reads on top of it.
        val volume = overlay.volumePoints
        if (volume.size >= 6) {
            val samples = CaptureController.VOLUME_SAMPLES
            val stroke = with(density) { 1.2.dp.toPx() }
            for (circle in 0 until 3) {
                for (i in 0 until samples) {
                    val a = (circle * samples + i) * 2
                    val b = (circle * samples + (i + 1) % samples) * 2
                    if (a + 1 >= volume.size || b + 1 >= volume.size) continue
                    val x0 = volume[a]; val y0 = volume[a + 1]
                    val x1 = volume[b]; val y1 = volume[b + 1]
                    if (x0.isNaN() || y0.isNaN() || x1.isNaN() || y1.isNaN()) continue
                    drawLine(
                        color = Ink.Amber.copy(alpha = 0.34f),
                        start = Offset(x0, y0),
                        end = Offset(x1, y1),
                        strokeWidth = stroke,
                    )
                }
            }
        }

        val points = overlay.ringPoints
        val covered = overlay.ringCovered
        if (points.size >= 4 && covered.isNotEmpty()) {
            val strokeCovered = with(density) { 5.dp.toPx() }
            val strokeMissing = with(density) { 2.dp.toPx() }
            val count = covered.size
            for (i in 0 until count) {
                val next = (i + 1) % count
                val x0 = points[i * 2]
                val y0 = points[i * 2 + 1]
                val x1 = points[next * 2]
                val y1 = points[next * 2 + 1]
                if (x0.isNaN() || y0.isNaN() || x1.isNaN() || y1.isNaN()) continue
                val isCovered = covered[i] && covered[next]
                drawLine(
                    color = if (isCovered) Ink.Phosphor else Ink.Steel.copy(alpha = 0.45f),
                    start = Offset(x0, y0),
                    end = Offset(x1, y1),
                    strokeWidth = if (isCovered) strokeCovered else strokeMissing,
                )
            }
        }

        overlay.subjectScreen?.let { screen ->
            val radius = with(density) { 15.dp.toPx() }
            drawCircle(
                color = Ink.Phosphor.copy(alpha = 0.9f),
                radius = radius,
                center = Offset(screen[0], screen[1]),
                style = androidx.compose.ui.graphics.drawscope.Stroke(
                    width = with(density) { 1.5.dp.toPx() },
                ),
            )
            val tick = with(density) { 7.dp.toPx() }
            drawLine(
                Ink.Phosphor, Offset(screen[0] - tick, screen[1]), Offset(screen[0] + tick, screen[1]),
                strokeWidth = with(density) { 1.5.dp.toPx() },
            )
            drawLine(
                Ink.Phosphor, Offset(screen[0], screen[1] - tick), Offset(screen[0], screen[1] + tick),
                strokeWidth = with(density) { 1.5.dp.toPx() },
            )
        }
    }
}

/** Twenty-four ticks, one per azimuth sector, lit when that sector holds a photo. */
@Composable
private fun CoverageStrip(overlay: CaptureController.Overlay) {
    val covered = overlay.ringCovered
    Row(
        modifier = Modifier.fillMaxWidth().height(18.dp),
        horizontalArrangement = Arrangement.spacedBy(3.dp),
    ) {
        val sectors = 24
        for (sector in 0 until sectors) {
            val index = (sector.toFloat() / sectors * covered.size).toInt()
            val lit = covered.isNotEmpty() && covered.getOrElse(index) { false }
            Box(
                modifier = Modifier
                    .weight(1f)
                    .fillMaxSize()
                    .clip(RoundedCornerShape(1.dp))
                    .background(if (lit) Ink.Phosphor else Ink.Edge),
            )
        }
    }
}

/**
 * Sets the radius of the reconstruction volume. Everything outside it is
 * discarded before dense stereo, so this is the single biggest lever the
 * operator has over both runtime and how much detail lands on the object.
 */
@Composable
private fun VolumeControl(radius: Float, onAdjust: (Float) -> Unit) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.Center,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text("CAPTURE VOLUME", style = Label)
        Spacer(Modifier.width(14.dp))
        StepButton("-") { onAdjust(-CaptureController.RADIUS_STEP) }
        Text(
            "  ${(radius * 200).roundToInt()} cm  ",
            style = Readout.copy(fontSize = 13.sp),
            color = Ink.Amber,
        )
        StepButton("+") { onAdjust(CaptureController.RADIUS_STEP) }
    }
}

@Composable
private fun StepButton(label: String, onClick: () -> Unit) {
    Box(
        modifier = Modifier
            .size(34.dp)
            .clip(RoundedCornerShape(3.dp))
            .background(Ink.Slate.copy(alpha = 0.9f))
            .clickable { onClick() },
        contentAlignment = Alignment.Center,
    ) {
        Text(label, style = Readout.copy(fontSize = 16.sp), color = Ink.Bone)
    }
}

@Composable
private fun StatusChip(text: String, tint: Color, onClick: (() -> Unit)? = null) {
    Box(
        modifier = Modifier
            .clip(RoundedCornerShape(3.dp))
            .background(Ink.Slate.copy(alpha = 0.86f))
            .then(if (onClick != null) Modifier.clickable { onClick.invoke() } else Modifier)
            .padding(horizontal = 9.dp, vertical = 5.dp),
    ) {
        Text(text, style = Readout.copy(fontSize = 10.sp), color = tint)
    }
}

@Composable
private fun QualitySelector(current: Quality, onSelect: (Quality) -> Unit) {
    Column(horizontalAlignment = Alignment.Start) {
        Text("QUALITY", style = Label)
        Spacer(Modifier.height(4.dp))
        Column {
            Quality.entries.forEach { option ->
                Text(
                    option.name.uppercase(),
                    style = Readout.copy(fontSize = 11.sp),
                    color = if (option == current) Ink.Phosphor else Ink.Steel,
                    modifier = Modifier
                        .clickable { onSelect(option) }
                        .padding(vertical = 2.dp),
                )
            }
        }
    }
}

@Composable
private fun ActionButton(label: String, enabled: Boolean, onClick: () -> Unit) {
    Box(
        modifier = Modifier
            .width(94.dp)
            .clip(RoundedCornerShape(4.dp))
            .background(if (enabled) Ink.Phosphor else Ink.Edge)
            .clickable(enabled = enabled) { onClick() }
            .padding(vertical = 11.dp),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            label,
            style = Readout.copy(fontSize = 12.sp),
            color = if (enabled) Ink.Graphite else Ink.Steel,
        )
    }
}
