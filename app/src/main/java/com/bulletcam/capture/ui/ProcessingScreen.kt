package com.bulletcam.capture.ui

import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlin.math.roundToInt

private val Stages = listOf(
    "Aligning captures",
    "Selecting views",
    "Dense stereo matching",
    "Fusing depth maps",
    "Cleaning point cloud",
    "Building surface",
    "Unwrapping UVs",
    "Baking texture",
    "Writing model",
)

@Composable
fun ProcessingScreen(stage: String, fraction: Float, onCancel: () -> Unit) {
    val animated by animateFloatAsState(targetValue = fraction, label = "progress")

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(Ink.Graphite)
            .padding(28.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center,
    ) {
        Text("RECONSTRUCTING", style = Label)
        Spacer(Modifier.height(28.dp))

        Box(contentAlignment = Alignment.Center) {
            Canvas(modifier = Modifier.size(196.dp)) {
                val stroke = 8.dp.toPx()
                val inset = stroke / 2f
                val arcSize = Size(size.width - stroke, size.height - stroke)
                drawArc(
                    color = Ink.Edge,
                    startAngle = -90f,
                    sweepAngle = 360f,
                    useCenter = false,
                    topLeft = androidx.compose.ui.geometry.Offset(inset, inset),
                    size = arcSize,
                    style = Stroke(width = stroke),
                )
                drawArc(
                    color = Ink.Phosphor,
                    startAngle = -90f,
                    sweepAngle = 360f * animated.coerceIn(0f, 1f),
                    useCenter = false,
                    topLeft = androidx.compose.ui.geometry.Offset(inset, inset),
                    size = arcSize,
                    style = Stroke(width = stroke),
                )
            }
            Text("${(animated * 100).roundToInt()}%", style = ReadoutLarge.copy(fontSize = 40.sp))
        }

        Spacer(Modifier.height(30.dp))
        Text(stage, style = Readout, color = Ink.Bone, textAlign = TextAlign.Center)
        Spacer(Modifier.height(22.dp))

        Column(modifier = Modifier.fillMaxWidth()) {
            val activeIndex = Stages.indexOfFirst { it.equals(stage, ignoreCase = true) }
            Stages.forEachIndexed { index, name ->
                val done = activeIndex >= 0 && index < activeIndex
                val active = index == activeIndex
                Text(
                    text = (if (done) "  done  " else if (active) "  >>>>  " else "  ....  ") + name,
                    style = Readout.copy(fontSize = 11.sp),
                    color = when {
                        active -> Ink.Phosphor
                        done -> Ink.Steel
                        else -> Ink.Edge
                    },
                    modifier = Modifier.padding(vertical = 2.dp),
                )
            }
        }

        Spacer(Modifier.height(34.dp))
        Text(
            "CANCEL",
            style = Readout.copy(fontSize = 12.sp),
            color = Ink.Steel,
            modifier = Modifier.clickable { onCancel() }.padding(10.dp),
        )
        Spacer(Modifier.height(8.dp))
        Text(
            "Everything runs on this device. Keep the screen on; the phone will get warm.",
            style = Readout.copy(fontSize = 10.sp),
            color = Ink.Edge,
            textAlign = TextAlign.Center,
        )
    }
}
