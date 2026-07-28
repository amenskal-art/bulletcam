package com.bulletcam.capture.ui

import android.content.Context
import android.content.Intent
import android.opengl.GLSurfaceView
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.FileProvider
import com.bulletcam.capture.viewer.ModelRenderer
import java.io.File

@Composable
fun ViewerScreen(
    objFile: File,
    glbFile: File,
    summary: String,
    onNewScan: () -> Unit,
    onBack: () -> Unit,
) {
    val context = LocalContext.current
    val renderer = remember(objFile.absolutePath) { ModelRenderer(objFile) }
    var wireframe by remember { mutableStateOf(false) }

    val glView = remember(objFile.absolutePath) {
        GLSurfaceView(context).apply {
            setEGLContextClientVersion(2)
            setRenderer(renderer)
            renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
        }
    }

    DisposableEffect(glView) {
        glView.onResume()
        onDispose { glView.onPause() }
    }

    Box(modifier = Modifier.fillMaxSize().background(Ink.Graphite)) {
        AndroidView(
            modifier = Modifier
                .fillMaxSize()
                .pointerInput(Unit) {
                    // One recogniser for both gestures: a drag orbits the camera
                    // and a pinch changes the distance.
                    detectTransformGestures { _, pan, zoom, _ ->
                        renderer.yawDegrees -= pan.x * 0.3f
                        renderer.pitchDegrees =
                            (renderer.pitchDegrees + pan.y * 0.3f).coerceIn(-85f, 85f)
                        if (zoom != 1f) {
                            renderer.distanceScale =
                                (renderer.distanceScale / zoom).coerceIn(1.2f, 8f)
                        }
                    }
                },
            factory = { glView },
        )

        Column(
            modifier = Modifier
                .align(Alignment.TopStart)
                .padding(20.dp),
        ) {
            Text("MODEL READY", style = Label)
            Spacer(Modifier.height(4.dp))
            Text(summary, style = Readout, color = Ink.Bone)
            Spacer(Modifier.height(4.dp))
            Text(
                "${objFile.parentFile?.name ?: ""}  ·  ${(glbFile.length() / 1024)} KB",
                style = Readout.copy(fontSize = 10.sp),
                color = Ink.Steel,
            )
        }

        Column(
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .fillMaxWidth()
                .padding(20.dp),
        ) {
            Text(
                "Drag to orbit · pinch to zoom",
                style = Readout.copy(fontSize = 10.sp),
                color = Ink.Steel,
                modifier = Modifier.align(Alignment.CenterHorizontally),
            )
            Spacer(Modifier.height(14.dp))
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(10.dp),
            ) {
                ViewerButton("SHADED", !wireframe, Modifier.weight(1f)) {
                    wireframe = false
                    renderer.wireframe = false
                }
                ViewerButton("WIRE", wireframe, Modifier.weight(1f)) {
                    wireframe = true
                    renderer.wireframe = true
                }
                ViewerButton("SHARE", false, Modifier.weight(1f)) {
                    shareModel(context, glbFile.takeIf { it.exists() } ?: objFile)
                }
            }
            Spacer(Modifier.height(10.dp))
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(10.dp),
            ) {
                ViewerButton("BACK", false, Modifier.weight(1f)) { onBack() }
                ViewerButton("NEW SCAN", true, Modifier.weight(1f)) { onNewScan() }
            }
        }
    }
}

@Composable
private fun ViewerButton(
    label: String,
    highlighted: Boolean,
    modifier: Modifier = Modifier,
    onClick: () -> Unit,
) {
    Box(
        modifier = modifier
            .clip(RoundedCornerShape(4.dp))
            .background(if (highlighted) Ink.Phosphor else Ink.Slate)
            .clickable { onClick() }
            .padding(vertical = 12.dp),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            label,
            style = Readout.copy(fontSize = 11.sp),
            color = if (highlighted) Ink.Graphite else Ink.Bone,
        )
    }
}

private fun shareModel(context: Context, file: File) {
    val uri = FileProvider.getUriForFile(context, "${context.packageName}.fileprovider", file)
    val intent = Intent(Intent.ACTION_SEND).apply {
        type = if (file.extension == "glb") "model/gltf-binary" else "application/octet-stream"
        putExtra(Intent.EXTRA_STREAM, uri)
        addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
    }
    context.startActivity(Intent.createChooser(intent, "Share model"))
}
