package com.bulletcam.capture.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

@Composable
fun FailureScreen(reason: String, onRetry: () -> Unit, onStartOver: () -> Unit) {
    Column(
        modifier = Modifier.fillMaxSize().background(Ink.Graphite).padding(30.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center,
    ) {
        Text("RECONSTRUCTION STOPPED", style = Label.copy(color = Ink.Amber))
        Spacer(Modifier.height(16.dp))
        Text(reason, style = Readout, color = Ink.Bone, textAlign = TextAlign.Center)
        Spacer(Modifier.height(34.dp))
        PlainButton("KEEP PHOTOS AND RETRY", onRetry)
        Spacer(Modifier.height(10.dp))
        PlainButton("START OVER", onStartOver)
    }
}

@Composable
fun PermissionScreen(rationale: String, onRequest: () -> Unit) {
    Column(
        modifier = Modifier.fillMaxSize().background(Ink.Graphite).padding(30.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center,
    ) {
        Text("BULLETCAM", style = ReadoutLarge, color = Ink.Phosphor)
        Spacer(Modifier.height(10.dp))
        Text("OFFLINE OBJECT PHOTOGRAMMETRY", style = Label)
        Spacer(Modifier.height(28.dp))
        Text(rationale, style = Readout, color = Ink.Bone, textAlign = TextAlign.Center)
        Spacer(Modifier.height(28.dp))
        PlainButton("GRANT CAMERA ACCESS", onRequest)
    }
}

@Composable
fun PlainButton(label: String, onClick: () -> Unit) {
    Box(
        modifier = Modifier
            .clip(RoundedCornerShape(4.dp))
            .background(Ink.Slate)
            .clickable { onClick() }
            .padding(horizontal = 22.dp, vertical = 13.dp),
        contentAlignment = Alignment.Center,
    ) {
        Text(label, style = Readout.copy(fontSize = 12.sp), color = Ink.Phosphor)
    }
}
