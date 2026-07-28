package com.bulletcam.capture.ui

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Typography
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp

/**
 * The palette comes from survey and metrology instruments: a graphite body, a
 * phosphor-green readout for what has been measured and amber for what has not.
 * Everything is a system font - the app ships no network permission, so nothing
 * may be fetched at runtime.
 */
object Ink {
    val Graphite = Color(0xFF0B0E11)
    val Slate = Color(0xFF161B21)
    val Edge = Color(0xFF232B34)
    val Phosphor = Color(0xFF4FD1C5)
    val Amber = Color(0xFFF6AD55)
    val Bone = Color(0xFFE2E8F0)
    val Steel = Color(0xFF7C8B9C)
    val Alert = Color(0xFFFC8181)
}

/** Instrument read-outs use monospace so digits never shift as they update. */
val Readout = TextStyle(
    fontFamily = FontFamily.Monospace,
    fontWeight = FontWeight.Medium,
    fontSize = 13.sp,
    letterSpacing = 1.2.sp,
)

val ReadoutLarge = Readout.copy(fontSize = 30.sp, letterSpacing = 2.sp, fontWeight = FontWeight.Light)
val Label = Readout.copy(fontSize = 10.sp, letterSpacing = 2.4.sp, color = Ink.Steel)

private val Scheme = darkColorScheme(
    primary = Ink.Phosphor,
    onPrimary = Ink.Graphite,
    secondary = Ink.Amber,
    background = Ink.Graphite,
    onBackground = Ink.Bone,
    surface = Ink.Slate,
    onSurface = Ink.Bone,
    error = Ink.Alert,
)

@Composable
fun BulletCamTheme(content: @Composable () -> Unit) {
    // The instrument look is dark by design; the system setting is read only so
    // that a future light variant has somewhere to hook in.
    @Suppress("UNUSED_EXPRESSION")
    isSystemInDarkTheme()
    MaterialTheme(
        colorScheme = Scheme,
        typography = Typography(
            bodyMedium = TextStyle(fontSize = 14.sp, color = Ink.Bone),
            bodySmall = TextStyle(fontSize = 12.sp, color = Ink.Steel),
            titleMedium = Readout.copy(fontSize = 16.sp, color = Ink.Bone),
        ),
        content = content,
    )
}
