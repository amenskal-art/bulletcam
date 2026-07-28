package com.bulletcam.capture.capture

import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Tracks which directions around the subject have been photographed.
 *
 * The viewing sphere is divided into azimuth sectors crossed with three
 * elevation bands. A reconstruction needs even angular coverage far more than
 * it needs many photos, so this is what the capture UI reports instead of a
 * frame count.
 */
class OrbitCoverage(
    private val azimuthSectors: Int = 24,
    private val elevationBands: Int = 3,
) {
    /** Elevation band edges in degrees, low to high. */
    private val bandEdges = floatArrayOf(-90f, 12f, 42f, 90f)

    private val hits = IntArray(azimuthSectors * elevationBands)
    private val directions = ArrayList<FloatArray>()

    val sectorCount: Int get() = azimuthSectors

    fun reset() {
        hits.fill(0)
        directions.clear()
    }

    /** @param direction unit vector from the subject towards the camera. */
    fun record(direction: FloatArray) {
        hits[binOf(direction)]++
        directions.add(direction.copyOf())
    }

    fun isAzimuthCovered(sector: Int): Boolean {
        for (band in 0 until elevationBands) {
            if (hits[band * azimuthSectors + sector] > 0) return true
        }
        return false
    }

    /** Fraction of reachable bins that hold at least one photo. */
    fun coverage(): Float {
        val filled = hits.count { it > 0 }
        return filled.toFloat() / hits.size.toFloat()
    }

    fun azimuthCoverage(): Float {
        var covered = 0
        for (sector in 0 until azimuthSectors) if (isAzimuthCovered(sector)) covered++
        return covered.toFloat() / azimuthSectors.toFloat()
    }

    /** Smallest angle, in degrees, between [direction] and any recorded view. */
    fun angleToNearestCapture(direction: FloatArray): Float {
        if (directions.isEmpty()) return 180f
        var best = -1f
        for (existing in directions) {
            val dot = existing[0] * direction[0] + existing[1] * direction[1] +
                existing[2] * direction[2]
            if (dot > best) best = dot
        }
        return Math.toDegrees(acos(best.coerceIn(-1f, 1f)).toDouble()).toFloat()
    }

    /** Human-readable hint about the largest hole in the orbit. */
    fun nextHint(): String {
        val missing = (0 until azimuthSectors).filterNot { isAzimuthCovered(it) }
        if (missing.isEmpty()) {
            val lowBand = (0 until azimuthSectors).count { hits[it] > 0 }
            val highBand = (0 until azimuthSectors).count { hits[2 * azimuthSectors + it] > 0 }
            return when {
                lowBand < azimuthSectors / 2 -> "Drop lower and orbit again"
                highBand < azimuthSectors / 2 -> "Rise higher and orbit again"
                else -> "Coverage looks good"
            }
        }
        return "${missing.size} of $azimuthSectors sectors still missing"
    }

    private fun binOf(direction: FloatArray): Int {
        val azimuth = atan2(direction[2], direction[0])
        val normalised = ((azimuth / (2f * Math.PI.toFloat())) + 1f) % 1f
        val sector = floor(normalised * azimuthSectors).toInt()
            .coerceIn(0, azimuthSectors - 1)
        val horizontal = sqrt(direction[0] * direction[0] + direction[2] * direction[2])
        val elevation = Math.toDegrees(atan2(direction[1], horizontal).toDouble()).toFloat()
        var band = 0
        for (i in 0 until elevationBands) {
            if (elevation >= bandEdges[i] && elevation < bandEdges[i + 1]) {
                band = i
                break
            }
            if (i == elevationBands - 1) band = elevationBands - 1
        }
        return band * azimuthSectors + sector
    }

    /** Ring of unit directions at [elevationDeg], for drawing the AR guide. */
    fun ringDirections(elevationDeg: Float, samples: Int): Array<FloatArray> {
        val elevation = Math.toRadians(elevationDeg.toDouble()).toFloat()
        val y = sin(elevation)
        val r = cos(elevation)
        return Array(samples) { i ->
            val a = 2f * Math.PI.toFloat() * i / samples
            floatArrayOf(r * cos(a), y, r * sin(a))
        }
    }

    /** Which azimuth sector a ring sample belongs to. */
    fun sectorOfRingSample(index: Int, samples: Int): Int =
        (index.toFloat() / samples * azimuthSectors).toInt().coerceIn(0, azimuthSectors - 1)

    companion object {
        fun angleBetween(a: FloatArray, b: FloatArray): Float {
            val dot = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]).coerceIn(-1f, 1f)
            return Math.toDegrees(acos(dot).toDouble()).toFloat()
        }

        fun normalise(v: FloatArray): FloatArray {
            val n = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
            return if (n < 1e-6f) floatArrayOf(0f, 0f, 1f) else
                floatArrayOf(v[0] / n, v[1] / n, v[2] / n)
        }

        fun distance(a: FloatArray, b: FloatArray): Float {
            val dx = a[0] - b[0]
            val dy = a[1] - b[1]
            val dz = a[2] - b[2]
            return sqrt(dx * dx + dy * dy + dz * dz)
        }

        fun approximatelyEqual(a: Float, b: Float, tolerance: Float): Boolean =
            abs(a - b) <= tolerance
    }
}
