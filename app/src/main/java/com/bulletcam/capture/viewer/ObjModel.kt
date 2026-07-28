package com.bulletcam.capture.viewer

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import java.io.File
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer

/**
 * Minimal reader for the OBJ files this app writes. It deliberately handles only
 * that dialect - triangles, one material, `v`/`vn`/`vt`/`f` - because the viewer
 * never sees a file from anywhere else.
 */
class ObjModel private constructor(
    val vertices: FloatBuffer,
    val normals: FloatBuffer,
    val texCoords: FloatBuffer,
    val vertexCount: Int,
    val texture: Bitmap?,
    val center: FloatArray,
    val radius: Float,
) {
    companion object {
        fun load(objFile: File): ObjModel? {
            if (!objFile.exists()) return null

            val positions = ArrayList<Float>()
            val normals = ArrayList<Float>()
            val uvs = ArrayList<Float>()
            val outPositions = ArrayList<Float>()
            val outNormals = ArrayList<Float>()
            val outUvs = ArrayList<Float>()

            objFile.bufferedReader().useLines { lines ->
                for (line in lines) {
                    when {
                        line.startsWith("v ") -> line.parseInto(positions, 3)
                        line.startsWith("vn ") -> line.parseInto(normals, 3)
                        line.startsWith("vt ") -> line.parseInto(uvs, 2)
                        line.startsWith("f ") -> {
                            val corners = line.substring(2).trim().split(WHITESPACE)
                            if (corners.size < 3) continue
                            for (i in 1 until corners.size - 1) {
                                appendCorner(corners[0], positions, normals, uvs, outPositions, outNormals, outUvs)
                                appendCorner(corners[i], positions, normals, uvs, outPositions, outNormals, outUvs)
                                appendCorner(corners[i + 1], positions, normals, uvs, outPositions, outNormals, outUvs)
                            }
                        }
                    }
                }
            }

            val vertexCount = outPositions.size / 3
            if (vertexCount == 0) return null

            var minX = Float.MAX_VALUE; var minY = Float.MAX_VALUE; var minZ = Float.MAX_VALUE
            var maxX = -Float.MAX_VALUE; var maxY = -Float.MAX_VALUE; var maxZ = -Float.MAX_VALUE
            for (i in 0 until vertexCount) {
                val x = outPositions[i * 3]; val y = outPositions[i * 3 + 1]; val z = outPositions[i * 3 + 2]
                if (x < minX) minX = x; if (x > maxX) maxX = x
                if (y < minY) minY = y; if (y > maxY) maxY = y
                if (z < minZ) minZ = z; if (z > maxZ) maxZ = z
            }
            val center = floatArrayOf((minX + maxX) / 2f, (minY + maxY) / 2f, (minZ + maxZ) / 2f)
            val radius = maxOf(maxX - minX, maxY - minY, maxZ - minZ) * 0.5f

            val texture = File(objFile.parentFile, objFile.nameWithoutExtension + ".png")
                .takeIf { it.exists() }
                ?.let { BitmapFactory.decodeFile(it.absolutePath) }

            return ObjModel(
                vertices = outPositions.toBuffer(),
                normals = if (outNormals.size == outPositions.size) outNormals.toBuffer() else outPositions.toBuffer(),
                texCoords = outUvs.toBuffer(),
                vertexCount = vertexCount,
                texture = texture,
                center = center,
                radius = if (radius > 1e-4f) radius else 1f,
            )
        }

        private val WHITESPACE = Regex("\\s+")

        private fun String.parseInto(target: ArrayList<Float>, count: Int) {
            val parts = trim().split(WHITESPACE)
            for (i in 1..count) target.add(parts.getOrNull(i)?.toFloatOrNull() ?: 0f)
        }

        private fun appendCorner(
            corner: String,
            positions: List<Float>,
            normals: List<Float>,
            uvs: List<Float>,
            outPositions: ArrayList<Float>,
            outNormals: ArrayList<Float>,
            outUvs: ArrayList<Float>,
        ) {
            val fields = corner.split('/')
            val positionIndex = fields.getOrNull(0)?.toIntOrNull() ?: return
            val uvIndex = fields.getOrNull(1)?.toIntOrNull()
            val normalIndex = fields.getOrNull(2)?.toIntOrNull()

            val p = (positionIndex - 1) * 3
            if (p + 2 < positions.size) {
                outPositions.add(positions[p]); outPositions.add(positions[p + 1]); outPositions.add(positions[p + 2])
            } else {
                outPositions.add(0f); outPositions.add(0f); outPositions.add(0f)
            }

            val n = ((normalIndex ?: 0) - 1) * 3
            if (normalIndex != null && n + 2 < normals.size) {
                outNormals.add(normals[n]); outNormals.add(normals[n + 1]); outNormals.add(normals[n + 2])
            } else {
                outNormals.add(0f); outNormals.add(1f); outNormals.add(0f)
            }

            val t = ((uvIndex ?: 0) - 1) * 2
            if (uvIndex != null && t + 1 < uvs.size) {
                // OBJ stores V from the bottom; GL samples from the top.
                outUvs.add(uvs[t]); outUvs.add(1f - uvs[t + 1])
            } else {
                outUvs.add(0f); outUvs.add(0f)
            }
        }

        private fun ArrayList<Float>.toBuffer(): FloatBuffer =
            ByteBuffer.allocateDirect(size * 4).order(ByteOrder.nativeOrder()).asFloatBuffer().apply {
                for (value in this@toBuffer) put(value)
                position(0)
            }
    }
}
