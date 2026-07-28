package com.bulletcam.capture.viewer

import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.GLUtils
import android.opengl.Matrix
import com.bulletcam.capture.ar.GlUtil
import java.io.File
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.cos
import kotlin.math.sin

/**
 * Orbit preview for a finished model. A hand-rolled GLES renderer rather than a
 * scene-graph dependency: the app only ever shows one textured mesh, and this
 * keeps the build free of anything that would want network access.
 */
class ModelRenderer(private val objFile: File) : GLSurfaceView.Renderer {

    @Volatile var yawDegrees = 25f
    @Volatile var pitchDegrees = 15f
    @Volatile var distanceScale = 2.6f
    @Volatile var wireframe = false

    @Volatile var loadFailed = false
        private set

    private var model: ObjModel? = null
    private var program = 0
    private var textureId = 0
    private var hasTexture = false

    private var positionAttribute = 0
    private var normalAttribute = 0
    private var texCoordAttribute = 0
    private var mvpUniform = 0
    private var modelUniform = 0
    private var lightUniform = 0
    private var hasTextureUniform = 0
    private var tintUniform = 0

    private val projection = FloatArray(16)
    private val view = FloatArray(16)
    private val modelMatrix = FloatArray(16)
    private val temp = FloatArray(16)
    private val mvp = FloatArray(16)

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.043f, 0.055f, 0.067f, 1f)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
        program = GlUtil.compileProgram(VERTEX_SHADER, FRAGMENT_SHADER)
        positionAttribute = GLES20.glGetAttribLocation(program, "a_Position")
        normalAttribute = GLES20.glGetAttribLocation(program, "a_Normal")
        texCoordAttribute = GLES20.glGetAttribLocation(program, "a_TexCoord")
        mvpUniform = GLES20.glGetUniformLocation(program, "u_Mvp")
        modelUniform = GLES20.glGetUniformLocation(program, "u_Model")
        lightUniform = GLES20.glGetUniformLocation(program, "u_LightDirection")
        hasTextureUniform = GLES20.glGetUniformLocation(program, "u_HasTexture")
        tintUniform = GLES20.glGetUniformLocation(program, "u_Tint")

        val loaded = ObjModel.load(objFile)
        if (loaded == null) {
            loadFailed = true
            return
        }
        model = loaded

        loaded.texture?.let { bitmap ->
            val ids = IntArray(1)
            GLES20.glGenTextures(1, ids, 0)
            textureId = ids[0]
            GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, textureId)
            GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR_MIPMAP_LINEAR)
            GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR)
            GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE)
            GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE)
            GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, bitmap, 0)
            GLES20.glGenerateMipmap(GLES20.GL_TEXTURE_2D)
            hasTexture = true
            bitmap.recycle()
        }
        GlUtil.checkError("ModelRenderer.create")
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        val aspect = width.toFloat() / height.coerceAtLeast(1).toFloat()
        Matrix.perspectiveM(projection, 0, 42f, aspect, 0.01f, 100f)
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        val current = model ?: return

        val yaw = Math.toRadians(yawDegrees.toDouble()).toFloat()
        val pitch = Math.toRadians(pitchDegrees.coerceIn(-85f, 85f).toDouble()).toFloat()
        val distance = current.radius * distanceScale.coerceIn(1.2f, 8f)
        val eyeX = distance * cos(pitch) * sin(yaw)
        val eyeY = distance * sin(pitch)
        val eyeZ = distance * cos(pitch) * cos(yaw)
        Matrix.setLookAtM(view, 0, eyeX, eyeY, eyeZ, 0f, 0f, 0f, 0f, 1f, 0f)

        // Recentre the model on the origin so the orbit always feels balanced.
        Matrix.setIdentityM(modelMatrix, 0)
        Matrix.translateM(modelMatrix, 0, -current.center[0], -current.center[1], -current.center[2])

        Matrix.multiplyMM(temp, 0, view, 0, modelMatrix, 0)
        Matrix.multiplyMM(mvp, 0, projection, 0, temp, 0)

        GLES20.glUseProgram(program)
        GLES20.glUniformMatrix4fv(mvpUniform, 1, false, mvp, 0)
        GLES20.glUniformMatrix4fv(modelUniform, 1, false, modelMatrix, 0)
        GLES20.glUniform3f(lightUniform, 0.35f, 0.75f, 0.55f)
        GLES20.glUniform1i(hasTextureUniform, if (hasTexture && !wireframe) 1 else 0)
        if (wireframe) {
            GLES20.glUniform3f(tintUniform, 0.31f, 0.82f, 0.77f)
        } else {
            GLES20.glUniform3f(tintUniform, 0.82f, 0.85f, 0.88f)
        }

        if (hasTexture && !wireframe) {
            GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
            GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, textureId)
            GLES20.glUniform1i(GLES20.glGetUniformLocation(program, "u_Texture"), 0)
        }

        current.vertices.position(0)
        current.normals.position(0)
        current.texCoords.position(0)
        GLES20.glVertexAttribPointer(positionAttribute, 3, GLES20.GL_FLOAT, false, 0, current.vertices)
        GLES20.glVertexAttribPointer(normalAttribute, 3, GLES20.GL_FLOAT, false, 0, current.normals)
        GLES20.glVertexAttribPointer(texCoordAttribute, 2, GLES20.GL_FLOAT, false, 0, current.texCoords)
        GLES20.glEnableVertexAttribArray(positionAttribute)
        GLES20.glEnableVertexAttribArray(normalAttribute)
        GLES20.glEnableVertexAttribArray(texCoordAttribute)

        if (wireframe) {
            GLES20.glDrawArrays(GLES20.GL_LINES, 0, current.vertexCount)
        } else {
            GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, current.vertexCount)
        }

        GLES20.glDisableVertexAttribArray(positionAttribute)
        GLES20.glDisableVertexAttribArray(normalAttribute)
        GLES20.glDisableVertexAttribArray(texCoordAttribute)
    }

    private companion object {
        const val VERTEX_SHADER = """
            uniform mat4 u_Mvp;
            uniform mat4 u_Model;
            attribute vec4 a_Position;
            attribute vec3 a_Normal;
            attribute vec2 a_TexCoord;
            varying vec3 v_Normal;
            varying vec2 v_TexCoord;
            void main() {
                gl_Position = u_Mvp * a_Position;
                v_Normal = normalize((u_Model * vec4(a_Normal, 0.0)).xyz);
                v_TexCoord = a_TexCoord;
            }
        """

        const val FRAGMENT_SHADER = """
            precision mediump float;
            uniform sampler2D u_Texture;
            uniform vec3 u_LightDirection;
            uniform vec3 u_Tint;
            uniform int u_HasTexture;
            varying vec3 v_Normal;
            varying vec2 v_TexCoord;
            void main() {
                vec3 normal = normalize(v_Normal);
                float lambert = max(dot(normal, normalize(u_LightDirection)), 0.0);
                // A little ambient from below keeps unlit faces readable.
                float shade = 0.34 + 0.66 * lambert;
                vec3 base = u_HasTexture == 1 ? texture2D(u_Texture, v_TexCoord).rgb : u_Tint;
                gl_FragColor = vec4(base * shade, 1.0);
            }
        """
    }
}
