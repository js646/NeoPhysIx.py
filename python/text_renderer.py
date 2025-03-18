import pygame
from OpenGL.GL import *
import OpenGL.GL.shaders as shaders
import numpy as np
import ctypes

class TextRenderer:
    def __init__(self, window_width, window_height):
        self.window_width = window_width
        self.window_height = window_height
        self.vao = glGenVertexArrays(1)
        glBindVertexArray(self.vao)
        self.vbo = glGenBuffers(1)
        self.setup_quad()
        self.shader = self.create_shader()
        glUseProgram(self.shader)  # Aktivieren Sie das Shader-Programm hier
        self.font = pygame.font.SysFont('Arial', 18)

    def create_shader(self):
        vertex_shader = """
        #version 330 core
        layout (location = 0) in vec4 vertex;
        out vec2 TexCoords;
        uniform mat4 projection;
        void main()
        {
            gl_Position = projection * vec4(vertex.xy, 0.0, 1.0);
            TexCoords = vertex.zw;
        }
        """

        fragment_shader = """
        #version 330 core
        in vec2 TexCoords;
        out vec4 color;
        uniform sampler2D text;
        void main()
        {
            color = texture(text, TexCoords);
        }
        """

        # Stelle sicher, dass das VAO vor der Shader-Validierung gebunden ist
        glBindVertexArray(self.vao)

        shader = shaders.compileProgram(
            shaders.compileShader(vertex_shader, GL_VERTEX_SHADER),
            shaders.compileShader(fragment_shader, GL_FRAGMENT_SHADER)
        )

        glValidateProgram(shader)
        if glGetProgramiv(shader, GL_VALIDATE_STATUS) == GL_FALSE:
            print("Shader validation error:", glGetProgramInfoLog(shader))

        glUseProgram(0)
        glBindVertexArray(0)  # VAO wieder entbinden, nachdem der Shader kompiliert und validiert wurde
        return shader


    def setup_quad(self):
        glBindVertexArray(self.vao)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        glBufferData(GL_ARRAY_BUFFER, 24 * 4, None, GL_DYNAMIC_DRAW)
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, ctypes.c_void_p(0))
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glBindVertexArray(0)

    def render_text(self, lines, x, y, scale, color):
        glUseProgram(self.shader)
        glBindVertexArray(self.vao)

        projection = np.eye(4, dtype=np.float32)
        glUniformMatrix4fv(glGetUniformLocation(self.shader, "projection"), 1, GL_FALSE, projection)

        glViewport(0, 0, self.window_width, self.window_height)

        glActiveTexture(GL_TEXTURE0)
        glBindVertexArray(self.vao)

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        line_height = self.font.get_linesize() * scale

        for i, line in enumerate(lines):
            text_surface = self.font.render(line, True, (int(color[0] * 255), int(color[1] * 255), int(color[2] * 255)))
            text_data = pygame.image.tostring(text_surface, "RGBA", True)

            texture = glGenTextures(1)
            glBindTexture(GL_TEXTURE_2D, texture)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, text_surface.get_width(), text_surface.get_height(), 0, GL_RGBA,
                         GL_UNSIGNED_BYTE, text_data)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

            x_pos, y_pos = -1 + (2 * x / self.window_width), 1 - (2 * (y + i * line_height) / self.window_height)
            w, h = (2 * text_surface.get_width() * scale / self.window_width), (
                    2 * text_surface.get_height() * scale / self.window_height)

            vertices = np.array([
                x_pos, y_pos, 0.0, 1.0,
                x_pos + w, y_pos, 1.0, 1.0,
                x_pos, y_pos - h, 0.0, 0.0,
                x_pos + w, y_pos - h, 1.0, 0.0
            ], dtype=np.float32)

            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
            glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.nbytes, vertices)
            glBindBuffer(GL_ARRAY_BUFFER, 0)

            glBindTexture(GL_TEXTURE_2D, texture)
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4)

            glDeleteTextures(1, [texture])

        glBindVertexArray(0)
        glBindTexture(GL_TEXTURE_2D, 0)
        glUseProgram(0)

        self.check_gl_error()

    def check_gl_error(self):
        err = glGetError()
        if err != GL_NO_ERROR:
            print(f"OpenGL error: {err}")
