import numpy as np
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *

from custom_math import Vector3D, cross_product


# -- static methods --

def check_opengl_error():
    err = glGetError()
    if err != GL_NO_ERROR:
        print(f"OpenGL error: {err}")


def check_opengl_version():
    version = glGetString(GL_VERSION)
    if version is not None:
        version = version.decode("utf-8")
        print(f"OpenGL Version: {version}")
        major_version = int(version.split(".")[0])
        if major_version < 2:
            raise RuntimeError("OpenGL version 2.0 or higher is required.")
    else:
        raise RuntimeError("Unable to determine OpenGL version.")


def compile_shader(source, shader_type):
    shader = glCreateShader(shader_type)
    glShaderSource(shader, source)
    glCompileShader(shader)

    if glGetShaderiv(shader, GL_COMPILE_STATUS) != GL_TRUE:
        raise RuntimeError(glGetShaderInfoLog(shader))

    return shader


def create_shader_program(vertex_source, fragment_source):
    vertex_shader = compile_shader(vertex_source, GL_VERTEX_SHADER)
    fragment_shader = compile_shader(fragment_source, GL_FRAGMENT_SHADER)

    program = glCreateProgram()
    glAttachShader(program, vertex_shader)
    glAttachShader(program, fragment_shader)
    glLinkProgram(program)

    if glGetProgramiv(program, GL_LINK_STATUS) != GL_TRUE:
        raise RuntimeError(glGetProgramInfoLog(program))

    glDeleteShader(vertex_shader)
    glDeleteShader(fragment_shader)

    return program

# ----


class Graphics:
    def __init__(self):
        # Pygame window settings
        self.window = None
        self.window_title = "NeoPhysIx.py"
        self.window_width = 800  # px
        self.window_height = 600  # px
        self.window_size = (self.window_width, self.window_height)
        self.aspect_ratio = self.window_width / self.window_height

        # Init pygame window with OPENGL/DOUBLEBUF mode
        self.window = pygame.display.set_mode(self.window_size, pygame.OPENGL | pygame.DOUBLEBUF)
        pygame.display.set_caption(self.window_title)  # sets the window title

        # Perspective
        self.fov = 45.0  # field of view
        self.near_plane = 0.1  # distance from camera to near clipping plane
        self.far_plane = 100.0  # distance from camera to far clipping plane

        self.y_rotate, self.x_rotate = 0.0, 0.0  # rotation angels
        self.x_move, self.y_move, self.z_move = 0.0, 0.0, 0.0  # perspective offsets

        # Mouse actions
        self.pressed_mouse_buttons = [False, False, False]  # currently pressed mouse buttons [left, mid, right]
        self.x_old, self.y_old = -1.0, -1.0  # used to save the old mouse position coordinates

        # Create additional Pygame window for text output
        self.text_window = pygame.Surface((self.window_width, self.window_height))
        self.font = pygame.font.SysFont('Arial', 18)

        # Placeholder
        self.landscape_vertices = None
        self.landscape_colors = None

        # --- OpenGL ---
        self.background_color = (0.1, 0.2, 0.2, 1)  # RGBA
        self.camera_starting_point = (0, -1, -12)  # XYZ

        # Set background color and enable depth testing
        glClearColor(self.background_color[0], self.background_color[1], self.background_color[2], self.background_color[3])
        glEnable(GL_DEPTH_TEST)

        # Rendering objects
        self.vao = None
        self.vbo = None

        # Create shader program with vertex_shader and fragment_shader files
        with open("shader/vertex_shader.glsl", "r") as f:
            vertex_shader_source = f.read()
        with open("shader/fragment_shader.glsl", "r") as f:
            fragment_shader_source = f.read()
        self.shader = create_shader_program(vertex_shader_source, fragment_shader_source)

        # Init transformation matrices used in shader programm
        self.init_shader_matrices()

    def init_shader_matrices(self):
        # -- Initialize the transformation matrices used in the shader program --
        glUseProgram(self.shader)

        # Set starting perspective (modifies projection matrix of OpenGL context)
        glMatrixMode(GL_PROJECTION)  # change matrix mode to modify the projection matrix of the OpenGL context
        gluPerspective(self.fov, self.aspect_ratio, self.near_plane, self.far_plane)
        glTranslatef(self.camera_starting_point[0], self.camera_starting_point[1], self.camera_starting_point[2])
        glMatrixMode(GL_MODELVIEW)  # change matrix mode back to modelview

        # Get uniform locations for matrices used in shader program (memory locations in GPU)
        model_loc = glGetUniformLocation(self.shader, "model")
        view_loc = glGetUniformLocation(self.shader, "view")
        projection_loc = glGetUniformLocation(self.shader, "projection")

        # Init matrices
        projection_matrix = np.identity(4, dtype=np.float32)  # placeholder for projection matrix from OpenGL context
        glGetFloatv(GL_PROJECTION_MATRIX, projection_matrix)  # get current projection matrix from OpenGL context
        view_matrix = np.identity(4, dtype=np.float32)  # init with identity matrix
        model_matrix = np.identity(4, dtype=np.float32)  # init with identity matrix

        # Transfer matrices into shader program memory
        glUniformMatrix4fv(projection_loc, 1, GL_FALSE, projection_matrix)
        glUniformMatrix4fv(view_loc, 1, GL_FALSE, view_matrix)
        glUniformMatrix4fv(model_loc, 1, GL_FALSE, model_matrix)

        glUseProgram(0)
        # -----

        # Check OpenGL version and error
        check_opengl_version()
        check_opengl_error()

    def update_shader_view_matrix(self):
        glUseProgram(self.shader)

        view_matrix = np.identity(4, dtype=np.float32)
        view_loc = glGetUniformLocation(self.shader, "view")

        glGetFloatv(GL_MODELVIEW_MATRIX, view_matrix)  # get current view matrix from OpenGL context
        glUniformMatrix4fv(view_loc, 1, GL_FALSE, view_matrix)  # update view matrix of shader

        glUseProgram(0)

    def update_shader_projection_matrix(self):
        glUseProgram(self.shader)

        projection_matrix = np.identity(4, dtype=np.float32)
        projection_loc = glGetUniformLocation(self.shader, "projection")

        glGetFloatv(GL_PROJECTION_MATRIX, projection_matrix)  # get current projection matrix from OpenGL context
        glUniformMatrix4fv(projection_loc, 1, GL_FALSE, projection_matrix)  # update projection matrix of shader

        glUseProgram(0)

    def update_screen(self):
        pygame.display.flip()

    def draw_scene(self, landscape, entities, max_world):
        # refresh screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Use the shader
        glUseProgram(self.shader)

        # draw environment and objects
        self.draw_landscape(landscape, max_world)
        self.draw_entities(entities)

        # check for errors
        check_opengl_error()

        # Reset the shader program
        glUseProgram(0)

    def draw_landscape(self, landscape, max_world_size):
        if self.landscape_vertices and self.landscape_colors is not None:
            self.render_data(self.landscape_vertices, self.landscape_colors)
            return

        vertices = []
        colors = []

        for x in range(max_world_size):
            for z in range(max_world_size):
                if landscape[x][z] < 0:
                    landscape[x][z] = 0

        for xx in range(max_world_size):
            for zz in range(max_world_size):
                yy = landscape[xx][zz]
                yy1 = landscape[xx + 1][zz]
                yy2 = landscape[xx][zz + 1]
                yy3 = landscape[xx + 1][zz + 1]

                diff = (landscape[xx][zz] - landscape[xx + 1][zz + 1]) / 20.0
                if diff > 1.0:
                    diff = 1.0
                if diff < -1.0:
                    diff = -1.0

                if landscape[xx][zz] == 0:
                    color = (0.75, 0.75, 0.75)
                else:
                    color = (0.75 + diff, 0.75 + diff, 0.75 + diff / 2.0)

                vertices.extend([
                    (xx - max_world_size / 2.0) / 10.0, yy / 10.0, (zz - max_world_size / 2.0) / 10.0,
                    (xx + 1 - max_world_size / 2.0) / 10.0, yy1 / 10.0, (zz - max_world_size / 2.0) / 10.0,
                    (xx - max_world_size / 2.0) / 10.0, yy2 / 10.0, (zz + 1 - max_world_size / 2.0) / 10.0,
                    (xx + 1 - max_world_size / 2.0) / 10.0, yy3 / 10.0, (zz + 1 - max_world_size / 2.0) / 10.0,
                    (xx + 1 - max_world_size / 2.0) / 10.0, yy1 / 10.0, (zz - max_world_size / 2.0) / 10.0,
                    (xx - max_world_size / 2.0) / 10.0, yy2 / 10.0, (zz + 1 - max_world_size / 2.0) / 10.0,
                ])
                colors.extend([color for _ in range(6)])

        self.landscape_vertices = vertices
        self.landscape_colors = colors
        self.render_data(vertices, colors)

    def draw_entities(self, entities):
        vertices = []
        colors = []

        for obj in entities:
            massPoints = obj.get_massPoints()
            for triMesh in obj.get_triMesh():
                # height difference between two points
                height = massPoints[triMesh.pointNum[0]].y - massPoints[triMesh.pointNum[2]].y

                # modify colors based on height difference
                triMesh.color[0] += height
                triMesh.color[1] += height
                triMesh.color[2] += height

                if triMesh.pointNum[0] != triMesh.pointNum[1] and \
                        triMesh.pointNum[0] != triMesh.pointNum[2]:

                    aa = triMesh.pointNum[0]
                    a = Vector3D(massPoints[aa].x, massPoints[aa].y,
                                 massPoints[aa].z)

                    bb = triMesh.pointNum[1]
                    b = Vector3D(massPoints[bb].x, massPoints[bb].y,
                                 massPoints[bb].z)

                    cc = triMesh.pointNum[2]
                    c = Vector3D(massPoints[cc].x, massPoints[cc].y,
                                 massPoints[cc].z)

                    # Calculate normal vector
                    d = Vector3D(a.x - b.x, a.y - b.y, a.z - b.z)
                    e = Vector3D(a.x - c.x, a.y - c.y, a.z - c.z)
                    normal = cross_product(d, e)

                    # Normalize the normal vector
                    norm = np.sqrt(normal.x ** 2 + normal.y ** 2 + normal.z ** 2)
                    normal.x /= norm
                    normal.y /= norm
                    normal.z /= norm

                    # Check if the normal vector points to the camera
                    view = Vector3D(0, 1, 1.0)
                    v_angle = view.x * normal.x + view.y * normal.y + view.z * normal.z

                    if v_angle < 0:
                        normal.x *= -1
                        normal.y *= -1
                        normal.z *= -1

                    # Lighting calculations
                    light = Vector3D(0.3, 1.0, 0.3)
                    brightness = (normal.x * light.x + normal.y * light.y + normal.z * light.z) / 10.0
                    color = [triMesh.color[0] + brightness, triMesh.color[1] + brightness,
                             triMesh.color[2] + brightness]

                    # Collect vertex and color data
                    vertices.extend([a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z])
                    colors.extend(color * 3)

                else:
                    # Single mass point
                    b = triMesh.pointNum[0]
                    a1 = massPoints[b].x
                    a2 = massPoints[b].y
                    a3 = massPoints[b].z

                    vertices.extend([a1 + 0.01, a2, a3,
                                     a1, a2 + 0.01, a3,
                                     a1, a2, a3 + 0.01])
                    colors.extend([triMesh.color[0], triMesh.color[1], triMesh.color[2]] * 3)

        self.render_data(vertices, colors)

    def render_data(self, vertices, colors):
        # transform to np.array
        vertices = np.array(vertices, dtype=np.float32)
        colors = np.array(colors, dtype=np.float32)
        vertex_count = len(vertices)

        # generate new vao & vbo
        self.vao = glGenVertexArrays(1)
        self.vbo = glGenBuffers(2)  # one buffer for vertices, one for colors

        # bind vao
        glBindVertexArray(self.vao)

        # bind vertices to vbo
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo[0])
        glBufferData(GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL_STATIC_DRAW)
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 12, ctypes.c_void_p(0))

        # bind colors to vbo
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo[1])
        glBufferData(GL_ARRAY_BUFFER, colors.nbytes, colors, GL_STATIC_DRAW)
        glEnableVertexAttribArray(1)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 12, ctypes.c_void_p(0))

        # draw bound data
        glDrawArrays(GL_TRIANGLES, 0, vertex_count)

        # check for errors and unbind vertex array
        check_opengl_error()
        glBindVertexArray(0)

    def draw_text(self, cpu_time, gfx_step, counter, run_time, pos):
        lines = [
            f"CPU time: {cpu_time:.4f}s for {gfx_step} steps",
            f"Simulated robot time: {counter / 360000.0:.2f}h ({counter} steps)",
            f"Total run time: {run_time:.0f}s",
        ]

        # Schalte in den 2D-Ortho-Modus für Text-Rendering
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        gluOrtho2D(0, self.window_width, 0, self.window_height)

        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        glDisable(GL_DEPTH_TEST)  # Tiefentest für 2D-Rendering deaktivieren
        glEnable(GL_BLEND)  # Alpha-Blending aktivieren für Transparenz
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        # Text zeichnen
        y_offset = self.font.get_linesize()
        for line in lines:
            text_surface = self.font.render(line, True, (255, 255, 255))
            text_data = pygame.image.tostring(text_surface, "RGBA", True)
            width, height = text_surface.get_size()

            glRasterPos2f(pos[0], self.window_height - pos[1] - y_offset)
            glDrawPixels(width, height, GL_RGBA, GL_UNSIGNED_BYTE, text_data)

            y_offset += self.font.get_linesize()

        # Auf ursprüngliche Einstellungen zurücksetzen
        glDisable(GL_BLEND)
        glPopMatrix()  # Zurück zur Modellansichtsmatrix
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glEnable(GL_DEPTH_TEST)  # Tiefentest wieder aktivieren

    def handle_control_events(self, event):
        camera_speed = 0.3  # velocity of camera movement
        glMatrixMode(GL_PROJECTION)  # to modify projection_matrix
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 4:  # mousewheel up
                glTranslatef(0, 0, camera_speed)  # moves camera to the front
            elif event.button == 5:  # mousewheel up down
                glTranslatef(0, 0, -camera_speed)  # moves camera to the back
            self.update_shader_projection_matrix()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                glTranslatef(camera_speed, 0, 0)  # moves camera to the left 0)
            elif event.key == pygame.K_RIGHT:
                glTranslatef(-camera_speed, 0, 0)  # moves camera to the right
            elif event.key == pygame.K_UP:
                glTranslatef(0, -camera_speed, 0)  # moves camera upwards
            elif event.key == pygame.K_DOWN:
                glTranslatef(0, camera_speed, 0)  # moves camera downwards
            self.update_shader_projection_matrix()
        glMatrixMode(GL_MODELVIEW)

    def handle_mouse_actions(self):
        self.get_mouse_interaction_data()

        # Apply transformations based on mouse interactions
        if self.pressed_mouse_buttons[0] is True:  # left click
            glRotatef(self.x_rotate, 1, 0, 0)
            glRotatef(self.y_rotate, 0, 1, 0)
            self.update_shader_view_matrix()
        elif self.pressed_mouse_buttons[2] is True:  # right click
            glTranslatef(self.x_move, self.y_move, self.z_move)
            self.update_shader_view_matrix()

    def get_mouse_interaction_data(self):
        mouse_x, mouse_y = pygame.mouse.get_pos()
        self.pressed_mouse_buttons = pygame.mouse.get_pressed()

        if self.pressed_mouse_buttons[0]:  # Left mouse button
            if self.x_old != -1 and self.y_old != -1:
                self.y_rotate = float((mouse_x - self.x_old) / 10)
                self.x_rotate = float((mouse_y - self.y_old) / 10)
            self.x_old = mouse_x
            self.y_old = mouse_y

        elif self.pressed_mouse_buttons[2]:  # Right mouse button
            if self.x_old != -1 and self.y_old != -1:
                self.x_move = float((mouse_x - self.x_old) / 10)
                self.z_move = float((mouse_y - self.y_old) / 10)
            self.x_old = mouse_x
            self.y_old = mouse_y

        else:
            self.x_old = -1
            self.y_old = -1

    def destroy(self):
        glDeleteVertexArrays(1, [self.vao])
        glDeleteBuffers(2, self.vbo)
