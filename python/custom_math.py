import random

import numpy as np


class Vector3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def cross_product(v1, v2):
    return Vector3D(
        v1.y * v2.z - v1.z * v2.y,
        v1.z * v2.x - v1.x * v2.z,
        v1.x * v2.y - v1.y * v2.x
    )


def pseudo_rand():
    rand_number = random.randint(0, 9999)
    rand_number = rand_number * 214013 + 2531011
    return ((rand_number // 1) >> 16) & 0x7fff


def calculate_translation_matrix(x, y, z):
    matrix = np.identity(4, dtype=np.float32)
    matrix[3, 0] = x
    matrix[3, 1] = y
    matrix[3, 2] = z
    return matrix


def calculate_rotation_matrix(x_angle, y_angle):
    x_rad = np.radians(x_angle)
    y_rad = np.radians(y_angle)

    rotation_x = np.array([
        [1, 0, 0, 0],
        [0, np.cos(x_rad), -np.sin(x_rad), 0],
        [0, np.sin(x_rad), np.cos(x_rad), 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    rotation_y = np.array([
        [np.cos(y_rad), 0, np.sin(y_rad), 0],
        [0, 1, 0, 0],
        [-np.sin(y_rad), 0, np.cos(y_rad), 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    return np.dot(rotation_y, rotation_x)


def rotate_vector(vector, yaw, pitch):
    yaw_rad = np.radians(yaw)
    pitch_rad = np.radians(pitch)

    rotation_yaw = np.array([
        [np.cos(yaw_rad), 0, np.sin(yaw_rad)],
        [0, 1, 0],
        [-np.sin(yaw_rad), 0, np.cos(yaw_rad)]
    ])

    rotation_pitch = np.array([
        [1, 0, 0],
        [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
        [0, np.sin(pitch_rad), np.cos(pitch_rad)]
    ])

    rotated_vector = np.dot(rotation_yaw, vector)
    rotated_vector = np.dot(rotation_pitch, rotated_vector)
    return rotated_vector
