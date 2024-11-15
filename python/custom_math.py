
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