from numpy import array, cos, sin, allclose
from numpy.linalg import norm

def quaternion_matrix_left(q):
    q0, q1, q2, q3 = q.q
    Q_left = array([
        [q0, -q1, -q2, -q3],
        [q1, q0, -q3, q2],
        [q2, q3, q0, -q1],
        [q3, -q2, q1, q0]
    ])
    return Q_left

def quaternion_matrix_right(q):
    q0, q1, q2, q3 = q.q
    Q_right = array([
        [q0, -q1, -q2, -q3],
        [q1, q0, q3, -q2],
        [q2, -q3, q0, q1],
        [q3, q2, -q1, q0]
    ])
    return Q_right

class quaternion:
    def __init__(self, w=0, x=0, y=0, z=0):
        self.q = [w, x, y, z]

    @staticmethod
    def build_from_angle(cls, angle, axis):
        half_angle = angle/2
        w = cos(half_angle)
        x = sin(half_angle) * axis[0]
        y = sin(half_angle) * axis[1]
        z = sin(half_angle) * axis[2]
        return quaternion(w, x, y, z)

    @staticmethod
    def build_from_vector(cls, w, v):
        x = v[0]
        y = v[1]
        z = v[2]
        return quaternion(w, x, y, z)

    @staticmethod
    def build_from_list(cls, lst):
        return quaternion(lst[0], lst[1], lst[2], lst[3])

    def __repr__(self):
        return f"quaternion({self.q[0]}, {self.q[1]}, {self.q[2]}, {self.q[3]})"

    def __add__(self, other):
        if isinstance(other, quaternion):
            ret = quaternion.build_from_list([self.q[i] + other.q[i] for i in range(len(self.q))])
        else:
            raise TypeError("Addition is only supported between two quaternions")

    def __sub__(self, other):
        if isinstance(other, quaternion):
            ret = quaternion.build_from_list([self.q[i] - other.q[i] for i in range(len(self.q))])
        else:
            raise TypeError("Subtraction is only supported between two quaternions")

    def __mul__(self, other):
        if isinstance(other, quaternion):
            w1, x1, y1, z1 = self.q
            w2, x2, y2, z2 = other.q
            return quaternion.build_from_list([
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
            ])
        elif isinstance(other, (int, float)):
            return quaternion.build_from_list(self.q * other)
        else:
            raise TypeError("Multiplication is only supported between quaternions or with a scalar")

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            return quaternion.build_from_list([self.q[i] / other for i in range(len(self.q))])
        else:
            raise TypeError("Division is only supported between a quaternion and a scalar")

    def conjugate(self):
        w, x, y, z = self.q
        return quaternion.build_from_list([w, -x, -y, -z])

    def norm(self):
        return norm(self.q)

    def normalize(self):
        return self / self.norm()

    def inverse(self):
        return self.conjugate() / (self.norm() ** 2)

    def vector(self):
        return [self.q[1], self.q[2], self.q[3]]

    def __eq__(self, other):
        if isinstance(other, quaternion):
            return allclose(self.q, other.q)
        else:
            return False
