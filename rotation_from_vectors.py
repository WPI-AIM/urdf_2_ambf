import PyKDL
import math


# https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/897677#897677
def skew_mat(v):
    return PyKDL.Rotation(0, -v[2], v[1],
                          v[2], 0, -v[0],
                          -v[1], v[0], 0)


def add_mat(mat1, mat2):
    out = PyKDL.Rotation()
    for i in range(0, 3):
        for j in range(0, 3):
            out[i, j] = mat1[i, j] + mat2[i, j]
    return out


def scalar_mul(mat, s):
    out = PyKDL.Rotation()
    for i in range(0, 3):
        for j in range(0, 3):
            out[i, j] = mat[i, j] * s
    return out


# https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/897677#897677
def rot_matrix_from_vecs(vec_a, vec_b):
    out = PyKDL.Rotation()
    vec_a.Normalize()
    vec_b.Normalize()
    vcross = vec_a * vec_b
    vdot = PyKDL.dot(vec_a, vec_b)
    # Check if the vectors are in the same direction
    if 1.0 - vdot < 0.1:
        return out
    # Or in the opposite direction
    elif 1.0 + vdot < 0.1:
        nx = PyKDL.Vector(1, 0, 0)
        temp_dot = PyKDL.dot(vec_a, nx)
        if -0.9 < abs(temp_dot) < 0.9:
            axis = vec_a * nx
            out = out.Rot(axis, 3.14)
        else:
            ny = PyKDL.Vector(0, 1, 0)
            axis = vec_a * ny
            out = out.Rot(axis, 3.14)
    else:
        skew_v = skew_mat(vcross)
        out = add_mat(add_mat(PyKDL.Rotation(), skew_v), scalar_mul(
            skew_v * skew_v, (1 - vdot) / (vcross.Norm() ** 2)))
    return out


##########################################################

def get_angle(v1, v2):
    dot = PyKDL.dot(v1, v2)
    angle = math.acos(dot / (v1.Norm() * v2.Norm()))
    return angle


# Get rotation matrix to represent rotation between two vectors
# Brute force implementation
def get_rot_mat_from_vecs(v1, v2):
    # Angle between two axis
    angle = get_angle(v1, v2)
    # Axis of rotation between child's joints axis and constraint_axis
    if abs(angle) <= 0.1:
        # Doesn't matter which axis we chose, the rot mat is going to be identity
        # as angle is almost 0
        axis = PyKDL.Vector(0, 1, 0)
    elif abs(angle) >= 3.13:
        # This is a more involved case, find out the orthogonal vector to vecA
        ny = PyKDL.Vector(0, 1, 0)
        temp_ang = get_angle(v1, ny)
        if 0.1 < abs(temp_ang) < 3.13:
            axis = v1 * ny
        else:
            nz = PyKDL.Vector(0, 0, 1)
            axis = v1 * nz
    else:
        axis = v1 * v2

    mat = PyKDL.Rotation()
    # Rotation matrix representing the above angular offset
    rot_mat = mat.Rot(axis, angle)
    return rot_mat

###############################################################


def round_vec(v):
    for i in range(0, 3):
        v[i] = round(v[i], 3)
    return v


def round_mat(m):
    for i in range(0, 3):
        for j in range(0, 3):
            m[i, j] = round(m[i, j])
    return m


a = PyKDL.Vector(0, 0, 1)
b = PyKDL.Vector(0, 0, -1)
r1 = rot_matrix_from_vecs(a, b)
v_test = PyKDL.Vector(0.5, 0, 0.5)
print 'Rotation:'
print 'R1'
print round_mat(r1)
print 'V1'
print round_vec(r1 * v_test)

r2 = get_rot_mat_from_vecs(a, b)
print 'Rotation:'
print 'R2'
print round_mat(r2)
print 'V2'
print round_vec(r2 * v_test)


