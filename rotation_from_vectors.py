import PyKDL

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


def rot_matrix_from_vecs(v1, v2):
    vcross = v1 * v2
    skew_v = skew_mat(vcross)
    out = add_mat(add_mat(PyKDL.Rotation(), skew_v), scalar_mul(
        skew_v * skew_v, (1 - PyKDL.dot(v1, v2)) / (vcross.Norm() ** 2)))
    return out


a = PyKDL.Vector(1, 0, 0)
b = PyKDL.Vector(0, 1, 0)
r = rot_matrix_from_vecs(a, b)
print 'Rotation:'
print r
