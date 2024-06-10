import numpy as np
def rotate_matrix_coordinate(r_x,r_y,r_z):
    """
    Right hand coordinate rotation matrix for coordinate rotation
    Args:
        r_x: rotation angle(degree) in x axis(counter-clockwise direction from zero) y->z
        r_y: rotation angle in y axis x->z
        r_z: rotation angle in z axis x->y

    Returns: rotation matrix in x-y-z order

    """
    r_x=np.radians(-r_x)
    r_y=np.radians(-r_y)
    r_z=np.radians(-r_z)
    Rotate_x=np.array([
        [1, 0, 0],
        [0, np.cos(r_x), -np.sin(r_x)],
        [0, np.sin(r_x), np.cos(r_x)]
    ])
    Rotate_y=np.array([
        [np.cos(r_y), 0, -np.sin(r_y)],
        [0, 1, 0],
        [np.sin(r_y), 0, np.cos(r_y)]
    ])
    Rotate_z=np.array([
        [np.cos(r_z), -np.sin(r_z), 0],
        [np.sin(r_z), np.cos(r_z), 0],
        [0, 0, 1]
    ])
    return Rotate_z@Rotate_y@Rotate_x

print(rotate_matrix_coordinate(0,0,-180)@np.array([1,0,0]))