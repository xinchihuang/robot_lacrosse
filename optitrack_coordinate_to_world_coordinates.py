import numpy as np
from scipy.spatial.transform import Rotation as R

def optitrack_coordinate_to_world_coordinates(position, rotation):
    """
    Converts a rigid body position or point position from optitrack coordinate to world coordinates
    Args:
        position: [x, y, z] x front, y up, z right
        rotation: Quaternion (x, y, z, w)

    Returns:
        x, y, z, theta: x front, y left, z up, theta in radians from x-axis ccw
    """
    # Convert position: swap y and z and negate the new y to change from right to left
    world_pos = np.array([position[0], -position[2], position[1]])
    x, y, z, w = rotation
    rotation = R.from_quat([x, y, z, w])
    # Convert to Euler angles
    euler_angles = rotation.as_euler('yzx', degrees=False)
    print(f"Euler angles: {euler_angles}")
    beta = euler_angles[0]
    return world_pos[0], world_pos[1], world_pos[2], beta

# Example usage
position = [1, 2, 3]  # x front, y up, z right
rotation = [0, -np.sqrt(3/4), 0, np.sqrt(1/4)]  # Quaternion representing 90 degree rotation around x-axis

x, y, z, theta = optitrack_coordinate_to_world_coordinates(position, rotation)
print(f"Converted coordinates: x={x}, y={y}, z={z}, theta={np.rad2deg(theta)} degrees")
