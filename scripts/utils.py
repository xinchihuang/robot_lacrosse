import numpy as np
# from cv_bridge import CvBridge, CvBridgeError
import math
import cv2
from squaternion import Quaternion
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
from scipy.spatial.transform import Rotation as R
import plotly.graph_objects as go
def calculate_rotation_angle(v1, v2):
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)
    cos_theta = dot_product / (norm_v1 * norm_v2)
    cross_product = v1[0] * v2[1] - v1[1] * v2[0]
    angle = np.arctan2(cross_product, cos_theta)
    return angle

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
        [np.cos(r_y), 0, np.sin(r_y)],
        [0, 1, 0],
        [-np.sin(r_y), 0, np.cos(r_y)]
    ])
    Rotate_z=np.array([
        [np.cos(r_z), -np.sin(r_z), 0],
        [np.sin(r_z), np.cos(r_z), 0],
        [0, 0, 1]
    ])
    return Rotate_z@Rotate_y@Rotate_x
def detect_ball(data):
    # Detect Ball
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])
    mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x_pixel = -1
    y_pixel = -1
    for contour in contours:
        M = cv2.moments(contour)
        if M['m00'] != 0:
            x_pixel = int(M['m10'] / M['m00'])
            y_pixel = int(M['m01'] / M['m00'])
            # if self.robot_name=="robot1":
            #     print(self.robot_name,f"Centroid of the blue object: X Position {x_pixel}, Y Position {y_pixel}")
    return x_pixel,y_pixel

def check_distance(position1,position2):
    return ((position1.x-position2.x)**2+(position1.y-position2.y)**2+(position1.z-position2.z)**2)**0.5

def fit_line(ball_memory):
    ball_memory=np.array(ball_memory)
    x = ball_memory[:, 0]
    x_reshape = x.reshape(-1, 1)
    y = ball_memory[:, 1]
    ransac = make_pipeline(PolynomialFeatures(degree=1), RANSACRegressor())
    ransac.fit(x_reshape, y)
    coefficients = ransac.named_steps['ransacregressor'].estimator_.coef_
    intercept = ransac.named_steps['ransacregressor'].estimator_.intercept_
    inlier_mask = ransac.named_steps['ransacregressor'].inlier_mask_
    a = coefficients[1]
    b = intercept + coefficients[0]
    return a, b,inlier_mask

def fit_parabola(ball_memory):
    x = ball_memory[:, 0]
    x_reshape = x.reshape(-1, 1)
    y = ball_memory[:, 1]
    ransac = make_pipeline(PolynomialFeatures(degree=2), RANSACRegressor())
    ransac.fit(x_reshape, y)
    coefficients = ransac.named_steps['ransacregressor'].estimator_.coef_
    intercept = ransac.named_steps['ransacregressor'].estimator_.intercept_
    a = coefficients[2]
    b = coefficients[1]
    c = intercept + coefficients[0]
    return a, b, c

def world_to_parabola_coordinate(ball_memory, m, b):
    new_coordinate = []
    for point in ball_memory:
        if (point[1] - b) * m > 0:
            new_coordinate.append([math.sqrt((point[1] - b) ** 2 + (point[0] ** 2)), point[2]])
        else:
            new_coordinate.append([-math.sqrt((point[1] - b) ** 2 + (point[0] ** 2)), point[2]])
    return np.array(new_coordinate)

def root(a, b, c):
    """
    Calculates the roots of the parabola equation
    If no real roots are found return None
    Args:
        a:
        b:
        c:

    Returns: roots

    """
    if b ** 2 - 4 * a * c < 0:
        print(b ** 2 - 4 * a * c)
        return None, None
    return (-b + math.sqrt(b ** 2 - 4 * a * c)) / 2 / a, (-b - math.sqrt(b ** 2 - 4 * a * c)) / 2 / a
def point_filters(points):
    filtered_points = []
    point_index=0
    check_window = 3
    thresh = 0.5
    while point_index<len(points)-1:
        ### check error type 1: Noise point far too low
        if points[point_index][1]<0.4:
            point_index += 1
            continue
        ### check error type 2: Noise point far too far away

        if point_index<1:
            filtered_points.append(points[point_index])
        elif abs(points[point_index][0]-points[point_index-1][0])<thresh:
            filtered_points.append(points[point_index])
        elif abs(points[point_index][0]-points[point_index-1][0])>thresh:
            noise=False
            for i in range(1,check_window+1):
                if point_index+i>=len(points):
                    noise = True
                elif points[point_index+i][0]-points[point_index][0]<0:
                    noise = True
            if noise==False:
                filtered_points.append(points[point_index])
        point_index+=1
    return np.array(filtered_points)
def optitrack_coordinate_to_world_coordinates(position, rotation,is_ball=False):
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
    if is_ball == True:
        return world_pos[0], world_pos[1], world_pos[2], 0
    x, y, z, w = rotation
    rotation = R.from_quat([x, y, z, w])
    # Convert to Euler angles
    euler_angles = rotation.as_euler('yzx', degrees=False)
    # print(f"Euler angles: {euler_angles}")
    beta = euler_angles[0]
    return world_pos[0], world_pos[1], world_pos[2], beta
