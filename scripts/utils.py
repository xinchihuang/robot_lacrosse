import numpy as np
# from cv_bridge import CvBridge, CvBridgeError
import cv2
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