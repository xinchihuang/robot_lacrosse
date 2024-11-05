import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import math
import cv2
from pyglet import image
from squaternion import Quaternion
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
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
    print(cv_image.shape)
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
def detect_ball_3D(data):
    ball_size = 0.055
    focal_length = 500.39832201574455

    def estimate_distance(pixel_height, focal_length=focal_length, known_height=ball_size):
        distance = (focal_length * known_height) / pixel_height
        return distance

    def non_maximum_suppression(boxes, scores, overlap_threshold=0.8):
        if len(boxes) == 0:
            return []

        boxes = np.array(boxes)
        scores = np.array(scores)
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        indices = np.argsort(scores)[::-1]

        selected_boxes = []
        while len(indices) > 0:
            i = indices[0]
            selected_boxes.append(i)
            xx1 = np.maximum(x1[i], x1[indices[1:]])
            yy1 = np.maximum(y1[i], y1[indices[1:]])
            xx2 = np.minimum(x2[i], x2[indices[1:]])
            yy2 = np.minimum(y2[i], y2[indices[1:]])

            w = np.maximum(0, xx2 - xx1 + 1)
            h = np.maximum(0, yy2 - yy1 + 1)
            overlap = (w * h) / areas[indices[1:]]

            indices = np.delete(indices, np.concatenate(([0], np.where(overlap > overlap_threshold)[0] + 1)))

        return [boxes[i] for i in selected_boxes]

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    image_size=cv_image.shape[0]
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])
    mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
    # mask = cv2.erode(mask, None, iterations=2)
    # mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    boxes = []
    scores = []
    x_w = None
    y_w = None
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if h > 3:
            area = w * h
            boxes.append([x, y, x + w, y + h])
            scores.append(area)

    selected_boxes = non_maximum_suppression(boxes, scores)
    if len(selected_boxes) == 0:
        return x_w, y_w, None

    (x1, y1, x2, y2) = selected_boxes[0]
    pixel_height = y2 - y1
    distance = estimate_distance(pixel_height)
    x_pixel = (x1+x2)/2
    y_pixel = (y1+y2)/2
    x_w = distance * (x_pixel-image_size / 2) / focal_length
    y_w = distance * (y_pixel-image_size / 2) / focal_length
    # print(x_w,y_w,distance)
    return x_w, y_w, distance


def check_distance(position1,position2):
    return ((position1.x-position2.x)**2+(position1.y-position2.y)**2+(position1.z-position2.z)**2)**0.5

def fit_line(ball_memory):
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
        ### check error type 1: Noise point far too far away
        if points[point_index][1]<0.47:
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
