import random
import numpy as np
from scipy.optimize import least_squares
import time
import math
def rotate_matrix_coordinate(r_x,r_y,r_z):
    """
    Right hand coordinate rotation matrix for coordinate rotation
    Note this is for coordinate rotation only

    Args:
        r_x: rotation angle(degree) in x axis(counter-clockwise direction from zero) y->z
        r_y: rotation angle in y axis x->z
        r_z: rotation angle in z axis x->y

    Returns: rotation matrix in x-y-z order relative to the rotation axis

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
def translation_vector_coordinate(t_x,t_y,t_z):
    return np.array((t_x,t_y,t_z))
def transform_coordinate(points,rotation_matrix,translation_vector):
    """
    3D transformation of a coordinate
    Get the point's new coordinate
    Rotation
    Args:
        points: Old coordinate
        rotation_matrix: Coordinate rotation matrix
        translation_vector:

    Returns: New coordinate of the point

    """
    # calculate the translation first
    points_translated=np.array(points)-translation_vector
    new_points=rotation_matrix@points_translated
    return new_points

class ParabolaFitter:
    def __init__(self,robot_pose=[0,0,0],camera_rpy_to=[0,0,0],camera_rpy_back=[0,0,0],min_data_samples=100):

        self.camera_pose=robot_pose
        # This rpy indicates the transformation of a vector from world coordinate to the camera coordinate
        self.camera_rpy_to=camera_rpy_to
        # This rpy indicates the transformation of a vector from camera coordinate to the world coordinate
        self.camera_rpy_back = camera_rpy_back
        self.g=-10
        self.min_date_sample=min_data_samples


    def basic_fit(self):

        pass
    def is_inlier(self,data_points,model_points,threshold):
        """
        Determine if a model predicted point is inlier
        Args:
            data_points:
            model_points:
            threshold:
        Returns: True if it is inlier, False otherwise

        """
        if np.linalg.norm(data_points-model_points)<threshold:
            return True
        else:
            return False
    def ransac_fit(self,data,model,is_inlier,sample_size,iterations,threshold):
        """
            Generic RANSAC implementation

            Args:
            - data: Full dataset
            - model: Function to fit model to a sample
            - is_inlier: Function to check if a point fits the model
            - sample_size: Minimum points needed to fit the model
            - iterations: Number of RANSAC iterations
            - threshold: Inlier determination criterion

            Returns:
            - Best model found
            - Indices of inliers
            """
        best_model = None
        best_inliers = []

        for _ in range(iterations):
            # Randomly sample data
            sample_indices = random.sample(range(len(data)), sample_size)
            sample = [data[i] for i in sample_indices]

            # Fit model to sample
            potential_model = model(sample)

            # Find inliers
            inliers = [
                i for i, point in enumerate(data)
                if is_inlier(potential_model, point, threshold)
            ]

            # Update best model if more inliers found
            if len(inliers) > len(best_inliers):
                best_model = potential_model
                best_inliers = inliers

        return best_model, best_inliers
    def parabola_model(self,params,x,y):
        a,b,c,d,e,f,g,h = params
        z=a*x**2+b*y**2+c*x*y+d*x+e*y+f
        return x,y,z
    def parabola_model_time(self,params,t):
        x_0,y_0,z_0,vx_0,vy_0,vz_0 = params
        x=x_0+vx_0*t
        y=y_0+vy_0*t
        z=z_0+vz_0*t+0.5*self.g*t**2
        return x,y,z
    def direct_fitter(self):
        pass
    def mono_fitter(self,data,odometry=None):
        """
        Fit a parabola with point's position(in meters 2D) in the camera coordinate
        The length of odometry must be the same as the length of the data
        Args:
            data: [[x_c,y_c,t],
                    [x_c,y_c,t]]
            odometry: [[x_w,y_w,z_w,t],] or None
        Returns: Parabola fitted params ndarrays

        """
        # print(data)
        if len(data)<self.min_date_sample:
            return None
        t_start = data[0][2]
        rotation_matrix_to=rotate_matrix_coordinate(self.camera_rpy_to[0],self.camera_rpy_to[1],self.camera_rpy_to[2])
        g_vector=np.array([0,0,self.g])
        g_x_c,g_y_c,g_z_c=transform_coordinate(g_vector,rotation_matrix_to,[0,0,0])
        # print(g_x_c,g_y_c,g_z_c)

        A=[]
        B=[]
        for i in range(0,len(data)):
            t=data[i][2]-t_start
            camera_vector = np.concatenate((np.array(data[i][:2]),np.array([1])))
            # compensate the movement of the robot with odometry data
            # make all points([x_c,y_c]) in camera coordinate relative to the start camera position
            if odometry is not None:
                # Equals to move opposite direction
                translation = -1*np.array(odometry[i][:3])
                # Assume the camera didn't rotate over time
                no_rotation = rotate_matrix_coordinate(0, 0,0)
                camera_vector = transform_coordinate(camera_vector,no_rotation,translation)
                # print(camera_vector)
            # All data points in camera coordinate
            alpha=camera_vector[0]
            beta=camera_vector[1]
            A.append([1, t, 0, 0, -alpha, -alpha * t])
            A.append([0, 0, 1, t, -beta, -beta * t])
            B.append(0.5 * t * t * (g_z_c * alpha - g_x_c))
            B.append(0.5 * t * t * (g_z_c * beta - g_y_c))
        A_array = np.array(A)
        B_array = np.array(B)

        initial_w = [0, 0, 0]
        initial_c = transform_coordinate(initial_w, rotation_matrix_to, [0, 0, 0])
        initial_vw = [0, 0, 0]
        initial_vc = transform_coordinate(initial_vw, rotation_matrix_to, [0, 0, 0])
        initial = np.array([initial_c[0], initial_vc[0], initial_c[1], initial_vc[1], initial_c[2], initial_vc[2]])
        # pseudo_inverse_A = np.linalg.pinv(A_array)
        # solved_parameters = pseudo_inverse_A @ B_array
        #
        # # The params in the camera coordinate
        # least_squares
        # x0_c, vx_c, y0_c, vy_c, z0_c, vz_c = solved_parameters[0][0], solved_parameters[1][0], solved_parameters[2][
        #     0], solved_parameters[3][0], solved_parameters[4][0], solved_parameters[5][0]
        # def residuals(x,A,b):
        #     return A @ x - b

        # solved_parameters = least_squares(residuals, x0, method='trf',args=(A_array,B_array))
        # x0_c, vx_c, y0_c, vy_c, z0_c, vz_c=solved_parameters.x
        def gradient_descent(A, B, initial, learning_rate=0.1, tolerance=0.01, max_iter=100,max_grad_norm=1000):
            
            x = initial
            for i in range(max_iter):
                gradient = -2 * A.T @ (B - A @ x)
                grad_norm = np.linalg.norm(gradient)
                if grad_norm > max_grad_norm:
                    gradient = gradient * (max_grad_norm / grad_norm)
                if grad_norm > 50:
                    learning_rate *= 0.5
                elif grad_norm < 10:
                    learning_rate *= 1.05
                # print(grad_norm)
                x_new = x - learning_rate * gradient
                # print(np.linalg.norm(x_new - x))
                if grad_norm < tolerance:

                    break
                x = x_new
            return x
        solved_parameters=gradient_descent(A_array, B_array,initial)
        x0_c, vx_c, y0_c, vy_c, z0_c, vz_c = solved_parameters
        rotation_matrix_back=rotate_matrix_coordinate(self.camera_rpy_back[0],self.camera_rpy_back[1],self.camera_rpy_back[2])
        x_0,y_0,z_0=transform_coordinate(np.array([x0_c,y0_c,z0_c]),rotation_matrix_back,[0,0,0])
        vx, vy, vz = transform_coordinate(np.array([vx_c, vy_c, vz_c]), rotation_matrix_back, [0, 0, 0])
        return x_0,y_0,z_0,vx,vy,vz
    def mono_pixel_fitter(self):
        pass

if __name__ == '__main__':
    point=[-8, 0, 0]

    rotation_matrix_coordinate=rotate_matrix_coordinate(0, 45, -90)
    translate_vector=translation_vector_coordinate(0,0,0)
    new_point=transform_coordinate(point,rotation_matrix_coordinate,translate_vector)
    print(new_point)
    rotation_matrix_coordinate = rotate_matrix_coordinate(45, 0, 90)
    translate_vector = translation_vector_coordinate(0, 0, 0)
    back = transform_coordinate(new_point, rotation_matrix_coordinate, translate_vector)
    print(back)