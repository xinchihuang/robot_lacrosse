from scipy.optimize import least_squares
from sympy import symbols, Eq, solve
from scripts.lstm_scratch import DynamicLSTM
import torch
from scripts.utils import *


class ParabolaFitter:
    def __init__(self, number_of_points):
        self.number_of_points = number_of_points

    def fit(self, data):
        pass

    def predict(self, x):
        pass

    def solve(self, z):
        pass


class ParabolaFitterDirect3D(ParabolaFitter):

    def fit_plane_and_parabola(self, params, x_f, y_f, z_f):
        a, b, c, d, e, f, g, h = params
        # 计算 y = gx + h
        y_est = g * x_f + h
        # 计算 z = ax^2 + b*x*y + c*y^2 + d*x + e*y + f
        z_est = a * x_f ** 2 + b * x_f * y_est + c * y_est ** 2 + d * x_f + e * y_est + f
        # 返回差的平方和
        return np.concatenate([(y_f - y_est), (z_f - z_est)])

    def fit(self, data):
        x_f = data[:self.number_of_points, 0]
        y_f = data[:self.number_of_points, 1]
        z_f = data[:self.number_of_points, 2]
        # 初始参数猜测
        initial_guess = [1, 1, 1, 1, 1, 1, 1, 1]

        # 最小二乘法求解
        result = least_squares(self.fit_plane_and_parabola, initial_guess, args=(x_f, y_f, z_f))
        # print(result)
        a, b, c, d, e, f, g, h = result.x
        self.parameters = [a, b, c, d, e, f, g, h]

    def predict(self, x):
        a, b, c, d, e, f, g, h = self.parameters[0], self.parameters[1], self.parameters[2], self.parameters[3], \
        self.parameters[4], self.parameters[5], self.parameters[6], self.parameters[7]
        y = g * x + h
        z = a * x ** 2 + b * x * y + c * y ** 2 + d * x + e * y + f
        return y, z

    def solve(self, z):
        a, b, c, d, e, f, g, h = self.parameters[0], self.parameters[1], self.parameters[2], self.parameters[3], \
            self.parameters[4], self.parameters[5], self.parameters[6], self.parameters[7]
        x, y = symbols('x y', real=True)
        y_eq = Eq(y, g * x + h)
        z_eq = Eq(a * x ** 2 + b * x * y + c * y ** 2 + d * x + e * y + f, z)

        # 将y方程代入z方程
        # z_eq_substituted = z_eq.subs(y, g * x + h)
        # print(self.parameters)
        # 求解x
        solution = solve((y_eq, z_eq), (x, y))
        print(a, b, c, d, e, f, g, h,solution)
        # solution_y = solve(z_eq_substituted, y)

        # print("二次方程的两个根是：")
        # print(solution)
        # x1,y1 distance to x2,y2
        x1 = solution[0]
        x2 = solution[1]
        return x1, x2
class ParabolaFitterRansac(ParabolaFitter):
    def __init__(self,number_of_points):
        super().__init__(number_of_points)

    def fit_line(self, ball_memory):
        ball_memory = np.array(ball_memory)
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
        return a, b, inlier_mask

    def fit_parabola(self, ball_memory):
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

    def world_to_parabola_coordinate(self, ball_memory, m, b):
        new_coordinate = []
        for point in ball_memory:
            new_coordinate.append([(m * (point[1] - b) + point[0]) / math.sqrt(m ** 2 + 1), point[2]])
        return np.array(new_coordinate)

    def fit(self, ball_memory):
        ball_memory = np.array(ball_memory)
        ball_memory_to_fit = ball_memory[:self.number_of_points]
        m, intercept, inlier_mask = self.fit_line(ball_memory_to_fit)
        # print(m, intercept)
        new_points_to_fit = self.world_to_parabola_coordinate(ball_memory_to_fit, m, intercept)
        new_points_to_fit = point_filters(new_points_to_fit)
        self.new_points_to_fit = new_points_to_fit

        a, b, c = self.fit_parabola(new_points_to_fit)
        self.params = [m, intercept, a, b, c]

    def predict(self, x, y):
        m, intercept, a, b, c = self.params
        print(m, intercept, a, b, c)
        x_parabola = self.world_to_parabola_coordinate([[x, y, 0]], m, intercept)[0][0]
        # print("x",x_parabola)
        z_m = a * x_parabola ** 2 + b * x_parabola + c
        x_m, y_m = x_parabola / math.sqrt(1 + m ** 2), x_parabola * m / math.sqrt(
            1 + m ** 2)
        print(x_parabola, x_m)
        return x_m, y_m + intercept, z_m
    def solve(self,z):
        m, intercept, a, b, c = self.params
        x = symbols('x')
        z_eq = Eq(a * x ** 2 + b * x + c, z)
        solution = solve(z_eq, x)
        x1 = solution[0]
        x2 = solution[1]
        x1_m, y1_m = x1 / math.sqrt(1 + m ** 2), x1 * m / math.sqrt(
            1 + m ** 2)
        x2_m, y2_m = x2 / math.sqrt(1 + m ** 2), x2 * m / math.sqrt(
            1 + m ** 2)
        return [(x1_m, y1_m + intercept), (x2_m, y2_m + intercept)]

class ParabolaFitterLSTM(ParabolaFitter):
    def __init__(self, model_path,number_of_points):
        super().__init__(model_path)
        self.number_of_points =number_of_points
        self.model = DynamicLSTM(input_dim=2, hidden_dim=20, output_dim=1,
                                 num_layers=2)
        self.model.load_state_dict(torch.load(model_path))
        self.new_points_to_fit=None

    def fit_line(self, ball_memory):
        ball_memory = np.array(ball_memory)
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
        return a, b, inlier_mask

    def fit_parabola(self, ball_memory):
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

    def world_to_parabola_coordinate(self, ball_memory, m, b):
        new_coordinate = []
        for point in ball_memory:
            new_coordinate.append([(m*(point[1] - b) + point[0])/math.sqrt(m**2+1), point[2]])
        return np.array(new_coordinate)

    def fit(self, ball_memory):

        ball_memory = np.array(ball_memory)
        ball_memory_to_fit = ball_memory[:self.number_of_points]
        m, intercept, inlier_mask = self.fit_line(ball_memory_to_fit)
        # print(m, intercept)
        new_points_to_fit = self.world_to_parabola_coordinate(ball_memory_to_fit, m, intercept)
        new_points_to_fit = point_filters(new_points_to_fit)
        self.new_points_to_fit=new_points_to_fit

        a, b, c = self.fit_parabola(new_points_to_fit)
        self.params = [m, intercept, a, b, c]

    def predict(self, x):
        m, intercept, a, b, c=self.params
        x_parabola  = self.world_to_parabola_coordinate([[x,0,0]], m, intercept)[0][0]
        # print("x",x_parabola)
        z_m = a * x_parabola ** 2 + b * x_parabola + c
        x_m, y_m = x_parabola / math.sqrt(1 + m ** 2), x_parabola * m / math.sqrt(
            1 + m ** 2)
        print(x_parabola,x_m)
        return x_m,y_m+intercept,z_m

    def solve(self, z):
        m, intercept, a, b, c = self.params
        x= symbols('x')
        z_eq = Eq(a * x ** 2 + b * x  + c , z)
        solution = solve(z_eq, x)
        x1 = solution[0]
        x2 = solution[1]
        sequence_length = torch.tensor([len(self.new_points_to_fit)])
        residual = self.model(torch.tensor(self.new_points_to_fit).float().unsqueeze(0), sequence_length)
        x1=x1+residual.item()
        x2=x2+residual.item()
        x1_m, y1_m = x1 / math.sqrt(1 + m ** 2), x1 * m / math.sqrt(
            1 + m ** 2)
        x2_m, y2_m = x2 / math.sqrt(1 + m ** 2), x2 * m / math.sqrt(
            1 + m ** 2)
        return [(x1_m, y1_m+intercept),(x2_m, y2_m+intercept)]
