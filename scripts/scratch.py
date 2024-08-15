import math
import numpy as np
def world_to_parabola_coordinate(ball_memory, m, b):

    new_coordinate = []
    for point in ball_memory:

        new_coordinate.append([(m*(point[1] - b) + point[0])/math.sqrt(m**2+1), point[2]])

    return np.array(new_coordinate)
x=[[-1,0,0]]
m=1
b=1
landing_x_parabola=world_to_parabola_coordinate(x,1,1)
print(world_to_parabola_coordinate(x,1,1))
x, y = landing_x_parabola / math.sqrt(1 + m ** 2), landing_x_parabola * m / math.sqrt(
    1 + m ** 2)
