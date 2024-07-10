import numpy as np
from scipy.optimize import fsolve

# Example fitted parameters [a, b, c, d, e, f]
params = [1, 1, 0, 1, 1, 1]  # These should be replaced with your actual fitted parameters

# The function to find x, y for a given z
def equations(p, params, z_given):
    x, y = p
    a, b, c, d, e, f = params
    # Return an array of residuals (equations)
    return [
        a*x**2 + b*y**2 + c*x*y + d*x + e*y + f - z_given
    ]

# Example z value
z_given = 15

# Initial guess for x and y
initial_guess = [0, 0]

# Solve for x and y
solution = fsolve(equations, initial_guess, args=(params, z_given))

# Since fsolve might sometimes return a flat array, reshape if necessary
if solution.ndim > 1:
    solution = solution.flatten()

print(f"Solved x, y: ({solution[0]}, {solution[1]})")
