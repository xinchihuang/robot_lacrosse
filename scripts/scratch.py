# Reusing the previously updated quaternion components for calculation
# Convert updated quaternion to Euler angles (roll, pitch, yaw) again
import math
# w=-0.0025392191018909216
# x=-0.2123934030532837
# y=0.004783613607287407
# z=0.9771692752838135
w=0.3916
x=-0.00814
y=-0.9201
z=0.00667
phi_updated_again = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
theta_updated_again = math.asin(2 * (w * y - z * x))
psi_updated_again = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

# Convert radians to degrees again
phi_updated_again_deg = math.degrees(phi_updated_again)
theta_updated_again_deg = math.degrees(theta_updated_again)
psi_updated_again_deg = math.degrees(psi_updated_again)

print(phi_updated_again_deg, theta_updated_again_deg, psi_updated_again_deg)
