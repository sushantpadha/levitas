import math

# Finds the radii of the path left and right wheels will follow
def find_radii(theta, l, w):
    r3 = (l / math.tan(theta)) - (w / 2)
    r1 = math.sqrt(l**2 + r3**2)
    r2 = math.sqrt(l**2 + (r3 + w)**2)
    rr = l / math.sin(theta)
    return r1, r2, rr

# Finds the velocities of the left and right wheels
def find_velocities(v, r1, r2, rr, theta, l, w):
    alpha = math.asin(w * math.sin(theta)/(2 * r2))
    beta = math.asin(w * math.sin(theta)/(2 * r1))
    vr = (v * rr * math.cos(alpha)) / r2
    v1 = (r1 * vr) / (rr * math.cos(beta))
    return v1, v

# Corrects for axis of tyres and universal joint being different.
def correction_terms(r1, r2, theta, delta):
    alpha = math.asin(w * math.sin(theta)/(2 * r2))
    beta = math.asin(w * math.sin(theta)/(2 * r1))
    return r1 - delta * math.cos(beta), r2 + delta * math.cos(alpha)

# Used when steering angle greater than maximum set axle steering angle
def diff_steering(v10, v20, theta, max_theta):
    return v10 * (2 - theta/max_theta), v20

if __name__ == "__main__":

    theta = math.pi / 4 # Angle of tyre from vertical in radians
    l = 11 # Shaft to shaft length from front to back in cm
    w = 15 # Shaft to shaft width from left to right in cm
    v = 10 # Required velocity of left wheels when turning right in cm/s
    max_theta = math.pi / 4 # Maximum angle tyres can turn in radians
    delta = 0 # Distance between axes of universal joint and tyre


    if v >= 0 and theta >= 0:
        dir1, dir2 = "front", "right"
    elif v >= 0 and theta < 0:
        theta = -theta
        dir1, dir2 = "front", "left"
    elif v < 0 and theta >= 0:
        v = -v
        dir1, dir2 = "back", "right"
    elif v < 0 and theta < 0:
        v = -v
        theta = -theta
        dir1, dir2 = "back", "left"

    r1, r2, rr = find_radii(theta, l, w)
    r1, r2 = correction_terms(r1, r2, theta, delta)
    if theta > max_theta:
        v10, v20 = find_velocities(v, r1, r2, rr, max_theta, l, w)
        v1, v2 = diff_steering(v10, v20, theta, max_theta)
    else:
        v1, v2 = find_velocities(v, r1, r2, rr, theta, l, w)

    if dir1 == "back":
        v1, v2 = -v1, -v2
    if dir2 == "left":
        v1, v2 = v2, v1

    print(f"Left tyres must move at {v2} cm/s")
    print(f"Right tyres must move at {v1} cm/s")

    

    

