from math import pi

JOINT_LIMITS = [
    (-pi, pi),            # q1
    (-pi/2, pi/2),        # q2
    (-pi, pi),            # q3
    (-pi, pi),            # q4
    (-pi, pi),            # q5
    (-pi, pi)             # q6
]

def lerp(start, end, t):
    """interpolation between start and end with t in [0, 1]"""
    return start + t*(end - start)

def lerp_pose(start_pose, end_pose, steps):
    """linear interpolation for position + orientation (6 values)"""
    poses = []
    for i in range(steps+1):
        t = i/steps
        interp = [lerp(start_pose[j], end_pose[j], t) for j in range(6)]
        poses.append(interp)
    return poses

def inside_of_limits(angles):
    for i, angle in enumerate(angles):
        low, high = JOINT_LIMITS[i]
        if angle < low or angle > high:
            return False
    return True
