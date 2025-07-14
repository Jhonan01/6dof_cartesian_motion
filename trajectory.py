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
    """Interpolação linear escalar"""
    return start + t*(end - start)

def lerp_pose(start_pose, end_pose, steps):
    """Interpolação linear para posição + orientação (6 valores)"""
    poses = []
    for i in range(steps+1):
        t = i/steps
        interp = [lerp(start_pose[j], end_pose[j], t) for j in range(6)]
        poses.append(interp)
    return poses

def dentro_dos_limites(angles):
    for i, angle in enumerate(angles):
        low, high = JOINT_LIMITS[i]
        if angle < low or angle > high:
            return False
    return True
