import numpy as np
import matplotlib.pyplot as plt
from sympy import symbols, cos, sin, pi, simplify, Matrix, atan2, sqrt
import time
import math

# --- Cinemática direta e inversa baseada no seu código original ---

def pose(theta, alpha, a, d):
    r11, r12 = cos(theta), -sin(theta)
    r23, r33 = -sin(alpha), cos(alpha)
    r21 = sin(theta) * cos(alpha)
    r22 = cos(theta) * cos(alpha)
    r31 = sin(theta) * sin(alpha)
    r32 = cos(theta) * sin(alpha)
    y = -d * sin(alpha)
    z = d * cos(alpha)
    
    T = Matrix([
        [r11, r12, 0.0, a],
        [r21, r22, r23, y],
        [r31, r32, r33, z],
        [0.0, 0.0, 0.0, 1]
    ])
    return simplify(T)

def forward_kin(q1,q2,q3,q4,q5,q6):
    d90 = pi/2
    T01 = pose(q1, 0, 0, 0.75)
    T0g = T01
    points = [T0g]
    T12 = pose(q2 - d90, -d90, 0.35, 0)
    T0g = T0g * T12
    points.append(T0g)
    T23 = pose(q3, 0, 1.25, 0)
    T0g = T0g * T23
    points.append(T0g)
    T34 = pose(q4, -d90, -0.054, 1.5)
    T0g = T0g * T34
    points.append(T0g)
    T45 = pose(q5, d90, 0, 0)
    T0g = T0g * T45
    points.append(T0g)
    T56 = pose(q6, -d90, 0, 0)
    T0g = T0g * T56
    points.append(T0g)
    T6g = pose(0, 0, 0, 0.303)
    T0g = T0g * T6g
    points.append(T0g)
    
    # Extrai só as posições x,y,z de cada junta para plotagem
    X = [float(p[0,3]) for p in points]
    Y = [float(p[1,3]) for p in points]
    Z = [float(p[2,3]) for p in points]
    return X, Y, Z

# Funções auxiliares para IK (baseadas no seu código original)
def get_hypotenuse(a, b):
    return sqrt(a*a + b*b)

def get_cosine_law_angle(a, b, c):    
    cos_gamma = (a*a + b*b - c*c) / (2*a*b)
    sin_gamma = sqrt(1 - cos_gamma * cos_gamma)
    gamma = atan2(sin_gamma, cos_gamma)
    return gamma

def get_wrist_center(gripper_point, R0g, dg = 0.303):
    xu, yu, zu = gripper_point 
    nx, ny, nz = R0g[0, 2], R0g[1, 2], R0g[2, 2]
    xw = xu - dg * nx
    yw = yu - dg * ny
    zw = zu - dg * nz 
    return xw, yw, zw

def get_first_three_angles(wrist_center):
    x, y, z  = wrist_center
    a1, a2, a3 = 0.35, 1.25, -0.054
    d1, d4 = 0.75, 1.5
    l = 1.50097168527591
    phi = 1.53481186671284
    
    x_prime = get_hypotenuse(x, y)
    mx = x_prime -  a1
    my = z - d1 
    m = get_hypotenuse(mx, my)
    alpha = atan2(my, mx)
    
    gamma = get_cosine_law_angle(l, a2, m)
    beta = get_cosine_law_angle(m, a2, l)
    
    q1 = atan2(y, x)
    q2 = pi/2 - beta - alpha 
    q3 = -(gamma - phi)
    
    return q1, q2, q3 

def get_last_three_angles(R):
    sin_q4 = R[2, 2]
    cos_q4 =  -R[0, 2]
    
    sin_q5 = sqrt(R[0, 2]**2 + R[2, 2]**2) 
    cos_q5 = R[1, 2]
    
    sin_q6 = -R[1, 1]
    cos_q6 = R[1, 0] 
    
    q4 = atan2(sin_q4, cos_q4)
    q5 = atan2(sin_q5, cos_q5)
    q6 = atan2(sin_q6, cos_q6)
    
    return q4, q5, q6

def get_angles(x, y, z, roll, pitch, yaw):
    from sympy import Matrix, symbols
    gripper_point = x, y, z
    q1, q2, q3, q4, q5, q6 = symbols('q1:7')
    alpha, beta, gamma = symbols('alpha beta gamma', real=True)
    
    # Rotation matrix R0u do gripper baseada em roll, pitch, yaw
    R0u = Matrix([
        [cos(alpha)*cos(beta), -sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha), sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)],
        [sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma) - sin(gamma)*cos(alpha)],
        [-sin(beta), sin(gamma)*cos(beta), cos(beta)*cos(gamma)]
    ])
    
    # Matriz RguT para ajustar orientação entre frames
    RguT_eval = Matrix([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
    
    # Avalia a rotação com os valores reais
    R0u_eval = R0u.evalf(subs = {alpha: yaw, beta: pitch, gamma: roll})
    R0g_eval = R0u_eval * RguT_eval
    
    wrist_center = get_wrist_center(gripper_point, R0g_eval, dg=0.303)
    j1, j2, j3 = get_first_three_angles(wrist_center)
    
    # R03 transposta
    R03T = Matrix([
        [sin(j2 + j3)*cos(j1), sin(j1)*sin(j2 + j3),  cos(j2 + j3)],
        [cos(j1)*cos(j2 + j3), sin(j1)*cos(j2 + j3), -sin(j2 + j3)],
        [-sin(j1), cos(j1), 0]
    ])
    R36_eval = R03T.evalf() * R0g_eval
    
    j4, j5, j6 = get_last_three_angles(R36_eval)
    
    return j1.evalf(), j2.evalf(), j3.evalf(), j4.evalf(), j5.evalf(), j6.evalf()

# --- Funções para interpolação linear suave em espaço cartesiano ---

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

# --- Funções para validação limites ---

JOINT_LIMITS = [
    (-pi, pi),            # q1
    (-pi/2, pi/2),        # q2
    (-pi, pi),            # q3
    (-pi, pi),            # q4
    (-pi, pi),            # q5
    (-pi, pi)             # q6
]

def dentro_dos_limites(angles):
    for i, angle in enumerate(angles):
        low, high = JOINT_LIMITS[i]
        if angle < low or angle > high:
            return False
    return True

# --- Funções para plotagem 3D ---

def cria_plot():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(0, 3)
    ax.set_ylim(0, 3)
    ax.set_zlim(0, 3)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.ion()
    plt.show()
    return fig, ax

def atualiza_plot(ax, X, Y, Z):
    ax.cla()
    ax.plot(X, Y, Z, '-o', color='blue')
    ax.set_xlim(0, 3)
    ax.set_ylim(0, 3)
    ax.set_zlim(0, 3)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.draw()
    plt.pause(0.05)

# --- Programa principal ---

def main():
    print("Controle do Braço Robótico 6-DOF\n")
    
    # Estado inicial da pose (posição + orientação) do end-effector
    current_pose = [0.5, 1.0, 1.0, 0.0, 0.0, 0.0]  # x,y,z, roll, pitch, yaw em rad
    
    fig, ax = cria_plot()
    
    while True:
        try:
            entrada = input("Digite x y z roll pitch yaw (em graus) ou 'sair': ")
            if entrada.strip().lower() == 'sair':
                break
            
            valores = list(map(float, entrada.strip().split()))
            if len(valores) != 6:
                print("Por favor, insira 6 valores: x y z roll pitch yaw")
                continue
            
            # Converte graus para radianos
            target_pose = [math.radians(v) if i >= 3 else v for i, v in enumerate(valores)]
            
            # Calcula IK para o alvo final (última pose)
            q_final = get_angles(*target_pose)
            if not dentro_dos_limites(q_final):
                print("[ERRO] Posição fora dos limites das juntas.")
                continue
            
            # Gera trajetória suave de 50 passos
            traj = lerp_pose(current_pose, target_pose, steps=50)
            
            print(f"Iniciando movimento para {valores[:3]} e orientação {valores[3:]} (em graus)")

            for pose in traj:
                try:
                    q = get_angles(*pose)
                    if not dentro_dos_limites(q):
                        print("[Aviso] Pose intermediária fora dos limites, ignorando passo.")
                        continue
                    X, Y, Z = forward_kin(*q)
                    atualiza_plot(ax, X, Y, Z)
                    print(f"Pose cartesiana: x={pose[0]:.3f}, y={pose[1]:.3f}, z={pose[2]:.3f}, roll={math.degrees(pose[3]):.1f}, pitch={math.degrees(pose[4]):.1f}, yaw={math.degrees(pose[5]):.1f}")
                    print(f"Juntas: {[round(float(ang), 3) for ang in q]}")
                except Exception as e:
                    print(f"[Erro] Durante interpolação: {e}")
                    continue

            current_pose = target_pose  # Atualiza pose atual após execução
            print("Movimento concluído.\n")

        except Exception as e:
            print(f"[Erro] Entrada inválida: {e}")

if __name__ == "__main__":
    main()
