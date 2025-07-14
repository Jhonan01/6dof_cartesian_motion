import math
from kinematics import get_angles, forward_kin
from trajectory import lerp_pose, dentro_dos_limites
from visualization import cria_plot, atualiza_plot

def main():
    print("Controle do Braço Robótico 6-DOF\n")
    
    current_pose = [0.5, 1.0, 1.0, 0.0, 0.0, 0.0]  # x,y,z,roll,pitch,yaw rad
    
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
            
            target_pose = [math.radians(v) if i >= 3 else v for i, v in enumerate(valores)]
            
            q_final = get_angles(*target_pose)
            if not dentro_dos_limites(q_final):
                print("[ERRO] Posição fora dos limites das juntas.")
                continue
            
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

            current_pose = target_pose
            print("Movimento concluído.\n")

        except Exception as e:
            print(f"[Erro] Entrada inválida: {e}")

if __name__ == "__main__":
    main()
