import math
from kinematics import get_angles, forward_kin
from trajectory import lerp_pose, inside_of_limits
from visualization import create_plot, update_plot

def main():
    print("Robot arm controls 6-DOF\n")
    
    current_pose = [0.5, 1.0, 1.0, 0.0, 0.0, 0.0]  # x,y,z,roll,pitch,yaw rad
    
    fig, ax = create_plot()
    
    while True:
        try:
            entrada = input("Enter x y z roll pitch yaw (in degrees) or 'exit': ")
            if entrada.strip().lower() == 'exit':
                break
            
            valores = list(map(float, entrada.strip().split()))
            if len(valores) != 6:
                print("Please, enter 6 values: x y z roll pitch yaw")
                continue
            
            target_pose = [math.radians(v) if i >= 3 else v for i, v in enumerate(valores)]
            
            q_final = get_angles(*target_pose)
            if not inside_of_limits(q_final):
                print("[ERROR] Joint limits out of range.")
                continue
            
            traj = lerp_pose(current_pose, target_pose, steps=50)

            print(f"Starting movement to {valores[:3]} and orientation {valores[3:]} (in degrees)")

            for pose in traj:
                try:
                    q = get_angles(*pose)
                    if not inside_of_limits(q):
                        print("[WARNING] Intermediate pose out of limits, skipping step.")
                        continue
                    X, Y, Z = forward_kin(*q)
                    update_plot(ax, X, Y, Z)
                    print(f"Cartesian pose: x={pose[0]:.3f}, y={pose[1]:.3f}, z={pose[2]:.3f}, roll={math.degrees(pose[3]):.1f}, pitch={math.degrees(pose[4]):.1f}, yaw={math.degrees(pose[5]):.1f}")
                    print(f"Joints: {[round(float(ang), 3) for ang in q]}")
                except Exception as e:
                    print(f"[ERROR] During interpolation: {e}")
                    continue

            current_pose = target_pose
            print("Movement completed.\n")

        except Exception as e:
            print(f"[ERROR] Invalid input: {e}")

if __name__ == "__main__":
    main()
