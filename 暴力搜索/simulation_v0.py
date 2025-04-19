import numpy as np
import matplotlib.pyplot as plt
from Vehicle import vehicle
from reward import compute_reward
from planner_complete_search import Kplanner
import Env_road
import time


def is_near_target(vehicle, threshold=1.0):
    dx = vehicle.x - vehicle.target_state[0]
    dy = vehicle.y - vehicle.target_state[1]
    return np.hypot(dx, dy) <= threshold

def run_simulation():
    # 仿真参数
    dt = 0.5
    horizon = 2.0  # 为加快仿真，这里采用较短预测步数，可根据需要调整
    weights = [200,20,100,10,1]
    # target_state_ego = (2, 16,1,np.pi/2)
    target_state_ego = (-16, 2,1,np.pi)  
    target_state_other = (16, -2,1,0)
    # 初始化路面信息
    road = Env_road.EnvRoads()
    Planner = Kplanner(dt=dt, time=horizon, gamma=0.9)
    

    # 初始化自主车辆和对手车辆
    # 自主车辆：初始位置 (2, -16)，速度 4 m/s，向上行驶（theta = π/2）
    # ego_vehicle = vehicle(x=2, y=-16, v=4.0, theta=np.pi/2,target_state=target_state_ego,dt=dt)
    ego_vehicle = vehicle(x=16, y=2, v=4.0, theta=np.pi,target_state=target_state_ego,dt=dt)
    # 对手车辆：初始位置 (-2, 16)，速度 4 m/s，向下行驶（theta = -π/2）
    other_vehicle = vehicle(x=-2, y=16, v=4.0, theta=-np.pi/2, target_state=target_state_other, dt=dt)
    # other_vehicle = vehicle(x=-16, y=-2, v=6.0, theta=0, target_state=target_state_other, dt=dt)

    # 初始化自主车辆控制器
    action_set = list(ego_vehicle.action_map.keys())
    # controller = AutonomousVehicleController(dt,horizon,ego_vehicle, action_set,road,weights)

    sim_time = 10  # 仿真时长（秒）
    steps = int(sim_time / dt)

    ego_trajectory = []
    other_trajectory = []
    ego_v_trajectory = []
    other_v_trajectory = []

    for step in range(steps):
        start_time = time.time()  # 记录开始时间
        if is_near_target(ego_vehicle) and is_near_target(other_vehicle):
            print("Both vehicles reached their target points.")
            break    
        other_action =Planner.levelk_decision(other_vehicle, ego_vehicle,1, action_set, road, weights)[0][0]
        # other_action = "Brake"  # 对手车辆保持不变
    # 自主车辆根据当前状态和对手状态选择动作
        # chosen_action, best_level,best_pred = controller.select_action(other_vehicle)
        chosen_action=Planner.levelk_decision(ego_vehicle, other_vehicle,0, action_set, road, weights)[0][0]
        # chosen_action='Brake'
       
        if not is_near_target(other_vehicle):
            other_vehicle.Update(other_action)
            other_v_trajectory.append(other_vehicle.copy())
            other_trajectory.append(other_vehicle.get_state())
        if not is_near_target(ego_vehicle):
            ego_vehicle.Update(chosen_action)
            ego_v_trajectory.append(ego_vehicle.copy())
            ego_trajectory.append(ego_vehicle.get_state())
        # 对手车辆使用简单规则，例如保持不变（Maintain）
        
        print(f"Step {step+1}/{steps}: Ego Vehicle Action: {chosen_action}, Other Vehicle Action: {other_action}")

        # 保存状态轨迹（用于后续绘图分析）
        
        print(f"Step {step+1}/{steps}: Ego Vehicle State: {ego_vehicle.get_state()}, Other Vehicle State: {other_vehicle.get_state()}")
        end_time = time.time()    # 记录结束时间
        elapsed = end_time - start_time
        print(f"Step {step+1}/{steps} took {elapsed:.4f} seconds.")
        
        fig, ax = plt.subplots(figsize=(8, 8))
        road.draw_env(ax,[ego_vehicle, other_vehicle])  # 绘制路面环境
            # 绘制车辆状态信息
        ax.text(16, -16,  # 自主车辆信息
                f"V: {ego_vehicle.v:.2f}\nPos: ({ego_vehicle.x:.2f}, {ego_vehicle.y:.2f})\nTheta: {ego_vehicle.theta:.2f}",
                color='green', fontsize=10, ha='center')
        ax.text(16, 16,  # 对手车辆信息
                f"V: {other_vehicle.v:.2f}\nPos: ({other_vehicle.x:.2f}, {other_vehicle.y:.2f})\nTheta: {other_vehicle.theta:.2f}",
                color='blue', fontsize=10, ha='center')
        plt.show()

        # 根据对手实际动作更新自主车辆控制器中对对手模型的估计
        # controller.update_model_prob(other_action, best_pred)

        

    # 转换轨迹为 numpy 数组便于绘图
    ego_traj = np.array(ego_trajectory)
    other_traj = np.array(other_trajectory)

    # 绘制轨迹
    fig, ax = plt.subplots(figsize=(8, 8))
    road.draw_env(ax, [ego_vehicle, other_vehicle])  # 绘制路面环境
    ax.plot(ego_traj[:, 0], ego_traj[:, 1], 'ro-', label='Ego Vehicle')
    ax.plot(other_traj[:, 0], other_traj[:, 1], 'bx-', label='Other Vehicle')
    for i in range(len(ego_v_trajectory)):
        ego_vehicle = ego_v_trajectory[i]
        ego_vehicle.draw_vehicle( ax, color='red',show_safe=False)
    for i in range(len(other_v_trajectory)):
        other_vehicle = other_v_trajectory[i]
        other_vehicle.draw_vehicle( ax, color='blue',show_safe=False)
    ax.set_xlabel("X Position")   # 注意这里是 set_xlabel
    ax.set_ylabel("Y Position")
    ax.legend()
    plt.show()
    
    

if __name__ == '__main__':
    run_simulation()
