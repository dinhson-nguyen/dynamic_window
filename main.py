import time
import numpy as np
from dynamic_window import *
from matplotlib.patches import Rectangle
import threading
import queue

data_queue = queue.Queue()
data_queue2 = queue.Queue()
show_animation = True
config = Config()
def producer(data_queue,data_queue2):
    ob = config.ob
    riskinside = config.riskinside
    n =0
    while True:
        n= n+0.1
        m = []
        riskinside_1= []
        t = [0.5,0.5]
        # Thực hiện các tác vụ và tạo dữ liệu
        for item in ob:
            m.append([item[0] + t[0]*n,item[1] - t[1]*n])
        for item in riskinside:
            riskinside_1.append([item[0] + t[0]*n,item[1] - t[1]*n])
        # Đưa dữ liệu vào hàng đợi
        data_queue.put(np.array(m))
        data_queue2.put(np.array(riskinside_1))
        # print(m)
        time.sleep(0.1)
        # print("pass")
def consumer(data_queue,data_queue2):
    fig, ax = plt.subplots()
    ax.set_xlim(-1, 25)
    ax.set_ylim(-1, 25)
    print(__file__ + " start!!")
    gx = 20.0
    gy = 20.0
    robot_type = RobotType.rectangle
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.2, 0.5])
    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])

    # input [forward speed, yaw_rate]
    config.robot_type = robot_type
    trajectory = np.array(x)
    l=0

    while True:
        m = data_queue.get()
        t = [0.5, 0.5]
        ob=[]
        for item in m:
            ob.append([item[0] + t[0],item[1] - t[1]])

        ob = np.concatenate((config.static_ob,np.array(ob)))
        # ob= np.array(ob)
        # riskinside_1 = np.concatenate((data_queue2.get(),config.static_risk))
        riskinside_1 = []
        for item in data_queue2.get():
            riskinside_1.append([item[0] + t[0],item[1] - t[1]])
        riskinside_2 = config.static_risk
        l +=1
        u, predicted_trajectory,remove_traject = dwa_control(x, config, goal, ob)
        # print(len(remove_traject))
        x = motion(x, u, config.dt)  # simulate robot
        trajectory = np.vstack((trajectory, x))  # store state history
        time.sleep(0.05)
        x_limits = ax.get_xlim()
        y_limits = ax.get_ylim()


        if show_animation:
            ax.clear()
            print(riskinside_1)
            ax.axis('equal')
            ax.axis('on')
            # for stopping simulation with the esc key.

            for item in remove_traject:
                ax.plot(item[:, 0], item[:, 1], linestyle='dashed',color='palegreen')
            ax.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            ax.plot(goal[0], goal[1], "xb")

            for item in ob:
                rec = Rectangle((item[0], item[1]), width=1, height=1, color='black')
                ax.add_patch(rec)
            for item in riskinside_1:
                rec = Rectangle((item[0], item[1]), width=1, height=1, color='gray')
                ax.add_patch(rec)
            for item in riskinside_2:
                rec = Rectangle((item[0], item[1]), width=1, height=1, color='gray')
                ax.add_patch(rec)

            plot_robot(x[0], x[1], x[2], config)
            plot_arrow(x[0], x[1], x[2])
            plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
            for item in config.block:
                rec = Rectangle((item[0], item[1]), width=item[2]+1, height=item[2]+1, edgecolor='darkblue',
                           fill=False)
                ax.add_patch(rec)
            ax.grid(color='green', linestyle='--', linewidth=0.5)
            ax.set_xlim(x_limits)
            ax.set_ylim(y_limits)
            plt.pause(0.0001)

        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.robot_radius:
            print("Goal!!")
            break


    print("Done")
    if show_animation:
        
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.pause(0.0001)

        plt.show()



