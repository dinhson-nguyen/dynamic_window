
from dynamic_window import *
from matplotlib.patches import Rectangle



show_animation = True
ax = plt.gca()
config = Config()
def main(gx=20.0, gy=20.0, robot_type=RobotType.circle):
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])

    # input [forward speed, yaw_rate]

    config.robot_type = robot_type
    trajectory = np.array(x)
    ob = config.ob

    l=0
    while True:
        l +=1
        u, predicted_trajectory = dwa_control(x, config, goal, ob)
        x = motion(x, u, config.dt)  # simulate robot
        trajectory = np.vstack((trajectory, x))  # store state history


        if show_animation:
            plt.xlim(0, 20)
            plt.ylim(0, 20)
            plt.cla()
            plt.xlim(0, 20)
            plt.ylim(0, 20)
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            # plt.plot(ob[:, 0], ob[:, 1], 's', color='black', markersize=5)
            # plt.plot(config.ob[:, 0], config.ob[:, 1], "ok")
            for item in ob:
                rec = Rectangle((item[0], item[1]), width=1, height=1, color='black')
                ax.add_patch(rec)
            plot_robot(x[0], x[1], x[2], config)
            plot_arrow(x[0], x[1], x[2])
            plt.axis('equal')
            plt.axis('on')
            # plt.tight_layout()

            plt.grid(color='green', linestyle='--', linewidth=0.5)
            plt.pause(0.0001)

        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.robot_radius:
            print("Goal!!")
            break

    print(l)


    print("Done")
    if show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.pause(0.0001)
        plt.show()



if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)
    # main(robot_type=RobotType.circle)
