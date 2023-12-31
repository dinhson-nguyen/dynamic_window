
import math
import numpy as np
from enum import Enum
import obstacle

m = obstacle.RandomMap()
ob = []
riskinside = []
static_ob= []
static_risk = []
for z in m.obstacle_point:
    ob.append([z.x+0.5,z.y+0.5])
for z in m.risk_inside:
    riskinside.append([z.x,z.y])
for z in m.staticObs:
    static_ob.append([z.x+0.5,z.y+0.5])
for z in m.static_risk_inside:
    static_risk.append([z.x,z.y])
class RobotType(Enum):
    circle = 0
    rectangle = 1
class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 1.5  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 90.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.5  # [m/ss]
        self.max_delta_yaw_rate = 45.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.1  # [m/s]
        self.yaw_rate_resolution = 1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.5  # [s] Time tick for motion prediction
        self.predict_time = 2.0  # [s]
        self.to_goal_cost_gain = 0.3
        self.speed_cost_gain = 120
        self.obstacle_cost_gain = 0.2
        self.robot_stuck_flag_cons = 0.1  # constant to prevent robot stucked
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 2.0  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.75  # [m] for collision check
        self.robot_length = 1.0  # [m] for collision check
        # obstacles [x(m) y(m), ....]
        self.riskinside = np.array(riskinside)
        self.block = m.block
        self.ob = np.array(ob)
        self.static_ob = np.array(static_ob)
        self.static_risk = np.array(static_risk)
        self.risk_cost = 0.2
    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value