import numpy as np

import Point


class RandomMap:
    def __init__(self, size=25):
        self.size = size
        # self.obstacle = size//8
        self.obstacle = 2
        self.GenerateObstacle()
        self.staticObstacle()
        self.risk()

    def GenerateObstacle(self):
        self.obstacle_point = []
        self.block = []
        z = np.random.randint(4, 6)
        # for t in range(1, z + 1):
        #     for g in range(1, z + 1):
        #         self.obstacle_point.append(Point.Point(self.size // 2 + t, self.size // 2 + g))

        for i in range(self.obstacle ):
            x = np.random.randint(2, self.size - 5)
            y = np.random.randint(2, self.size - 5)
            self.obstacle_point.append(Point.Point(x, y))

            # if (np.random.rand() > 0.5): # Random boolean
            if True:  # Random boolean
                z = np.random.randint(2, 3)
                for t in range(0, z + 1):
                    for g in range(0, z + 1):
                        self.obstacle_point.append(Point.Point(x + g, y + t))
                self.block.append([x,y,z])
        # return self.obstacle_point
    def staticObstacle(self):
        self.staticObs = []
        z = np.random.randint(4, 6)
        # for t in range(1, z + 1):
        #     for g in range(1, z + 1):
        #         self.obstacle_point.append(Point.Point(self.size // 2 + t, self.size // 2 + g))

        for i in range(self.obstacle ):
            x = np.random.randint(2, self.size - 5)
            y = np.random.randint(2, self.size - 5)
            self.obstacle_point.append(Point.Point(x, y))

            # if (np.random.rand() > 0.5): # Random boolean
            if True:  # Random boolean
                z = np.random.randint(3, 5)
                for t in range(0, z + 1):
                    for g in range(0, z + 1):
                        self.obstacle_point.append(Point.Point(x + g, y + t))


    def risk(self):
        self.risk_inside = []
        self.risk_outside = []
        for t in range(self.size):
            for h in range(self.size):
                if self.IsObstacle(t, h) == True:
                    for m in range(-1, 2, 1):
                        for n in range(-1, 2, 1):
                            if self.IsObstacle(t + m, h + n) == False:
                                self.risk_inside.append(Point.Point(t + m, h + n))

        for t in range(self.size):
            for h in range(self.size):
                if self.Is_risk_inside(t, h):

                    for m in range(-1, 2, 1):
                        for n in range(-1, 2, 1):
                            if self.Is_risk_inside(t + m, h + n) == False:
                                if self.IsObstacle(t + m, h + n) == False:
                                    self.risk_outside.append(Point.Point(t + m, h + n))
    def static_risk(self):
        self.static_risk_inside = []
        self.static_risk_outside = []
        for t in range(self.size):
            for h in range(self.size):
                if self.IsObstacle(t, h) == True:
                    for m in range(-1, 2, 1):
                        for n in range(-1, 2, 1):
                            if self.IsObstacle(t + m, h + n) == False:
                                self.risk_inside.append(Point.Point(t + m, h + n))

        for t in range(self.size):
            for h in range(self.size):
                if self.Is_risk_inside(t, h):

                    for m in range(-1, 2, 1):
                        for n in range(-1, 2, 1):
                            if self.Is_risk_inside(t + m, h + n) == False:
                                if self.IsObstacle(t + m, h + n) == False:
                                    self.risk_outside.append(Point.Point(t + m, h + n))

    def Is_risk_inside(self, i, j):
        for p in self.risk_inside:
            if i == p.x and j == p.y:
                return True
        return False

    def Is_risk_outside(self, i, j):
        for p in self.risk_outside:
            if i == p.x and j == p.y:
                return True
        return False

    def IsObstacle(self, i, j):
        for p in self.obstacle_point:
            if i == p.x and j == p.y:
                return True
        return False
    def Is_static_Obstacle(self, i, j):
        for p in self.staticObs:
            if i == p.x and j == p.y:
                return True
        return False
