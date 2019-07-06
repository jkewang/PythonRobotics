"""
  D* Lite is a improvement of Dynamic A* , is widely used in dynamic environment
  Author: Jinkge Wang
  Date: 2019/7/5
"""

import matplotlib.pyplot as plt
import math

show_animation = True

class D_Lite_planner(object):

    def __init__(self, ox, oy, reso, rr, start, goal):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]
        """
        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()
        self.start = start
        self.now_pose = self.start
        self.goal = goal

        self.Inf = 10000
        self.queue = []
        self.km = 0

    class Node:
        def __init__(self, x, y, g, rhs, key, succ, pred):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.g = g
            self.rhs = rhs
            self.key = key
            self.succ = succ
            self.pred = pred

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.g) + "," + str(self.rhs)

    def planning(self):

        self.InitAllNodes()

        rx, ry = self.calc_final_path()
        return rx, ry

    def InitAllNodes(self):
        self.Node_list = []
        for y in range(int(self.ywidth)):
            for x in range((int(self.xwidth))):
                node = self.Node(self.calc_xyindex(x, self.minx), self.calc_xyindex(y, self.miny),
                                 self.Inf, self.Inf, [self.Inf, self.Inf], None, None)
                self.Node_list.append(node)

        self.goal_node = self.Node_list[self.calc_node_list_index(self.goal[0], self.goal[1])]
        self.goal_node.rhs = 0
        self.queue.append(self.goal_node)

        self.start_node = self.Node_list[self.calc_node_list_index(self.start[0], self.start[1])]

    def ComputeShortestPath(self):

        while(1):
            c_id = min(self.queue, key=lambda o: self.queue[o].key)
            current = self.queue[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.minx),
                         self.calc_grid_position(current.y, self.miny), "xc")
                plt.pause(0.001)

            if current.x == self.start.x and current.y == self.start.y:
                print("Find goal")
                self.start_node.pred = current.pred
                self.start_node.key = current.key
                break

            self.open_list.remove(current)
            stat = self.check_node_stat(current)
            if stat == 0:
                pass
            else:
                current.g = current.rhs

            for i, _ in enumerate(self.motion):
                n_id = self.Node_list[self.calc_grid_index(current)]
                if self.check_obs(n_id):
                    pass
                else:
                    pass



    def UpdateVertex(self):
        pass

    def MotionOneStep(self):
        pass

    def calc_final_path(self):
        rx = []
        ry = []

        return rx,ry

    def check_node_stat(self, node):
        if node.g == node.rhs or (node.g > 9000 and node.rhs > 9000):
            #consistent
            return 0
        elif node.g > node.rhs:
            #under consistent
            return 1
        else:
            #over consistent
            return 2

    def check_obs(self, node):
        return self.obmap[node.x][node.y]

    def calc_xyindex(self, position, min_pos):
        return round((position - min_pos) / self.reso)

    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def calc_node_list_index(self, x, y):
        x, y = self.calc_xyindex(x, self.minx), self.calc_xyindex(y, self.miny)
        index = (y - self.miny) * self.xwidth + (x - self.minx)
        return index

    def calc_grid_position(self, index, minp):
        """
        calc grid position

        :param index:
        :param minp:
        :return:
        """
        pos = index * self.reso + minp
        return pos

    def calc_obstacle_map(self, ox, oy):

        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))
        print("minx:", self.minx)
        print("miny:", self.miny)
        print("maxx:", self.maxx)
        print("maxy:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.reso)
        self.ywidth = round((self.maxy - self.miny) / self.reso)
        print("xwidth:", self.xwidth)
        print("ywidth:", self.ywidth)

        # obstacle map generation
        self.obmap = [[False for i in range(int(self.ywidth))]
                      for i in range(int(self.xwidth))]
        for ix in range(int(self.xwidth)):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(int(self.ywidth)):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.sqrt((iox - x) ** 2 + (ioy - y) ** 2)
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
                        break


    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    # set obstable positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    d_lite = D_Lite_planner(ox, oy, grid_size, robot_radius, [10, 10], [50, 50])
    rx, ry = d_lite.planning()

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()
