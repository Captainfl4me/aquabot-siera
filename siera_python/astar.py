"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
reworked by: Nicolas THIERRY

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import math

import matplotlib.pyplot as plt
import numpy as np
from siera_python.raycasting_grid_map import generate_ray_casting_grid_map, draw_heatmap


class AStarPlanner:

    def __init__(self, pmap: list[int, int], min_x: float, max_x: float, min_y: float, max_y: float, resolution):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        """

        self.resolution = resolution

        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        self.motion = self.get_motion_model()
        self.obstacle_map = pmap

    def update_map(self, pmap: list[int, int]):
        self.obstacle_map = pmap

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node
        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if False:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)


            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y] == 100:
            return False

        return True

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


    ox = [  -2.93217606,   -3.25401621,   -3.57484402,   -3.89469084,
         -4.21396058,   -4.53217398,   -4.8561529 ,   -5.17372125,
         -5.49078465,   -5.80639674,   -6.1217662 ,   -6.43610273,
         -6.75033096,   -7.06918787,   -7.38251886,   -7.69477009,
         -8.00522111,   -8.31867224,   -8.62983844,   -8.93710959,
         -9.25682164,   -9.56501486,   -9.87396012,  -10.18084737,
        -10.48743703,  -10.79567819,  -11.10105051,  -11.40872129,
        -11.72162078,  -12.02912616,  -12.33395065,  -12.63554113,
        -12.93874489,  -13.24153322,  -13.54472629,  -13.8469366 ,
        -14.15928548,  -14.46218973,  -14.76208767,  -15.0657264 ,
        -15.36476873,  -15.66629365,  -15.96510361,  -16.26362286,
        -16.57341428,  -16.8713478 ,  -17.17124463,  -17.46390683,
        -17.76480243,  -18.06145875,  -18.35889697,  -18.65400843,
        -18.95393349,  -19.26127291,  -19.55529728,  -19.85045208,
        -20.14591495,  -20.43522441,  -20.73651075,  -21.03120842,
        -21.32895257,  -21.62046557,  -21.91607085,  -22.21426087,
        -22.51843432,  -22.80787683,  -23.1056941 ,  -23.47762557,
        -23.86210385,  -24.25595369,  -24.64880692,  -25.04729294,
        -25.45050119,  -25.85441154,  -26.25543283,  -27.45450087,
        -27.77019618,  -29.23315141,  -30.8157353 ,  -32.52799424,
        -47.62488981,  -47.72038254,  -47.8473347 ,  -47.94928111,
        -48.05854798,  -48.16548389,  -48.27326402,  -48.22007599,
        -48.32162095,  -48.43152736,  -48.54154543,  -48.647718  ,
        -48.76646441,  -48.70563119,  -48.81754699,  -48.9235776 ,
        -49.10986285,  -49.3178922 ,  -49.46064397,  -49.67012483,
        -49.87452025,  -50.06635339,  -50.27839676,  -50.44266502,
        -50.62488489,  -50.84895067,  -51.03161515,  -51.20321888,
        -51.40597736,  -51.60652248,  -51.76572014,  -51.9680457 ,
        -52.16949412,  -52.37733455,  -52.51848073,  -52.72102507,
        -52.92552063,  -53.08810096,  -53.28917759,  -53.48844602,
        -53.6399423 ,  -53.83694745,  -54.04071442,  -54.2163938 ,
        -54.39174016,  -54.60251057,  -54.76618707,  -54.94679087,
        -55.11153616,  -55.3176875 ,  -55.5116394 ,  -55.67015067,
        -55.86526392,  -56.03195591,  -56.22663868,  -56.39557984,
        -56.59076805,  -56.77377149,  -56.9400389 ,  -57.13791472,
        -57.35124173,  -57.63488646,  -57.98797945,  -58.27913763,
        -58.62508539,  -58.90369285,  -59.48459515,  -59.85045038,
        -60.13407122,  -60.48614987,  -60.78153566,  -61.12017614,
        -61.40439339,  -61.76216844,  -62.11590494,  -62.40182608,
        -62.77107649,  -63.12997645,  -63.41652356,  -63.77175634,
        -64.13202311,  -64.43244892,  -64.77822371,  -65.52159668,
        -66.06654533,  -66.93482774,  -67.81836227,  -68.71919571,
        -69.29511656,  -70.24446883,  -70.84095844,  -71.83679421,
        -72.84602849,  -73.47716887,  -74.53310205,  -75.63005824,
        -76.29552988,  -77.44858076,  -78.14614989,  -78.89024073,
        -79.6248131 , -126.82861613, -126.53998994, -126.48072634,
       -126.21380351, -125.94642776, -125.75941877, -126.82417795,
       -127.90405821, -123.63943023, -123.7703399 , -123.50012957,
       -123.22779015, -123.40193552, -123.72162377, -123.99456768,
       -123.93855741, -124.23502066, -124.5300694 , -124.83093961,
       -125.14029922, -125.45859914, -125.76476694, -126.07285112,
       -126.37828851, -123.2093657 , -122.70166208, -122.67090901,
       -122.54977412, -122.66447133]
    oy = [ -97.14191423,  -97.0169852 ,  -96.88536467,  -96.74933053,
        -96.61900297,  -96.48280696,  -96.47731006,  -96.35107789,
        -96.22882922,  -96.0941231 ,  -95.9677716 ,  -95.83664917,
        -95.71445349,  -95.66397546,  -95.54431738,  -95.41920875,
        -95.28065119,  -95.18505422,  -95.06932545,  -94.91846079,
        -94.9035292 ,  -94.77338482,  -94.65681874,  -94.52639743,
        -94.39917668,  -94.29183867,  -94.16426547,  -94.06070694,
        -94.0026442 ,  -93.90347065,  -93.78696123,  -93.65028225,
        -93.52992012,  -93.41033849,  -93.29708737,  -93.18034853,
        -93.13346647,  -93.02580721,  -92.90178574,  -92.80396347,
        -92.68037975,  -92.57440019,  -92.45483329,  -92.33620506,
        -92.2828092 ,  -92.16455407,  -92.05911678,  -91.91734972,
        -91.821127  ,  -91.70467483,  -91.59406609,  -91.47372549,
        -91.3785378 ,  -91.31955147,  -91.19828623,  -91.08400062,
        -90.97262009,  -90.83525814,  -90.75243513,  -90.64162019,
        -90.5449471 ,  -90.42295832,  -90.31928054,  -90.22697197,
        -90.15912748,  -90.0329181 ,  -89.94068966,  -90.13339951,
        -90.36626399,  -90.62575757,  -90.87254771,  -91.13120322,
        -91.39794276,  -91.65815274,  -91.89946048,  -94.89079553,
        -94.79035619,  -98.55802289, -102.63017951, -107.02857912,
        -85.21621283,  -84.71882831,  -84.28210857,  -83.80596557,
        -83.34738669,  -82.88937016,  -82.43733755,  -81.7169459 ,
        -81.26527253,  -80.83194237,  -80.40288697,  -79.97151557,
        -79.56468001,  -78.87067991,  -78.46223249,  -78.04802846,
        -77.76460644,  -77.51711247,  -77.16897899,  -76.92686358,
        -76.67803374,  -76.41121992,  -76.1764634 ,  -75.87080662,
        -75.59392688,  -75.38062405,  -75.10700267,  -74.81868752,
        -74.57723628,  -74.33358141,  -74.031798  ,  -73.79307277,
        -73.5540229 ,  -73.32482635,  -73.00367533,  -72.76930799,
        -72.53842222,  -72.25123841,  -72.01760412,  -71.78230865,
        -71.48412658,  -71.24781242,  -71.02115738,  -70.75847318,
        -70.49647334,  -70.28116575,  -70.00610241,  -69.75378273,
        -69.48240216,  -69.26405127,  -69.03097894,  -68.75477636,
        -68.52468948,  -68.26056034,  -68.0313343 ,  -67.77172613,
        -67.54440066,  -67.30313914,  -67.0428601 ,  -66.82048219,
        -66.61646704,  -66.49373583,  -66.44963798,  -66.33302513,
        -66.27729509,  -66.14405721,  -66.34712714,  -66.30655875,
        -66.17335319,  -66.11405953,  -65.99129441,  -65.91409359,
        -65.7768878 ,  -65.71703301,  -65.65094479,  -65.51176453,
        -65.45854701,  -65.39251148,  -65.25008991,  -65.17693186,
        -65.10701005,  -64.97481032,  -64.88691884,  -65.19290848,
        -65.29578569,  -65.71176683,  -66.13408853,  -66.56450872,
        -66.67350079,  -67.1348425 ,  -67.25190591,  -67.74081561,
        -68.23253677,  -68.36265672,  -68.88032378,  -69.42539135,
        -69.56635266,  -70.14359646,  -70.29988748,  -70.49214426,
        -70.66982982,   16.46302623,   16.85716497,   17.28105746,
         17.67585887,   18.0691755 ,   18.47288203,   19.06389445,
         19.66497124,   21.13667587,   21.58640961,   21.96620326,
         22.34425107,   22.8034374 ,   23.29176139,   23.77388486,
         24.19424437,   24.68481322,   25.17773944,   25.67451157,
         26.17576399,   26.68169725,   27.18786576,   27.69724413,
         28.20885009,   31.87267211,   32.1806453 ,   32.61255232,
         33.02067049,   33.49310683]

    # start and goal position
    sx = 0.0  # [m]
    sy = 0.0  # [m]
    gx = -120.0  # [m]
    gy = -90.0  # [m]
    robot_radius = 10.0  # [m]
    xyreso = 2  # x-y grid resolution [m]
    yawreso = np.deg2rad(1.0)  # yaw angle resolution [rad]

    pmap, minx, maxx, miny, maxy, xyreso = generate_ray_casting_grid_map(
        ox, oy, xyreso, yawreso, robot_radius=robot_radius)
    print(np.shape(pmap))
    a_star = AStarPlanner(pmap, minx, maxx, miny, maxy, xyreso)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    plt.cla()
    draw_heatmap(pmap, minx, maxx, miny, maxy, xyreso)
    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "og")
    plt.plot(gx, gy, "xb")
    plt.plot(rx, ry, "-r")
    plt.grid(True)
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()