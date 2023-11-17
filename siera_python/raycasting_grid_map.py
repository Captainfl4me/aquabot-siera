"""
Raycasting grid map library

orignal author: Atsushi Sakai (@Atsushi_twi)
Reworked and fix: Nicolas THIERRY

"""

import math
import numpy as np
import matplotlib.pyplot as plt

EXTEND_AREA = 10.0


def calc_grid_map_config(max_range: float, xyreso: float):
    minx = -max_range - EXTEND_AREA / 2.0
    maxx = max_range + EXTEND_AREA / 2.0
    miny = -max_range - EXTEND_AREA / 2.0
    maxy = max_range + EXTEND_AREA / 2.0
    xw = int(round((maxx - minx) / xyreso))
    yw = int(round((maxy - miny) / xyreso))

    return minx, miny, maxx, maxy, xw, yw


class precastDB:
    def __init__(self):
        self.px = 0.0
        self.py = 0.0
        self.d = 0.0
        self.angle = 0.0
        self.ix = 0
        self.iy = 0

    def __str__(self):
        return str(self.px) + "," + str(self.py) + "," + str(self.d) + "," + str(self.angle)


def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0

    return angle


def precasting(minx, miny, xw, yw, xyreso, yawreso):

    precast = [[] for i in range(int(round((math.pi * 2.0) / yawreso)) + 1)]

    for ix in range(xw):
        for iy in range(yw):
            px = ix * xyreso + minx
            py = iy * xyreso + miny

            d = math.hypot(px, py)
            angle = atan_zero_to_twopi(py, px)
            angleid = int(math.floor(angle / yawreso))

            pc = precastDB()

            pc.px = px
            pc.py = py
            pc.d = d
            pc.ix = ix
            pc.iy = iy
            pc.angle = angle

            precast[angleid].append(pc)

    return precast


def generate_ray_casting_grid_map(ox, oy, xyreso: float, yawreso: float, max_range: float=130.0, robot_radius: float=0.0):

    minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(max_range, xyreso)

    pmap = np.zeros((xw, yw), dtype=int)

    precast = precasting(minx, miny, xw, yw, xyreso, yawreso)

    for (x, y) in zip(ox, oy):

        d = math.hypot(x, y)
        angle = atan_zero_to_twopi(y, x)
        angleid = int(math.floor(angle / yawreso))

        gridlist = precast[angleid]

        ix = int(round((x - minx) / xyreso))
        iy = int(round((y - miny) / xyreso))

        robot_radius = 4
        for grid in gridlist:
            if grid.d > d:
                pmap[grid.ix][grid.iy] = -1
            if grid.d < d and grid.d > d - robot_radius:
                pmap[grid.ix][grid.iy] = 100

        pmap[ix][iy] = 100

    return pmap, minx, maxx, miny, maxy, xyreso


def draw_heatmap(data, minx, maxx, miny, maxy, xyreso):
    x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                    slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
    plt.pcolor(x, y, data, vmax=1.0, cmap=plt.cm.Blues)
    plt.axis("equal")


def main():
    print(__file__ + " start!!")

    xyreso = 0.5  # x-y grid resolution [m]
    yawreso = np.deg2rad(10.0)  # yaw angle resolution [rad]

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

    pmap, minx, maxx, miny, maxy, xyreso = generate_ray_casting_grid_map(
        ox, oy, xyreso, yawreso)
    print(np.shape(pmap))

    plt.cla()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
    draw_heatmap(pmap, minx, maxx, miny, maxy, xyreso)
    plt.plot(ox, oy, "xr")
    plt.plot(0.0, 0.0, "ob")
    plt.show()


if __name__ == '__main__':
    main()
