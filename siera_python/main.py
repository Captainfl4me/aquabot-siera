""" Main file for the Siera Python package. """
import time
from enum import Enum
from math import pi, atan2, asin, copysign, radians, cos, sin

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from ros_gz_interfaces.msg import ParamVec
from rcl_interfaces.msg import Parameter

from sensor_msgs.msg import Imu, LaserScan, NavSatFix


class TaskState(Enum):
    """ Enum for the state of the task. """
    INITIAL = "initial"
    READY = "ready"
    RUNNING = "running"
    FINISHED = "finished"


class TaskInfo:
    """ Class for storing the task info. """
    currentTask = None

    def __init__(self):
        self.name: str = ""
        self.score: float = 0.0
        self.timed_out: bool = False
        self.elapsed_time: float = 0.0
        self.running_time: float = 0.0
        self.ready_time: float = 0.0
        self.remaining_time: float = 0.0
        self.num_collisions: int = 0
        self.state: TaskState = TaskState.INITIAL

        TaskInfo.currentTask: TaskInfo = self

    def parse_from_paramvec(self, data_list: list[Parameter]) -> None:
        """
            Parses the data from the ParamVec message and stores it in the class.
        """
        for data in data_list:
            if data.name == "name":
                self.name = data.value.string_value
            elif data.name == "score":
                self.score = data.value.double_value
            elif data.name == "timed_out":
                self.timed_out = data.value.bool_value
            elif data.name == "elapsed_time":
                self.elapsed_time = data.value.double_value
            elif data.name == "running_time":
                self.running_time = data.value.double_value
            elif data.name == "ready_time":
                self.ready_time = data.value.double_value
            elif data.name == "remaining_time":
                self.remaining_time = data.value.double_value
            elif data.name == "num_collisions":
                self.num_collisions = data.value.integer_value
            elif data.name == "state":
                self.state = TaskState(data.value.string_value)

    def __str__(self) -> str:
        return f"""
        name: {self.name}
        score: {self.score}
        timed_out: {self.timed_out}
        elapsed_time: {self.elapsed_time}
        running_time: {self.running_time}
        ready_time: {self.ready_time}
        remaining_time: {self.remaining_time}
        num_collisions: {self.num_collisions}
        state: {self.state}\n
        """


class Quaternion:
    """ Class for storing a quaternion. """
    def __init__(self):
        self._x: float = 0.0
        self._y: float = 0.0
        self._z: float = 0.0
        self._w: float = 0.0

    @property
    def x(self) -> float:
        """ Returns the x value of the quaternion. """
        return self._x
    @property
    def y(self) -> float:
        """ Returns the y value of the quaternion. """
        return self._y
    @property
    def z(self) -> float:
        """ Returns the z value of the quaternion. """
        return self._z
    @property
    def w(self) -> float:
        """ Returns the w value of the quaternion. """
        return self._w

    @x.setter
    def x(self, value: float) -> None:
        self._x = value
    @y.setter
    def y(self, value: float) -> None:
        self._y = value
    @z.setter
    def z(self, value: float) -> None:
        self._z = value
    @w.setter
    def w(self, value: float) -> None:
        self._w = value

    def to_euler(self) -> tuple[float, float, float]:
        """ Returns the euler angles of the quaternion. """
        sinrp = 2.0 * (self._w * self._x + self._y * self._z)
        cosrp = 1.0 - 2.0 * (self._x * self._x + self._y * self._y)
        roll = atan2(sinrp, cosrp)

        sinp = 2.0 * (self._w * self._y - self._z * self._x)
        if abs(sinp) >= 1:
            pitch = copysign(pi / 2, sinp)
        else:
            pitch = asin(sinp)

        sinyaw = 2.0 * (self._w * self._z + self._x * self._y)
        cosyaw = 1.0 - 2.0 * (self._y * self._y + self._z * self._z)
        yaw = atan2(sinyaw, cosyaw)

        return (roll, pitch, yaw)


class IMU:
    """ Class for storing the IMU data. """
    def __init__(self):
        self._quaternion: Quaternion = Quaternion()
        self._roll: float = 0.0
        self._pitch: float = 0.0
        self._yaw: float = 0.0

    @property
    def quaternion(self) -> Quaternion:
        """ Returns the quaternion. """
        return self._quaternion
    @property
    def roll(self) -> float:
        """ Returns the roll angle in radians. """
        return self._roll
    @property
    def pitch(self) -> float:
        """ Returns the pitch angle in radians. """
        return self._pitch
    @property
    def yaw(self) -> float:
        """ Returns the yaw angle in radians. """
        return self._yaw

    def parse_from_imu_msg(self, data: Imu) -> None:
        """ Parses the data from the Imu message and stores it in the class. """
        self._quaternion.x = data.orientation.x
        self._quaternion.y = data.orientation.y
        self._quaternion.z = data.orientation.z
        self._quaternion.w = data.orientation.w

        self._roll, self._pitch, self._yaw = self._quaternion.to_euler()


class GPS:
    """ Class for storing the GPS data. """
    def __init__(self):
        self._latitude: float = 0.0
        self._longitude: float = 0.0
        self._altitude: float = 0.0

        self._reference_latitude_rad: float = -1.0
        self._reference_longitude_rad: float = -1.0
        self._reference_altitude: float = -1.0
        # Earth radius in meters
        self._earth_radius: float = 6371000.0

    @property
    def latitude(self) -> float:
        """ Returns the latitude in degrees. """
        return self._latitude
    @property
    def longitude(self) -> float:
        """ Returns the longitude in degrees. """
        return self._longitude
    @property
    def altitude(self) -> float:
        """ Returns the altitude in meters. """
        return self._altitude

    def parse_from_gps_msg(self, data: NavSatFix) -> None:
        """
        Parses the data from the NavSatFix message and stores it in the class.

        :param data: NavSatFix message data to be parsed and stored in the class.
        :type data: NavSatFix
        :return: None
        """
        if self._reference_latitude_rad == -1.0 or self._reference_longitude_rad == -1.0 or self._reference_altitude == -1.0:
            self._reference_latitude_rad = radians(data.latitude)
            self._reference_longitude_rad = radians(data.longitude)
            self._reference_altitude = data.altitude

        self._latitude = data.latitude
        self._longitude = data.longitude
        self._altitude = data.altitude

    def geodetic_to_cartesian(self):
        """
        Converts the geodetic coordinates to cartesian coordinates.
        Assumes that the reference coordinates have been set and earth is perfectly round.
        Returns the cartesian coordinates in meters.

        :return: A tuple of three floats representing the cartesian coordinates in meters.
        :raises ValueError: If the reference latitude has not been set.
        """
        if self._reference_latitude_rad == -1.0 or self._reference_longitude_rad == -1.0 or self._reference_altitude == -1.0:
            raise ValueError("GPS: reference latitude has not been set!!")

        latitude_rad = radians(self._latitude)
        longitude_rad = radians(self._longitude)

        x = (self._earth_radius + self._altitude - self._reference_altitude) * cos(latitude_rad - self._reference_latitude_rad) * cos(longitude_rad - self._reference_longitude_rad)
        y = (self._earth_radius + self._altitude - self._reference_altitude) * cos(latitude_rad - self._reference_longitude_rad) * sin(longitude_rad - self._reference_longitude_rad)
        z = (self._earth_radius + self._altitude - self._reference_altitude) * sin(latitude_rad - self._reference_latitude_rad)

        return x, y, z

    def __str__(self) -> str:
        return f"latitude: {self.latitude}\nlongitude: {self.longitude}\naltitude: {self.altitude}\n"


class Pinger:
    """ Class for storing the pinger data. """
    currentPinger = None

    def __init__(self):
        self._range: float = 0.0 # in meters
        self._bearing: float = 0.0 # in radians relative to boat (positive clockwise)

        Pinger.currentPinger: Pinger = self

    def parse_from_paramvec(self, data_list: list[Parameter]) -> None:
        """ Parses the data from the ParamVec message and stores it in the class. """
        for data in data_list:
            if data.name == "range":
                self._range = data.value.double_value
            elif data.name == "bearing":
                self._bearing = data.value.double_value

    @property
    def range(self) -> float:
        """ Returns the range of the pinger in meters. """
        return self._range

    @property
    def bearing(self) -> float:
        """ Returns the bearing of the pinger in radians. """
        return self._bearing

    def __str__(self) -> str:
        return f"range: {self.range}\nbearing: {self.bearing}\n"


class LidarScan:
    """ Class for storing the lidar scan data. """
    def __init__(self):
        self._ranges: list[float] = []
        self._angles: list[float] = []
        self._ox: list[float] = []
        self._oy: list[float] = []

    @property
    def ranges(self) -> list[float]:
        """ Returns the ranges of the lidar scan. """
        return self._ranges
    @property
    def angles(self) -> list[float]:
        """ Returns the angles of the lidar scan. """
        return self._angles

    def parse_from_lidar_msg(self, data: LaserScan) -> None:
        """ Parses the data from the LaserScan message and stores it in the class. """
        self._ranges = np.array(data.ranges)
        self._angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
        if np.size(self._ranges) != np.size(self._angles):
            raise ValueError("LidarScan: ranges and angles are not the same size!!")

        self._ox = np.sin(self._angles) * self._ranges
        self._oy = np.cos(self._angles) * self._ranges


class SieraNode(Node):
    """ Main node class. """
    def __init__(self):
        super().__init__('SieraNode')
        self.taskinfo: TaskInfo = TaskInfo()
        self.pinger: Pinger = Pinger()
        self.imu: IMU = IMU()
        self.lidar_scan: LidarScan = LidarScan()

        self.task_info_sub = self.create_subscription(
            ParamVec,
            '/vrx/task/info',
            self.taskinfo_callback,
            10)

        self.pinger_sub = self.create_subscription(
            ParamVec,
            '/wamv/sensors/acoustics/receiver/range_bearing',
            self.pinger_callback,
            10)

        self.imu_sub = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.imu_callback,
            10)

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/wamv/sensors/lidars/lidar_wamv_sensor/scan',
            self.lidar_callback,
            10)

        self.last_exec = 0
        self.create_timer(0.1, self.main_loop)
        self.get_logger().info('SieraNode has been started')

    def main_loop(self):
        """
            This is the main loop of the node. It is called every 0.1s. 
            Loop time should be less than 100ms.
        """
        start_exec = time.process_time()

        print(self.pinger)
        print(f"Yaw: {self.imu.yaw}")

        print(f"Loop time: {(time.process_time() - start_exec) * 1000}ms")

    def taskinfo_callback(self, msg: ParamVec):
        """ Callback for the task info message. """
        self.taskinfo.parse_from_paramvec(msg.params)

    def pinger_callback(self, msg: ParamVec):
        """ Callback for the pinger message. """
        self.pinger.parse_from_paramvec(msg.params)

    def imu_callback(self, msg: Imu):
        """ Callback for the imu message. """
        self.imu.parse_from_imu_msg(msg)

    def lidar_callback(self, msg: LaserScan):
        """ Callback for the lidar message. """
        self.lidar_scan.parse_from_lidar_msg(msg)

def main(args=None):
    """ Main function. """
    rclpy.init(args=args)
    try:
        siera_node = SieraNode()

        executor = SingleThreadedExecutor()
        executor.add_node(siera_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            siera_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
