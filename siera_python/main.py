import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from enum import Enum
import time
from math import cos, sin, radians, pi, atan2, asin, copysign

import numpy as np
import matplotlib.pyplot as plt

from ros_gz_interfaces.msg import ParamVec
from rcl_interfaces.msg import Parameter

from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan


class TaskState(Enum):
    Initial = "initial"
    Ready = "ready"
    Running = "running"
    Finished = "finished"


class TaskInfo:
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
        self.state: TaskState = TaskState.Initial

        TaskInfo.currentTask: TaskInfo = self
    
    def parseFromParamVec(self, data_list: list[Parameter]) -> None:
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
    def __init__(self):
        self._x: float = 0.0
        self._y: float = 0.0
        self._z: float = 0.0
        self._w: float = 0.0
    
    @property
    def x(self) -> float:
        return self._x
    @property
    def y(self) -> float:
        return self._y
    @property
    def z(self) -> float:
        return self._z
    @property
    def w(self) -> float:
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

    def toEuler(self) -> tuple[float, float, float]:
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
    def __init__(self):
        self._quaternion: Quaternion = Quaternion()
        self._roll: float = 0.0
        self._pitch: float = 0.0
        self._yaw: float = 0.0

    @property
    def quaternion(self) -> Quaternion:
        return self._quaternion
    @property
    def roll(self) -> float:
        return self._roll
    @property
    def pitch(self) -> float:
        return self._pitch
    @property
    def yaw(self) -> float:
        return self._yaw

    def parseFromMsgIMU(self, data: Imu) -> None:
        self._quaternion.x = data.orientation.x
        self._quaternion.y = data.orientation.y
        self._quaternion.z = data.orientation.z
        self._quaternion.w = data.orientation.w

        self._roll, self._pitch, self._yaw = self._quaternion.toEuler()


class Pinger:
    currentPinger = None

    def __init__(self):
        self._range: float = 0.0 # in meters
        self._bearing: float = 0.0 # in radians relative to boat (positive clockwise)

        Pinger.currentPinger: Pinger = self
    
    def parseFromParamVec(self, data_list: list[Parameter]) -> None:
        for data in data_list:
            if data.name == "range":
                self._range = data.value.double_value
            elif data.name == "bearing":
                self._bearing = data.value.double_value
    
    @property
    def range(self) -> float:
        return self._range
    
    @property
    def bearing(self) -> float:
        return self._bearing

    def __str__(self) -> str:
        return f"range: {self.range}\nbearing: {self.bearing}\n"


class LidarScan:
    def __init__(self):
        self._ranges: list[float] = []
        self._angles: list[float] = []
        self._ox: list[float] = []
        self._oy: list[float] = []
    
    @property
    def ranges(self) -> list[float]:
        return self._ranges
    @property
    def angles(self) -> list[float]:
        return self._angles
    
    def parseFromMsgLidar(self, data: LaserScan) -> None:
        self._ranges = np.array(data.ranges)
        self._angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
        if np.size(self._ranges) != np.size(self._angles):
            raise Exception("LidarScan: ranges and angles are not the same size!!")

        self._ox = np.sin(self._angles) * self._ranges
        self._oy = np.cos(self._angles) * self._ranges


class SieraNode(Node):
    def __init__(self):
        super().__init__('SieraNode')
        self.taskinfo: TaskInfo = TaskInfo()
        self.pinger: Pinger = Pinger()
        self.imu: IMU = IMU()
        self.lidarScan: LidarScan = LidarScan()

        self.task_info_sub = self.create_subscription(
            ParamVec,
            '/vrx/task/info',
            self.taskinfo_callback,
            10)
        self.task_info_sub  # prevent unused variable warning

        self.pinger_sub = self.create_subscription(
            ParamVec,
            '/wamv/sensors/acoustics/receiver/range_bearing',
            self.pinger_callback,
            10)
        self.pinger_sub  # prevent unused variable warning

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
        This is the main loop of the node. It is called every 0.1s. Loop time should be less than 100ms.
        """
        start_exec = time.process_time()

        print(self.pinger)
        print(f"Yaw: {self.imu.yaw}")

        print(f"Loop time: {(time.process_time() - start_exec) * 1000}ms")

    def taskinfo_callback(self, msg: ParamVec):
        self.taskinfo.parseFromParamVec(msg.params)

    def pinger_callback(self, msg: ParamVec):
        self.pinger.parseFromParamVec(msg.params)

    def imu_callback(self, msg: Imu):
        self.imu.parseFromMsgIMU(msg)
    
    def lidar_callback(self, msg: LaserScan):
        self.lidarScan.parseFromMsgLidar(msg)

def main(args=None):
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