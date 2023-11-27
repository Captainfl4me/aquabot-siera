""" Main file for the Siera Python package. """
from enum import Enum
from math import cos, sin

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from ros_gz_interfaces.msg import ParamVec
from rcl_interfaces.msg import Parameter
from std_msgs.msg import Float64


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

    def get_xy(self, security_circle=0.0) -> tuple[float, float]:
        """
        Returns the x and y coordinates of the pinger relative to the boat.

        :return: The x and y coordinates of the pinger relative to the boat.
        """
        return ((self._range-security_circle) * sin(self._bearing), (self._range-security_circle) * cos(self._bearing))

    def __str__(self) -> str:
        return f"range: {self.range}\nbearing: {self.bearing}\n"


class SieraConvertNode(Node):
    """ Main node class. """
    def __init__(self):
        super().__init__('SieraNode')
        self.taskinfo: TaskInfo = TaskInfo()
        self.pinger: Pinger = Pinger()

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

        self.pinger_range_pub = self.create_publisher(
            Float64,
            '/siera/pinger/range',
            10)
        self.pringer_bearing_pub = self.create_publisher(
            Float64,
            '/siera/pinger/bearing',
            10)

        self.get_logger().info('SieraConvertNode has been started')

    def taskinfo_callback(self, msg: ParamVec):
        """ Callback for the task info message. """
        self.taskinfo.parse_from_paramvec(msg.params)

    def pinger_callback(self, msg: ParamVec):
        """ Callback for the pinger message. """
        self.pinger.parse_from_paramvec(msg.params)
        self.pinger_range_pub.publish(Float64(data=self.pinger.range))
        self.pringer_bearing_pub.publish(Float64(data=self.pinger.bearing))


def main(args=None):
    """ Main function. """
    rclpy.init(args=args)
    try:
        convert_node = SieraConvertNode()

        executor = SingleThreadedExecutor()
        executor.add_node(convert_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            convert_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
