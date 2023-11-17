""" Gridmap ros2 node """
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np

from siera_python.raycasting_grid_map import generate_ray_casting_grid_map


class GridmapNode(Node):
    """ Gridmap ros2 node """
    def __init__(self):
        super().__init__('gridmap_node')
        self.get_logger().info("Gridmap node started.")

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/wamv/sensors/lidars/lidar_wamv_sensor/scan',
            self.lidar_callback,
            10)

        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid,
            '/siera/gridmap',
            10)
        
        self.get_logger().info("Gridmap node initialized.")

    def lidar_callback(self, msg: LaserScan):
        """ Callback for the lidar message. """
        # self.get_logger().info("LidarScan received.")
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        # Remove values below range_min or above range_max
        filter_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]
        angles = angles[(ranges >= msg.range_min) & (ranges <= msg.range_max)]
        ranges = filter_ranges

        if np.size(ranges) != np.size(angles):
            raise ValueError("LidarScan: ranges and angles are not the same size!!")

        if np.size(ranges) == 0:
            # self.get_logger().info("No data in the lidar scan.")
            return

        ox = np.sin(angles) * ranges
        oy = np.cos(angles) * ranges
        xyreso = 2  # x-y grid resolution [m]
        yawreso = np.deg2rad(1)  # yaw angle resolution [rad]
        pmap, minx, maxx, miny, maxy, xyreso = generate_ray_casting_grid_map(ox, oy, xyreso, yawreso, max_range=130.0)
        # self.get_logger().info("Gridmap computed.")

        gridmap_msg = OccupancyGrid()
        gridmap_msg.header.stamp = self.get_clock().now().to_msg()
        gridmap_msg.header.frame_id = "wamv/wamv/base_link"
        gridmap_msg.info.resolution = float(xyreso)
        gridmap_msg.info.width = pmap.shape[1]
        gridmap_msg.info.height = pmap.shape[0]
        gridmap_msg.info.origin.position.x = float(miny)
        gridmap_msg.info.origin.position.y = float(minx)
        gridmap_msg.info.origin.position.z = 0.0
        gridmap_msg.info.origin.orientation.x = 0.0
        gridmap_msg.info.origin.orientation.y = 0.0
        gridmap_msg.info.origin.orientation.z = 0.0
        gridmap_msg.info.origin.orientation.w = 1.0
        gridmap_msg.data = pmap.flatten().tolist()

        self.occupancy_grid_pub.publish(gridmap_msg)
        # self.get_logger().info("Gridmap published.")



def main(args=None):
    """ Main function. """
    rclpy.init(args=args)
    try:
        siera_node = GridmapNode()

        executor = SingleThreadedExecutor()
        executor.add_node(siera_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            siera_node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
