import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CloudCheck(Node):

    def __init__(self):
        super().__init__('cloud_check')

        self.sub = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.callback,
            10
        )

    def callback(self, msg):

        times = []
        rings = []

        for p in point_cloud2.read_points(
            msg,
            field_names=("ring","time"),
            skip_nans=True
        ):
            rings.append(p[0])
            times.append(p[1])

        print("===================")
        print("points:", len(times))

        print(
            "ring:",
            min(rings),
            max(rings)
        )

        print(
            "time:",
            min(times),
            max(times)
        )

        self.destroy_node()


def main():

    rclpy.init()

    node = CloudCheck()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__=="__main__":
    main()
