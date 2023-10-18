import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class Practice_ICP_SLAM(Node):
    def __init__(self):
        super().__init__('practice_icp_slam_node')

        self.pointcloud_subscriber = self.create_subscription(
                                    PointCloud2,
                                    '/diff_drive/pointcloud',
                                    self.pointcloud_callback,
                                    10
                                    )


    def pointcloud_callback(self, msg):
        print('receive pointcloud')

def main(args=None):
    rclpy.init(args=args)
    practice_icp_slam = Practice_ICP_SLAM()

    while rclpy.ok():
        rclpy.spin(practice_icp_slam)

    practice_icp_slam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
