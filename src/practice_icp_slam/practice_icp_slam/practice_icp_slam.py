import os
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from .utils import point_cloud2
from .utils import ICP
from .utils import PtAssistant 

class Practice_ICP_SLAM(Node):
    def __init__(self):
        super().__init__('practice_icp_slam_node')

        self.pointcloud_subscriber = self.create_subscription(
                                    PointCloud2,
                                    '/diff_drive/pointcloud',
                                    self.pointcloud_callback,
                                    10
                                    )

        self.initialize = False
        self.prev_pt = None
        self.curr_pt = None
        self.icp_initial = np.eye(4)
        self.num_icp_points = 5000
        ''' Test From Kitti ''' 
        # kitti_dataset_path = os.path.join('/root', 'navigation_ws', 'src', 'datas', 'LiDAR-Point-Cloud-Preprocessing-matlab', 'sample', '1.bin')
        # scan = np.fromfile(kitti_dataset_path, dtype=np.float32)
        # scan = scan.reshape((-1, 4))
        # ptcloud_xyz = scan[:, :-1]

    def pointcloud_callback(self, msg):
        pt_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.curr_pt = np.array(list(pt_data))
        self.curr_pt = self.curr_pt[~np.isinf(self.curr_pt).any(axis=1)]
        if self.prev_pt is None:
            self.prev_pt = self.curr_pt 
            self.initialize = True
        else:
            curr_down_pt = PtAssistant.random_sampling(self.curr_pt, self.num_icp_points)
            prev_down_pt = PtAssistant.random_sampling(self.prev_pt, self.num_icp_points)
            odom_transform, _, _ = ICP.icp(curr_down_pt, prev_down_pt, init_pose=self.icp_initial, max_iterations=20)
            self.icp_initial = odom_transform
            self.prev_pt = self.curr_pt
            print(odom_transform)

def main(args=None):
    rclpy.init(args=args)
    practice_icp_slam = Practice_ICP_SLAM()

    while rclpy.ok():
        rclpy.spin(practice_icp_slam)

    practice_icp_slam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
