import os
import sys
import copy
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from .utils import point_cloud2
from .utils import ICP
from .utils import PtAssistant 
from .utils.PoseGraphManager import PoseGraphManager
from .utils.ScanContextManager import ScanContextManager

class Practice_ICP_SLAM(Node):
    def __init__(self):
        super().__init__('practice_icp_slam_node')

        self.pointcloud_subscriber = self.create_subscription(
                                    PointCloud2,
                                    '/diff_drive/pointcloud',
                                    self.pointcloud_callback,
                                    10
                                    )

        self.prev_pt = None
        self.curr_pt = None
        self.icp_initial = np.eye(4)
        self.num_icp_points = 5000
        self.curr_idx = 0
        self.PGM = PoseGraphManager()
        self.PGM.addPriorFactor()
        self.SCM = ScanContextManager(shape=[20, 60], num_candidates=10, threshold=0.11)
        self.try_gap_loop_detection = 100
        ''' Test From Kitti ''' 
        # kitti_dataset_path = os.path.join('/root', 'navigation_ws', 'src', 'datas', 'LiDAR-Point-Cloud-Preprocessing-matlab', 'sample', '1.bin')
        # scan = np.fromfile(kitti_dataset_path, dtype=np.float32)
        # scan = scan.reshape((-1, 4))
        # ptcloud_xyz = scan[:, :-1]

    def pointcloud_callback(self, msg):
        pt_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.curr_pts = np.array(list(pt_data))
        self.curr_pts = self.curr_pts[~np.isinf(self.curr_pts).any(axis=1)]
        curr_down_pts = PtAssistant.random_sampling(self.curr_pts, self.num_icp_points)
        self.PGM.curr_node_idx = self.curr_idx
        self.SCM.addNode(self.PGM.curr_node_idx, ptcloud=curr_down_pts)

        if self.prev_pt is not None:
            prev_down_pts = PtAssistant.random_sampling(self.prev_pts, self.num_icp_points)
            odom_transform, _, _ = ICP.icp(curr_down_pts, prev_down_pts, init_pose=self.icp_initial, max_iterations=20)
            self.icp_initial = odom_transform
            self.prev_pts = self.curr_pts
            translation = odom_transform[:3, 3]
            rotation = odom_transform[:3, :3]
            print("Translation Vector:")
            print(translation)
            print("\nRotation matrix:")
            print(rotation)
            PGM.curr_se3 = np.matmul(PGM.curr_se3, odom_transform)
            PGM.addOdometryFactor(odom_transform)

        self.curr_idx += 1
        self.prev_pt = copy.deepcopy(self.curr_pt)
        self.PGM.prev_node_idx = self.PGM.curr_node_idx
        if (self.PGM.curr_node_idx > 1) and (self.PGM.curr_node_idx % self.try_gap_loop_detection == 0):
            loop_idx, loop_dist, yaw_diff_deg = self.SCM.detectLoop()
            if (loop_idx == None):
                pass
            else:
                print(f'detect loop idx is {loop_idx}')
                print(f'curr idx is {self.PGM.curr_node_idx}')

def main(args=None):
    rclpy.init(args=args)
    practice_icp_slam = Practice_ICP_SLAM()

    while rclpy.ok():
        rclpy.spin(practice_icp_slam)

    practice_icp_slam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
