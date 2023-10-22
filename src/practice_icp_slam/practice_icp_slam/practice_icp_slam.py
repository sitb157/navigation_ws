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
from .utils.UtilsMisc import *

np.set_printoptions(precision=4)
from matplotlib.animation import FFMpegWriter

class Practice_ICP_SLAM(Node):
    def __init__(self):
        super().__init__('practice_icp_slam_node')
        self.pointcloud_subscriber = self.create_subscription(
                                    PointCloud2,
                                    '/diff_drive/pointcloud',
                                    self.pointcloud_callback,
                                    10
                                    )

        self.prev_pts = None
        self.curr_pts = None
        self.icp_initial = np.eye(4)
        self.num_icp_points = 5000
        self.curr_idx = 0
        self.PGM = PoseGraphManager()
        self.PGM.addPriorFactor()
        self.SCM = ScanContextManager(shape=[20, 60], num_candidates=10, threshold=0.11)
        self.try_gap_loop_detection = 10
        self.result_saver = None
        self.num_frames_to_skip_to_show = 5
        self.fig_idx = 1
        self.fig = plt.figure(self.fig_idx)
        self.writer = FFMpegWriter(fps=15)
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

        if self.prev_pts is not None:
            prev_down_pts = PtAssistant.random_sampling(self.prev_pts, self.num_icp_points)
            odom_transform, _, _ = ICP.icp(curr_down_pts, prev_down_pts, init_pose=self.icp_initial, max_iterations=20)
            self.icp_initial = odom_transform
            self.prev_pts = self.curr_pts
            #translation = odom_transform[:3, 3]
            #rotation = odom_transform[:3, :3]
            self.PGM.curr_se3 = np.matmul(self.PGM.curr_se3, odom_transform)
            self.PGM.addOdometryFactor(odom_transform)
        else:
            self.result_saver = PoseGraphResultSaver(init_pose=self.PGM.curr_se3,
                                                    save_gap=300, 
                                                    num_frames=10000,
                                                    seq_idx='00',
                                                    save_dir='/root/navigation_ws/src')

        self.curr_idx += 1
        self.prev_pts = copy.deepcopy(self.curr_pts)
        self.PGM.prev_node_idx = self.PGM.curr_node_idx
        print(f'curr idx is {self.PGM.curr_node_idx}')
        if (self.PGM.curr_node_idx > 1) and (self.PGM.curr_node_idx % self.try_gap_loop_detection == 0):
            loop_idx, loop_dist, yaw_diff_deg = self.SCM.detectLoop()
            if (loop_idx == None):
                pass
            else:
                loop_down_pts = self.SCM.getPtcloud(loop_idx)
                loop_transform, _, _ = ICP.icp(curr_down_pts, loop_down_pts, init_pose=yawdeg2se3(yaw_diff_deg), max_iterations=20)
                print(f'detect loop idx is {loop_idx}')
                print(f'curr idx is {self.PGM.curr_node_idx}')
                self.PGM.addLoopFactor(loop_transform, loop_idx)
                self.PGM.optimizePoseGraph()
                print('optimize done')
                self.result_saver.saveOptimizedPoseGraphResult(self.PGM.curr_node_idx, self.PGM.graph_optimized)
        self.result_saver.saveUnoptimizedPoseGraphResult(self.PGM.curr_se3, self.PGM.curr_node_idx)
        if(self.curr_idx % self.num_frames_to_skip_to_show == 0):
            self.result_saver.vizCurrentTrajectory(fig_idx=self.fig_idx)
                
def main(args=None):
    rclpy.init(args=args)
    practice_icp_slam = Practice_ICP_SLAM()

    while rclpy.ok():
        rclpy.spin(practice_icp_slam)

    practice_icp_slam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
