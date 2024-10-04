import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from math import pi, cos, sin
from tm_msgs.msg import *
from tm_msgs.srv import *
from move_tm.tmr_utils import TMRUtils

tmr = TMRUtils()
m2mm = 1000
mm2m = 1 / 1000
r2d = 180 / np.pi
d2r = 1 / r2d

class RobotTF(Node):
    def __init__(self):
        super().__init__('robot_tf_sim')

        # Robot transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / 30, self.timer_callback)  # 30 Hz
        
        self.t = 0

        '''
        D-H params
        '''
        self.dh_params = self.dh_table()
    
        # Initialize robot configuration
        self.joint_angles = np.array([0., 0., 90., 0., 90., -90.]) * d2r  # [rad]

    def dh_table(self):
        '''
        Set up robot modified D-H parameters [m, rad] 
        '''
        # # TM5M-700
        # alpha0 =       0; a0 =            0; d1 =  145.2 * mm2m; # frame {0} -> {1}
        # alpha1 = -pi / 2; a1 =            0; d2 =             0; # frame {1} -> {2}
        # alpha2 =       0; a2 = 329.0 * mm2m; d3 =             0; # frame {2} -> {3}
        # alpha3 =       0; a3 = 311.5 * mm2m; d4 = -122.3 * mm2m; # frame {3} -> {4}
        # alpha4 = -pi / 2; a4 =            0; d5 =    106 * mm2m; # frame {4} -> {5}
        # alpha5 = -pi / 2; a5 =            0; d6 = 113.15 * mm2m; # frame {5} -> {6}

        # TM5M-900
        alpha0 =       0; a0 =            0; d1 =  165.2 * mm2m; # frame {0} -> {1}
        alpha1 = -pi / 2; a1 =            0; d2 =             0; # frame {1} -> {2}
        alpha2 =       0; a2 = 536.1 * mm2m; d3 =             0; # frame {2} -> {3}
        alpha3 =       0; a3 = 457.9 * mm2m; d4 = -156.3 * mm2m; # frame {3} -> {4}
        alpha4 = -pi / 2; a4 =            0; d5 =    106 * mm2m; # frame {4} -> {5}
        alpha5 = -pi / 2; a5 =            0; d6 = 113.15 * mm2m; # frame {5} -> {6}
        
        # Angle offsets in D-H params, dh_offsets [deg]
        dh_offset1 = 0 
        dh_offset2 = -pi / 2
        dh_offset3 = 0 
        dh_offset4 = -pi / 2
        dh_offset5 = 0 
        dh_offset6 = pi
        
        return np.array([[alpha0, a0, d1, dh_offset1],
                         [alpha1, a1, d2, dh_offset2],
                         [alpha2, a2, d3, dh_offset3],
                         [alpha3, a3, d4, dh_offset4],
                         [alpha4, a4, d5, dh_offset5],
                         [alpha5, a5, d6, dh_offset6],], dtype=float) # [m] [rad]

    
    def dh_transform(self, alpha, a, d, theta, i):
        Rot_alpha = tmr.Rp2SE3(tmr.Rotx(alpha, degree=False), np.zeros(3))
        Trans_a = tmr.Rp2SE3(np.eye(3), np.array([a, 0, 0]))
        Trans_d = tmr.Rp2SE3(np.eye(3), np.array([0, 0, d]))
        Rot_theta = tmr.Rp2SE3(tmr.Rotz(theta, degree=False), np.zeros(3))
        return Rot_alpha @ Trans_a @ Trans_d @ Rot_theta
    
    def publish_tf(self, transform, child_frame_id, parent_frame_id):
        tf_stamp = TransformStamped()
        tf_stamp.header.stamp = self.get_clock().now().to_msg()
        tf_stamp.header.frame_id = parent_frame_id
        tf_stamp.child_frame_id = child_frame_id

        tf_stamp.transform.translation.x = transform[0, 3]
        tf_stamp.transform.translation.y = transform[1, 3]
        tf_stamp.transform.translation.z = transform[2, 3]

        # Convert rotation matrix to quaternion
        quat = tmr.Rot2Quat(transform[:3, :3])
        tf_stamp.transform.rotation.w = quat[0]
        tf_stamp.transform.rotation.x = quat[1]
        tf_stamp.transform.rotation.y = quat[2]
        tf_stamp.transform.rotation.z = quat[3]

        self.tf_broadcaster.sendTransform(tf_stamp)
    
    def time_varying_joint_angles(self):
        # self.t += 1
        # theta1 = 10 * sin(2* np.pi * 0.01 * self.t)
        # theta2 = 10 * cos(2* np.pi * 0.02 * self.t)
        # theta3 = 10 * sin(2* np.pi * 0.03 * self.t)
        # theta4 = 10 * cos(2* np.pi * 0.04 * self.t)
        # theta5 = 10 * sin(2* np.pi * 0.05 * self.t)
        # theta6 = 10 * cos(2* np.pi * 0.06 * self.t)
        # self.joint_angles = np.array([theta1, theta2, theta3, theta4, theta5, theta6]) * d2r 
        # self.joint_angles += np.array([0., 0., 90., 0., 90., -90.]) * d2r

        self.joint_angles = np.array([-264.77154541015625, -68.06529998779297, 130.55740356445312, 29.048137664794922, 93.86833190917969, -90]) * d2r
    
    def publish_transforms(self):
        self.publish_tf(np.eye(4), "B_sim", "map")
        
        for i in range(6):
            [alpha, a, d, dh_offset] = self.dh_params[i]
            theta = self.joint_angles[i] + dh_offset
            transform = self.dh_transform(alpha, a, d, theta, i)
            self.publish_tf(transform, f"j{i+1}_sim", "B_sim" if i == 0 else f"j{i}_sim")

        self.publish_tf(tmr.R6Pose2SE3(np.array([0, 0, 100 * mm2m, 0, 0, 0])), "E_sim", "j6_sim")
    
    def timer_callback(self):
        self.time_varying_joint_angles()
        self.publish_transforms()


def main(args=None):
    rclpy.init(args=args)
    node = RobotTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
