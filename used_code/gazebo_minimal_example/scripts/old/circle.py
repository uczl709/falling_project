#!/usr/bin/python
from math import pi
from math import sin
from math import cos
from math import sqrt
from time import sleep

import numpy as np

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

import rospy


def euler2quat(euler):
   phi, theta, psi = euler

   c_phi = cos(phi / 2.0)
   c_theta = cos(theta / 2.0)
   c_psi = cos(psi / 2.0)
   s_phi = sin(phi / 2.0)
   s_theta = sin(theta / 2.0)
   s_psi = sin(psi / 2.0)

   qx = s_phi * c_theta * c_psi - c_phi * s_theta * s_psi
   qy = c_phi * s_theta * c_psi + s_phi * c_theta * s_psi
   qz = c_phi * c_theta * s_psi - s_phi * s_theta * c_psi
   qw = c_phi * c_theta * c_psi + s_phi * s_theta * s_psi

   mag = sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
   q = [qw / mag, qx / mag, qy / mag, qz / mag]
   return q


class ROSNode(object):
    def __init__(self):
        self.pubs = {}
        self.subs = {}

    def register_publisher(self, topic, msg_type, queue_size=1):
        self.pubs[topic] = rospy.Publisher(topic,
                                           msg_type,
                                           queue_size=queue_size)

    def register_subscriber(self, topic, msg_type, callback):
        self.subs[topic] = rospy.Subscriber(topic, msg_type, callback)


class MyDepthCamera(ROSNode):
    def __init__(self):
        super(MyDepthCamera, self).__init__()
        self.pose = None
        self.pose_topic = "/my_depth_camera/pose"
        self.pose_set_topic = "/my_depth_camera/pose/set"

        self.register_publisher(self.pose_set_topic, Pose)
        self.register_subscriber(self.pose_topic, PoseStamped, self.pose_cb)

    def pose_cb(self, msg):
        self.pose = msg

    def set_orientation(self, q_WS):
        msg = Pose()
        msg.position.x = self.pose.pose.position.x
        msg.position.y = self.pose.pose.position.y
        msg.position.z = self.pose.pose.position.z
        msg.orientation.w = q_WS[0]
        msg.orientation.x = q_WS[1]
        msg.orientation.y = q_WS[2]
        msg.orientation.z = q_WS[3]
        self.pubs[self.pose_set_topic].publish(msg)

    def set_position(self, r_WS):
        msg = Pose()
        msg.position.x = r_WS[0]
        msg.position.y = r_WS[1]
        msg.position.z = r_WS[2]
        msg.orientation.w = self.pose.pose.orientation.w
        msg.orientation.x = self.pose.pose.orientation.x
        msg.orientation.y = self.pose.pose.orientation.y
        msg.orientation.z = self.pose.pose.orientation.z
        self.pubs[self.pose_set_topic].publish(msg)

    def set_pose(self, r_WS, q_WS):
        msg = Pose()
        msg.position.x = r_WS[0]
        msg.position.y = r_WS[1]
        msg.position.z = r_WS[2]
        msg.orientation.w = q_WS[0]
        msg.orientation.x = q_WS[1]
        msg.orientation.y = q_WS[2]
        msg.orientation.z = q_WS[3]
        self.pubs[self.pose_set_topic].publish(msg)



if __name__ == "__main__":
    rospy.init_node("circle")
    depth_camera = MyDepthCamera()
    rospy.sleep(1.0)

    circle_c = [0.0, 0.0]
    scan_r = 2.0
    scan_h = 2.0
    scan_pitch = 0.4

    scan_velocity = 0.5
    scan_dist = 2.0 * pi * scan_r
    scan_time = scan_dist / scan_velocity

    w = 2.0 * pi / scan_time
    dt = 0.001  # Gazebo sim time-step

    theta_start = -pi
    theta_end = pi
    theta = theta_start

    while (theta <= theta_end and not rospy.is_shutdown()):
        theta = theta + w * dt  # Move counter clock-wise

        # Calculate position and orientation
        x = scan_r * cos(theta) + circle_c[0]
        y = scan_r * sin(theta) + circle_c[1]
        z = scan_h
        roll = 0.0
        pitch = scan_pitch
        yaw = theta - pi # to point inwards

        # Wrap yaw to -180 to 180
        if yaw > pi:
            yaw -= 2.0 * pi
        elif yaw < -pi:
            yaw += 2.0 * pi

        # Set depth camera pose
        pos = [x, y, z]
        rot = euler2quat([roll, pitch, yaw])
        depth_camera.set_pose(pos, rot)
        sleep(dt)

    # theta_start = -pi
    # theta_end = pi
    # theta = theta_start

    # while (theta <= theta_end and not rospy.is_shutdown()):
    #     theta = theta + w * dt  # Move counter clock-wise

    #     # Calculate position and orientation
    #     x = scan_r * cos(theta) + circle_c[0]
    #     y = 0
    #     z = scan_r * sin(theta) + circle_c[0] + 2
        
    #     roll = 0
    #     pitch = pi - theta
    #     yaw = 0 # to point inwards

    #     # Wrap yaw to -180 to 180
    #     if yaw > pi:
    #         yaw -= 2.0 * pi
    #     elif yaw < -pi:
    #         yaw += 2.0 * pi

    #     # Set depth camera pose
    #     pos = [x, y, z]
    #     rot = euler2quat([roll, pitch, yaw])
    #     depth_camera.set_pose(pos, rot)
    #     sleep(dt)
