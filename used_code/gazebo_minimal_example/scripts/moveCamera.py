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



class moveCamera(ROSNode):
    def __init__(self):
        super(moveCamera, self).__init__()
        self.pose = None
        self.pose_topic = "/my_depth_camera/pose"
        self.pose_set_topic = "/my_depth_camera/pose/set"
        self.register_publisher(self.pose_set_topic, Pose)
        self.register_subscriber(self.pose_topic, PoseStamped, self.pose_cb)

    def pose_cb(self, msg):
        self.pose = msg


    def set_orientation(self, q_ws):
        msg = Pose()
        msg.position.x = self.pose.pose.position.x
        msg.position.y = self.pose.pose.position.y
        msg.position.z = self.pose.pose.position.z
        msg.orientation.w = q_ws[0]
        msg.orientation.x = q_ws[1]
        msg.orientation.y = q_ws[2]
        msg.orientation.z = q_ws[3]
        self.pubs[self.pose_set_topic].publish(msg)


    def set_position(self, r_ws):
        msg = Pose()
        msg.position.x = r_ws[0]
        msg.position.y = r_ws[1]
        msg.position.z = r_ws[2]
        msg.orientation.w = self.pose.pose.orientation.w
        msg.orientation.x = self.pose.pose.orientation.x
        msg.orientation.y = self.pose.pose.orientation.y
        msg.orientation.z = self.pose.pose.orientation.z
        self.pubs[self.pose_set_topic].publish(msg)


    def set_pose(self, r_WS, q_WS):
        msg = Pose()
        msg.position.x = self.pose.pose.position.x
        msg.position.y = self.pose.pose.position.y
        msg.position.z = self.pose.pose.position.z
        msg.orientation.w = q_WS[0]
        msg.orientation.x = q_WS[1]
        msg.orientation.y = q_WS[2]
        msg.orientation.z = q_WS[3]
        self.pubs[self.pose_set_topic].publish(msg)

    def moveInX(self, targetX):
        msg = Pose()
        msg.position.x = self.pose.pose.position.x
        msg.position.y = self.pose.pose.position.y
        msg.position.z = self.pose.pose.position.z
        msg.orientation.w = self.pose.pose.orientation.w
        msg.orientation.x = self.pose.pose.orientation.x
        msg.orientation.y = self.pose.pose.orientation.y
        msg.orientation.z = self.pose.pose.orientation.z
        self.pubs[self.pose_set_topic].publish(msg)

    def test(self):
        print('I am camera')


if __name__ == "__main__":
    rospy.init_node("moveCamera")
    depth_camera = moveCamera()
    rospy.sleep(1.0)


    move_velocity = -0.1
    move_dis = -4
    move_time = move_dis / move_velocity

    curPoseX = -2
    targetX = -6
    dt = 0.001

    
    x = -2
    y = 1
    z = 2
    maxMove = 3
    moved = 0

    while moved<maxMove and not rospy.is_shutdown():
        y = y - 0.1 * dt
        moved += 0.1*dt
        depth_camera.set_position([x,y,z])
        sleep(dt)


    # while curPoseX >= targetX and not rospy.is_shutdown():
    # 	curPoseX = curPoseX + move_velocity * dt
    # 	x = curPoseX
    # 	depth_camera.moveInX(x)
    # 	sleep(dt)
