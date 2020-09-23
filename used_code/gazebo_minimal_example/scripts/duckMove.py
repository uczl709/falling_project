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



class moveDuck(ROSNode):
    def __init__(self):
        super(moveDuck, self).__init__()
        self.pose = None
        self.pose_topic = "/my_duck/pose"
        self.pose_set_topic = "/my_duck/pose/set"
        self.register_publisher(self.pose_set_topic, Pose)
        self.register_subscriber(self.pose_topic, PoseStamped, self.pose_cb)

    def pose_cb(self, msg):
        self.pose = msg

    def get_time(self):
      return (self.pose.header.stamp).to_sec()

    def moveInX(self, targetX):
        msg = Pose()
        msg.position.x = targetX
        msg.position.y = self.pose.pose.position.y
        msg.position.z = self.pose.pose.position.z
        msg.orientation.w = self.pose.pose.orientation.w
        msg.orientation.x = self.pose.pose.orientation.x
        msg.orientation.y = self.pose.pose.orientation.y
        msg.orientation.z = self.pose.pose.orientation.z
        self.pubs[self.pose_set_topic].publish(msg)

    def get_current_pose(self):
      return [self.pose.pose.orientation.w,
              self.pose.pose.orientation.x,
              self.pose.pose.orientation.y,
              self.pose.pose.orientation.z]

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
        msg.position.x = r_WS[0]
        msg.position.y = r_WS[1]
        msg.position.z = r_WS[2]
        msg.orientation.w = q_WS[0]
        msg.orientation.x = q_WS[1]
        msg.orientation.y = q_WS[2]
        msg.orientation.z = q_WS[3]
        self.pubs[self.pose_set_topic].publish(msg)



if __name__ == "__main__":
    rospy.init_node("moveduck")
    duck = moveDuck()
    rospy.sleep(1.0)

    print("start:")

    move_velocity = -0.2
    move_dis = -4
    move_time = move_dis / move_velocity

    roll_begin = 0
    pitch_begin = 0
    yaw_begin = pi * (90/180)
    #yaw_begin = 2.844426

    scan_velocity = 0.5
    scan_dist = 2.0 * pi * 2
    scan_time = scan_dist / scan_velocity

    w =  pi / scan_time
    w = 3/2.5
    print("w is:",w)
    dt = 0.001
    rotSum = 0

    x = 0
    y = 0
    xsum = 0

    rotSum = 0
    timeSum = 0

    duck_px = 0

    while rotSum<= 3*pi and xsum<=3 and not rospy.is_shutdown():
      yaw_begin = yaw_begin + 2.5*w*dt
      timeSum += dt
      rotSum += 2*w*dt
      if yaw_begin >pi:
        yaw_begin -= 2*pi
      elif yaw_begin < -pi:
        yaw_begin += 2*pi
      #rot = euler2quat([roll_begin,pitch_begin,yaw_begin])
      #duck.set_orientation(rot)
      #duck_px -= 0.08*dt
      duck.set_pose([duck_px,0,0.8],
              euler2quat([roll_begin,pitch_begin,yaw_begin]))
      sleep(dt)

    print("rotation speed:", rotSum/timeSum)


    # rotSum = 0
    # while rotSum<=2.5*pi and not rospy.is_shutdown():
    #   roll_begin = roll_begin + 2*w*dt
    #   rotSum += 2*w*dt
    #   if roll_begin >pi:
    #     roll_begin -= 2*pi
    #   elif roll_begin < -pi:
    #     roll_begin += 2*pi
    #   rot = euler2quat([roll_begin,pitch_begin,yaw_begin])
    #   duck.set_orientation(rot)
    #   sleep(dt)


    # rotSum = 0
    # while rotSum<= 3*pi and xsum<=3 and not rospy.is_shutdown():
    #   pitch_begin = pitch_begin + 2.5*w*dt

    #   rotSum += 2.5*w*dt
    #   if pitch_begin >pi:
    #     pitch_begin -= 2*pi
    #   elif pitch_begin < -pi:
    #     pitch_begin += 2*pi
    #   rot = euler2quat([roll_begin,pitch_begin,yaw_begin])
    #   duck.set_orientation(rot)
    #   sleep(dt)




    # sleep(1)


#############################good####################################

    # rotSum = 0
    # # #roatation along the z
    # while rotSum<= 2*pi and xsum<=3 and not rospy.is_shutdown():
    #   yaw_begin = yaw_begin + 2*w*dt
    #   #pitch_begin = pitch_begin + w*dt
    #   #roll_begin = roll_begin + 2*w*dt
    #   # x = x - 0.1*dt
    #   # y = y - 0.05*dt
    #   #xsum += 0.1*dt
    #   rotSum += 2*w*dt
    #   if yaw_begin >pi:
    #     yaw_begin -= 2*pi
    #   elif yaw_begin < -pi:
    #     yaw_begin += 2*pi
    #   rot = euler2quat([roll_begin,pitch_begin,yaw_begin])
    #   duck.set_orientation(rot)
    #   sleep(dt)

    # sleep(1)

    # #rotation along the y
    # rotSum = 0
    # while rotSum<=4*pi and not rospy.is_shutdown():
    #   roll_begin = roll_begin - 2*w*dt
    #   rotSum += 2*w*dt
    #   if roll_begin >pi:
    #     roll_begin -= 2*pi
    #   elif roll_begin < -pi:
    #     roll_begin += 2*pi
    #   rot = euler2quat([roll_begin,pitch_begin,yaw_begin])
    #   duck.set_orientation(rot)
    #   sleep(dt)


    sleep(1)
#############################good####################################

    # rotSum = 0
    # # #roatation along the z
    # while rotSum<= 0.5*pi and xsum<=3 and not rospy.is_shutdown():
    #   yaw_begin = yaw_begin + 2*w*dt
    #   rotSum += 2*w*dt
    #   if yaw_begin >pi:
    #     yaw_begin -= 2*pi
    #   elif yaw_begin < -pi:
    #     yaw_begin += 2*pi
    #   rot = euler2quat([roll_begin,pitch_begin,yaw_begin])
    #   duck.set_pose([x,y,0],rot)
    #   sleep(dt)


    # rotSum = 0
    # while rotSum<=0.3*pi and not rospy.is_shutdown():
    #   pitch_begin = pitch_begin - 2*w*dt
    #   rotSum += 2*w*dt
    #   if pitch_begin >pi:
    #     pitch_begin -= 2*pi
    #   elif pitch_begin < -pi:
    #     pitch_begin += 2*pi
    #   rot = euler2quat([roll_begin,pitch_begin,yaw_begin])
    #   duck.set_orientation(rot)
    #   sleep(dt)

    # rotSum = 0
    # while rotSum<=0.6*pi and not rospy.is_shutdown():
    #   pitch_begin = pitch_begin + 2*w*dt
    #   rotSum += 2*w*dt
    #   if pitch_begin >pi:
    #     pitch_begin -= 2*pi
    #   elif pitch_begin < -pi:
    #     pitch_begin += 2*pi
    #   rot = euler2quat([roll_begin,pitch_begin,yaw_begin])
    #   duck.set_orientation(rot)
    #   sleep(dt)

    #duck.set_pose([0,0,2],tmp)



