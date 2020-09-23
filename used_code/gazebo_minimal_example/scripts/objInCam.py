import numpy as np
from time import sleep
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

from pyquaternion import Quaternion
import math

import rospy

class objIncam(object):
	def __init__(self):
		self.pubs = {}
		self.subs = {}
		self.obj_T_world = None
		self.cam_T_world = None
		self.obj_T_cam = None
		self.cam_T_obj = None

		self.TimeHeader = None
	
	def register_publisher(self, topic, msg_type, queue_size = 1):
		self.pubs[topic] = rospy.Publisher(	topic,
										  	msg_type,
											queue_size = queue_size)

	def register_subscriber(self, topic, msg_type, callback):
		self.subs[topic] = rospy.Subscriber(topic, msg_type, callback)

	def initNode(self):
		#self.objInWorld_Topic = "/my_depth_camera/pose"
		#self.camInWorld_Topic = "/gazebo/model_states"
		#self.register_subscriber(self.camInWorld_Topic, PoseStamped,self.findCamInw)
		self.gazebo_info_topic = "/gazebo/model_states"
		self.register_subscriber(self.gazebo_info_topic,ModelStates,self.findTransform)

		self.pub_camTobj_topic = "/my_depth_camera/poseinobj"
		self.register_publisher(self.pub_camTobj_topic, PoseStamped)

	def findTransform(self,data):
		objName = "duck"
		camName = "depth_camera"

		objStatesPosition = data.name.index(objName)
		camStatesPosition = data.name.index(camName)

		objQuaternion = Quaternion(	data.pose[objStatesPosition].orientation.w,
									data.pose[objStatesPosition].orientation.x,
									data.pose[objStatesPosition].orientation.y,
									data.pose[objStatesPosition].orientation.z )
		camQuaternion = Quaternion(	data.pose[camStatesPosition].orientation.w,
									data.pose[camStatesPosition].orientation.x,
									data.pose[camStatesPosition].orientation.y,
									data.pose[camStatesPosition].orientation.z )
		
		self.obj_T_world = objQuaternion.transformation_matrix
		self.obj_T_world[0,3] = data.pose[objStatesPosition].position.x
		self.obj_T_world[1,3] = data.pose[objStatesPosition].position.y
		self.obj_T_world[2,3] = data.pose[objStatesPosition].position.z
		
		self.cam_T_world = camQuaternion.transformation_matrix
		self.cam_T_world[0,3] = data.pose[camStatesPosition].position.x
		self.cam_T_world[1,3] = data.pose[camStatesPosition].position.y
		self.cam_T_world[2,3] = data.pose[camStatesPosition].position.z

	def getRes(self,seq):
		#Let the z axis toward the obj:
		Rotate_y_quaternion = Quaternion(axis=[0,1,0],angle=np.pi/2)
		Rotate_y_mtx = Rotate_y_quaternion.transformation_matrix

		Rotate_z_quaternion = Quaternion(axis=[0,0,-1],angle=np.pi/2)
		Rotate_z_mtx = Rotate_z_quaternion.transformation_matrix
		
		# change the frame:
		#self.cam_T_obj = np.linalg.inv(self.obj_T_world) @ self.cam_T_world
		self.cam_T_obj = np.linalg.inv(self.obj_T_world) @ (self.cam_T_world)
		print('--------------------------------------------')
		print(self.cam_T_world)
		#self.cam_T_obj = self.cam_T_world @ Rotate_y_mtx
		obj_T_cam_inQuaternion = Quaternion( matrix=np.array(self.cam_T_obj) )

		# log the infor mation in (x,y,z) (r,p,y)
		#self.loginfo()

		# publishe the info
		pubMsg = PoseStamped()
		pubMsg.header.seq = seq
		pubMsg.header.stamp = rospy.Time.now()
		pubMsg.header.frame_id = "duck"

		pubMsg.pose.position.x = self.cam_T_obj[0][3]
		pubMsg.pose.position.y = self.cam_T_obj[1][3]
		pubMsg.pose.position.z = self.cam_T_obj[2][3]

		pubMsg.pose.orientation.x = obj_T_cam_inQuaternion.x
		pubMsg.pose.orientation.y = obj_T_cam_inQuaternion.y
		pubMsg.pose.orientation.z = obj_T_cam_inQuaternion.z
		pubMsg.pose.orientation.w = obj_T_cam_inQuaternion.w

		self.pubs[self.pub_camTobj_topic].publish(pubMsg)
	
	def loginfo(self):
		print("----------------------------------------------")
		print("Time is:", rospy.Time.now())
		## print Obj in world
		objXYZRPY = self.comput_xyz_RPY(self.obj_T_world)
		#print(self.obj_T_world)
		print("The object in world is:(x,y,z)=","%.4f" %objXYZRPY[0], "%.4f" %objXYZRPY[1], "%.4f" %objXYZRPY[2],
		 	  "(R,P,Y) =", "%.4f" %objXYZRPY[3], "%.4f" %objXYZRPY[4], "%.4f" %objXYZRPY[5])

		## Print cam in Wrold:
		camXYZRPY = self.comput_xyz_RPY(self.cam_T_world)
		#print(self.cam_T_world)
		print("The camera in world is:(x,y,z)=","%.4f" %camXYZRPY[0], "%.4f" %camXYZRPY[1], "%.4f" %camXYZRPY[2],
		 	  "(R,P,Y) =", "%.4f" %camXYZRPY[3], "%.4f" %camXYZRPY[4], "%.4f" %camXYZRPY[5])

		## print cam in obj:
		camInobjXYZRPY = self.comput_xyz_RPY(self.cam_T_obj)
		#print(self.cam_T_obj)
		print("The camera in object frame is:(x,y,z)=","%.4f" %camInobjXYZRPY[0], "%.4f" %camInobjXYZRPY[1], 
			  "%.4f" %camInobjXYZRPY[2],
		 	  "(R,P,Y) =", "%.4f" %camInobjXYZRPY[3], "%.4f" %camInobjXYZRPY[4], "%.4f" %camInobjXYZRPY[5])


	def comput_xyz_RPY(self,transformatrix):
		res = [0]*6
		x,y,z = transformatrix[0][3], transformatrix[1][3], transformatrix[2][3]
		quaternion = Quaternion( matrix = np.array(transformatrix))
		RPY = self.quaternion_to_RPY(quaternion.w , quaternion.x,
										quaternion.y , quaternion.z)
		res[:3]=[x,y,z]
		res[3:6]=[RPY[0],RPY[1],RPY[2]]

		return res


	def quaternion_to_RPY(self,w,x,y,z):
		RPY=[0]*3
		r = (180/math.pi) * math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
		p = (180/math.pi) * math.asin(2*(w*y-z*x))
		y = (180/math.pi) * math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
		RPY[:3] = [r,p,y]
		return RPY

if __name__ == '__main__':
	
	rospy.init_node("objIncam")
	objIncam_Node = objIncam()
	objIncam_Node.initNode()
	rospy.sleep(1);
	sequence = 0

	dt = rospy.Rate(900)#
	while not rospy.is_shutdown():
		objIncam_Node.getRes(sequence)
		sequence += 1
		dt.sleep()