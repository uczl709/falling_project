from time import sleep
import numpy as np
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import rospy

bridge = CvBridge()

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



class checkInfo(ROSNode):
    def __init__(self):
        super(checkInfo, self).__init__()
        self.pose = None
        self.image_topic = "my_depth_camera/depth"
        self.pose_topic = "/my_depth_camera/poseinobj"
        self.fustion_frame = 0
        
        self.image_buffer = []
        self.pose_buffer = []

        self.register_subscriber(self.pose_topic, PoseStamped, self.get_pose)
        self.register_subscriber(self.image_topic, Image, self.get_imgae)

        self.root = "/home/wei/codebuket/pdd_tsdf/data/"
        self.T_BC = np.array( [ [0,0,1,0], [-1,0,0,0], [0,-1,0,0], [0,0,0,1] ] )

    def get_pose(self, msg):
      buffer_len = len(self.pose_buffer)
      if buffer_len>=2000: self.pose_buffer = self.pose_buffer[1:]
      self.pose_buffer.append(msg)
  
    def get_imgae(self, msg):
      buffer_len = len(self.image_buffer)
      if buffer_len >= 80:
        self.image_buffer = self.image_buffer[1:]
      self.image_buffer.append(msg)

    def info(self):
      print(len(self.pose_buffer),len(self.image_buffer))

    def check_image(self):
      msg = self.image_buffer.pop()
      encoding_type = msg.encoding
      print(encoding_type)
      cv_img = bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough")
      print("img info:")
      print(cv_img)

    def check_pose(self):
      for msg in self.pose_buffer:
        print("----------------------------------")
        print("Time:", msg.header.stamp)
        print("position:\n", msg.pose.position)
        print("orientation:\n", msg.pose.orientation)

    def sychrnized(self):
      if not self.pose_buffer or not self.image_buffer: return 

      current_image = self.image_buffer.pop(0)
      current_image_time = ( current_image.header.stamp ).to_nsec()
      pose_msg = self.query( current_image_time )

      #print( self.fustion_num, len(self.pose_buffer), len(self.image_buffer) )
      if pose_msg:
        self.fustion_frame += 1
        print( "-------------------" )
        print( self.fustion_frame, len(self.pose_buffer), len(self.image_buffer) )
        self.save( current_image, pose_msg )

    def query( self, depth_time):

      if ( self.pose_buffer[0].header.stamp ).to_nsec() > depth_time:
        print("error, need to increase the buffer")

      for msg in self.pose_buffer:
        pose_time = ( msg.header.stamp ).to_nsec()
        if pose_time == depth_time:
          return msg
        if pose_time > depth_time:
          return None

    def save( self, image, pose ):

      # write the pose.txt
      pose_name = "pose"+("%06d" % self.fustion_frame)+".out"
      matrix = quaternion_matrix( [pose.pose.orientation.x,
                                   pose.pose.orientation.y,
                                   pose.pose.orientation.z,
                                   pose.pose.orientation.w ] )
      matrix[0,3] = pose.pose.position.x
      matrix[1,3] = pose.pose.position.y
      matrix[2,3] = pose.pose.position.z 

      np.savetxt( self.root + pose_name , np.dot(matrix,self.T_BC) )

      # wirte the depth img:
      img_name = "depth"+( "%06d" % self.fustion_frame )+".png"
      cv_img = bridge.imgmsg_to_cv2(image,desired_encoding="passthrough" ) 
      cv2.imwrite( self.root + img_name , cv_img )


if __name__ == "__main__":

  rospy.init_node("check_info")
  obj = checkInfo()
  
  rospy.sleep(0.5)

  while not rospy.is_shutdown():
    obj.sychrnized()
    rospy.sleep(0.05)

  #sleep(1.0)
  #obj.info()
  #sleep(1.0)
  #obj.check_image()
  #obj.check_pose()





