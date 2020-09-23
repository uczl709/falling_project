import duckMove 
import moveCamera
from math import pi
from time import sleep


import rospy

# def set_init():
# 	camera = moveCamera.moveCamera()
# 	duck = duckMove.moveDuck()



if __name__ == "__main__":

	rospy.init_node("moveboth")

	# Set the initial postition:

	camera = moveCamera.moveCamera()
	duck = duckMove.moveDuck()
	rospy.sleep(1.0)

	duckPos = [0,0,2]
	duckOrt = duckMove.euler2quat([0,pi * (30/180), pi * (90/180)])

	cameraPos = [0 , 0 , 1]
	cameraOrt = duckMove.euler2quat( [0, -1.57, 0] )

	# # Init the pose:
	duck.set_pose(duckPos,duckOrt)
	camera.set_pose(cameraPos,cameraOrt)

	duck_px, duck_py, duck_pz = duckPos[0], duckPos[1], duckPos[2]
	duck_roll, duck_pitch, duck_yaw = 0 , pi * (30/180) , pi * (90/180)
	camera_px, camera_py, camera_pz = cameraPos[0] , cameraPos[1], cameraPos[2]
	camera_roll, camera_pitch, camera_yaw = 0, -1.57 ,0

	camera_v = 0.3
	dt = 0.001
	#w = 2.5
	#w = 1
	#w = 0.1
	w = 0.5
	#a = 9.8/10000
	a = 0
	duck_v = 0
	sumt = 0

	totaldistance = 0
	duck_pitch_sum = 0

	start_time = duck.get_time()
	while duck_pz>=0.9 and duck_pitch_sum<=9 and not rospy.is_shutdown():
		#move camera:
		#camera_py -= camera_v*dt
		totaldistance += camera_v*dt

		duck_pitch += 2.5*w*dt
		duck_pitch_sum += 2.5*w*dt

		duck_px += 0
		duck_py += 0
		duck_v = duck_v + a * dt
		duck_pz = duck_pz - duck_v*dt 
		sumt = sumt+dt
		print("duck_pz:%3f" % duck_pz,"v:%3f" % duck_v,"sum time:%3f" % sumt)
		# publish to the gazebo:

		camera.set_position([camera_px,camera_py,camera_pz])
		duck.set_pose([duck_px,duck_py,duck_pz],
			  duckMove.euler2quat([duck_roll,duck_pitch,duck_yaw]))
		#sleep(dt)

	end_time = duck.get_time()
	print("Camera moving distance:",totaldistance)
	print("Falling end and time cost:",end_time-start_time)
	print("Rotation angle in row {%4f} rad" %(duck_roll - 0) )
	print("Rotation angle in pitch {%4f} rad" %( (duck_pitch - pi * (30/180))*(180/pi) ),'in rad {%4f}' %(duck_pitch - pi * (30/180))  )
	print("Rotation angle in yaw {%4f} rad" %(duck_yaw - pi * (90/180)) )
	# reset-pose:
	duckPos = [0,0,2]
	duckOrt = duckMove.euler2quat([0,pi * (30/180), pi * (90/180)])

	cameraPos = [0 , 0 , 1]
	cameraOrt = duckMove.euler2quat( [0, -1.57, 0] )

	# # Init the pose:
	duck.set_pose(duckPos,duckOrt)
	#camera.set_pose(cameraPos,cameraOrt)
	camera.set_position([0,0,1])






