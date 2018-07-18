#!/usr/bin/env python
import sys
import time
# ROS imports
import rospy
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

import rospy
import roslaunch
import sys
import cv2
from message_filters import TimeSynchronizer, Subscriber
from std_msgs.msg import String, Float32MultiArray
from PyQt4.QtGui import QTransform
from PyQt4.QtCore import QPointF
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters 
from find_object_2d.msg import ObjectsStamped
# Instantiate CvBridge
bridge = CvBridge()

def wait_for_valid_time(timeout):
	"""Wait for a valid time (non-zero), this is important
	when using a simulated clock"""
	# Loop until:
	# * ros master shutdowns
	# * control+C is pressed (handled in is_shutdown())
	# * timeout is achieved
	# * time is valid
	start_time = time.time()
	while not rospy.is_shutdown():
		if not rospy.Time.now().is_zero():
			return
		if time.time() - start_time > timeout:
			rospy.logerr("Timed-out waiting for valid time.")
			exit(0)
		time.sleep(0.1)
	# If control+C is pressed the loop breaks, we can exit
	exit(0)


def get_status_string(status_code):
	return GoalStatus.to_string(status_code)

class TiagoObjectDetection:
	def __init__(self, objectsFolder ='~/objects/'):
		rospy.init_node('run_motion_python')
		rospy.loginfo("Starting run_motion_python application...")
		wait_for_valid_time(10.0)
		self.client = SimpleActionClient('/play_motion', PlayMotionAction)

		rospy.loginfo("Waiting for Action Server...")
		self.client.wait_for_server()
		self.image = None
		self.fullPath = objectsFolder
		self.objectPath = None # Initially we dont have an object
		self.objectFound = 0

	def act(self, action = "detectObject", objectPath = ""):
		print(action)
		goal = PlayMotionGoal()
		goal.motion_name = action
		goal.skip_planning = False
		goal.priority = 0  # Optional
		self.objectPath = objectPath

		if (action == 'detectObject'):

			self.writeXML()
			
			cli_args = ['objectDetectRoslaunch.launch']			
			roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0])]
			print(roslaunch_file)
			uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)    
			roslaunch.configure_logging(uuid)
			launch = roslaunch.parent.ROSLaunchParent(uuid,  roslaunch_file)

			launch.start()

			image = rospy.wait_for_message("/xtion/rgb/image_raw", Image, 30.0)
			object_detect = rospy.wait_for_message('/objectsStamped', ObjectsStamped, 30.0)
			self.doObjectDetect(image,object_detect)		

			self.exitRoslaunch()
			launch.shutdown()
		elif (action == 'preSearchObject'):
			print("preSearchObject")
		elif (action == 'wave'):

			rospy.loginfo("Sending goal with motion: " + goal.motion_name)
			self.client.send_goal(goal)

			rospy.loginfo("Waiting for result...")
			action_ok = self.client.wait_for_result(rospy.Duration(30.0))

			state = self.client.get_state()

	def reset(self):
		pass

	def exitRoslaunch(self):
		print("Good bye")
 
	# Ros indigo - roslaunch doesnt support xml parameter, so it is a way to do it.
	def writeXML(self):
		with open("objectDetectRoslaunch.launch", "w") as f:
			string="<launch><!-- Nodes --><node name=\"find_object_2d\" pkg=\"find_object_2d\" type=\"find_object_2d\" output=\"screen\"><remap from=\"image\" to=\"/xtion/rgb/image_raw\"/><param name=\"gui\" value=\"false\" type=\"bool\"/><param name=\"objects_path\" value=\""+ str(self.fullPath + self.objectPath) + "\" type=\"str\"/><param name=\"settings_path\" value=\"~/.ros/find_object_2d.ini\" type=\"str\"/></node></launch>"
			f.write(string)

	def doObjectDetect(self,image,object_detect):
		try:
			# Convert your ROS Image message to OpenCV2
			cv2_img = bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError, e:
			print(e)
		else:
			print("Find image")

		msg = object_detect.objects.data
		if(len(msg)):
			print("Len: {}".format(len(msg)))
			j = 0
			while j < len(msg):
				idx = msg[j]
				objectWidth = msg[j+1]
				objectHeight = msg[j+2]
				
				# Find corners Qt
				qtHomography = QTransform(msg[j+3], msg[j+4], msg[j+5],
				msg[j+6], msg[j+7], msg[j+8],
				msg[j+9], msg[j+10], msg[j+11])
				
				qtTopLeft = qtHomography.map(QPointF(0,0));
				qtTopRight = qtHomography.map(QPointF(objectWidth,0));
				qtBottomLeft = qtHomography.map(QPointF(0,objectHeight));
				qtBottomRight = qtHomography.map(QPointF(objectWidth,objectHeight));
				
				print("Object {} detected, Qt corners at ({},{}) ({},{}) ({},{}) ({},{})\n".format(idx,
				qtTopLeft.x(), qtTopLeft.y(),
				qtTopRight.x(), qtTopRight.y(),
				qtBottomLeft.x(), qtBottomLeft.y(),
				qtBottomRight.x(), qtBottomRight.y()))
				topleft = (int(qtTopLeft.x()), int(qtTopLeft.y()))
				bottomright = (int(qtBottomRight.x()), int(qtBottomRight.y()))
				cv2.rectangle(cv2_img, topleft, bottomright,(255,255,255),3)
				#cv2.imshow("Teste", cv2_img)
				#cv2.waitKey(30)
				j+=12
				self.objectFound = 1
		else:
			self.objectFound = 0
			print("No objects detected.\n")


		self.image = cv2_img


	def getImage(self):
		return self.image

	def getObjectFoundFlag(self):
		return self.objectFound

if __name__ == '__main__':

	while(True):
		robot = TiagoObjectDetection()
		#robot.act(action = "detectObject", objectPath='ball')
		objectName = raw_input("Write the name of the object to find: ")
		robot.act(action = "detectObject", objectPath=objectName)
		robot.reset()
		cv2.imshow("Teste", robot.getImage())
		cv2.waitKey(30)

		# if object was found you can check it
		# print (robot.getObjectFoundFlag())

		# if(robot.getObjectFoundFlag())
			#cv2.imshow("Teste", robot.getImage())
			#cv2.waitKey(30)
			# SET OUTPUT LINK WITH OBJECT FOUND IN IMAGE
		# else
			# SET OUT PUT LINK WITH OBJECT NOT FOUND IN IMAGE




