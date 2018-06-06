#!/usr/bin/env python
import rospy
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


def callback(image,data):
	print(image.header.stamp)
	print(data.header.stamp)
	#assert image.header.stamp == data.header.stamp
	try:
		# Convert your ROS Image message to OpenCV2
		cv2_img = bridge.imgmsg_to_cv2(image, "bgr8")
	except CvBridgeError, e:
		print(e)
	else:
		print("Find image")

	msg = data.objects.data
	if(len(msg)):
		print("Len: {}".format(len(msg)))
		j=0
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
			cv2.imshow("Teste", cv2_img)
			cv2.waitKey(30)
			j+=12
	else:
		print("No objects detected.\n")

def listener():
	rospy.init_node('listener', anonymous=True)

	images = message_filters.Subscriber('/xtion/rgb/image_raw', Image)
	object_detect = message_filters.Subscriber('/objectsStamped', ObjectsStamped)
	ts = message_filters.ApproximateTimeSynchronizer([images,object_detect],10,0.1)
	ts.registerCallback(callback)

	try:
		rospy.spin()
	except KeyboardInterrupt:        
		print "Shutting down"
		cv2.destroyAllWindows()

if __name__ == '__main__':
	listener()
		