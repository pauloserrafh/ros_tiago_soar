#!/usr/bin/env python
import rospy
import sys
import cv2
from message_filters import Subscriber
from std_msgs.msg import String, Float32MultiArray
from PyQt4.QtGui import QTransform
from PyQt4.QtCore import QPointF
import message_filters 

def callback(data):

	msg = data.data
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
			j+=12
	else:
		print("No objects detected.\n")

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/objects",Float32MultiArray, callback)
	
	try:
		rospy.spin()
	except KeyboardInterrupt:        
		print "Shutting down"
	
if __name__ == '__main__':
	listener()