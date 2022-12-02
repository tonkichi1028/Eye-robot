#!/usr/bin/env python3
import rospy
import os,sys
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Image_processing(object):
	def __init__(self):
		self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
		self.bridge = CvBridge()
	
		self.cascade = cv2.CascadeClassifier('/home/ubuntu/classwork/haarcascades/haarcascade_frontalface_alt2.xml')
		self.eye_cascade = cv2.CascadeClassifier('/home/ubuntu/classwork/haarcascades/haarcascade_eye_tree_eyeglasses.xml')




	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			eye_image = self.eye(cv_image)
			cv2.imshow("eye_image", eye_image)

			cv2.waitKey(1)
		except CvBridgeError as e:
			print(e)





	def eye(self, image):
		rgb = image
		gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
		faces = self.cascade.detectMultiScale(gray, scaleFactor=1.11, minNeighbors=3, minSize=(100, 100))
		if len(faces) == 1:
			x, y, w, h = faces[0, :]
			cv2.rectangle(rgb, (x, y), (x + w, y + h), (255, 0, 0), 2)
			
			eyes_gray = gray[y : y + int(h/2), x : x + w]
			eyes = self.eye_cascade.detectMultiScale(
			eyes_gray, scaleFactor=1.11, minNeighbors=3, minSize=(8, 8))

			for ex, ey, ew, eh in eyes:
				cv2.rectangle(rgb, (x + ex, y + ey), (x + ex + ew, y + ey + eh), (255, 255, 0), 1)

			if len(eyes) == 0:
				cv2.putText(rgb,"Sleepy eyes. Wake up!",
					(10,100), cv2.FONT_HERSHEY_PLAIN, 3, (0,0,255), 2, cv2.LINE_AA)
		return rgb


if __name__ == "__main__":
    rospy.init_node("image")
    Img = Image_processing()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
