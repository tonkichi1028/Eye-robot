#!/usr/bin/env python3
import rospy
import os,sys
import cv2
import numpy as np
import dlib
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from imutils import face_utils
from scipy.spatial import distance
from std_msgs.msg import UInt8

class Image_processing(object):
	def __init__(self):
		# Ros
		self.img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_Image)
		self.key_pub = rospy.Publisher('keys', UInt8, queue_size=1)
		self.bridge = CvBridge()

		# Path
		self.face_cascade = cv2.CascadeClassifier('/home/ubuntu/classwork/haarcascades/haarcascade_frontalface_alt2.xml')
		self.face_parts_detector = dlib.shape_predictor('/usr/share/dlib/shape_predictor_68_face_landmarks.dat')

		# Dry Eye
		self.dry_eye_count = 0
		self.dry_eye_time_stamp = 0
		self.flag_blink = 0

		# Timer
		self.time_start = 0
		self.time = 0
		
		
		
	def callback_Image(self, data):
		if self.time_start == 0:
			self.time_start = time.time()
		else:
			self.time = time.time() - self.time_start

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			eye_image = self.eye(cv_image)
			cv2.imshow("eye_image", eye_image)
			cv2.waitKey(1)

		except CvBridgeError as e:
			print(e)
			
			
			
	def calc_ear(self, eye):
		A = distance.euclidean(eye[1], eye[5])
		B = distance.euclidean(eye[2], eye[4])
		C = distance.euclidean(eye[0], eye[3])
		eye_ear = (A + B) / (2.0 * C)
		return round(eye_ear, 3)



	def eye_marker(self, face_mat, position):
		for i, ((x, y)) in enumerate(position):
			cv2.circle(face_mat, (x, y), 1, (255, 255, 255), -1)
			cv2.putText(face_mat, str(i), (x + 2, y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)



	def eye(self, image):
		tick = cv2.getTickCount()
		rgb = image
		gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    		
		faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.11, minNeighbors=3, minSize=(100, 100))
		
		if len(faces) == 1:
			x, y, w, h = faces[0, :]
			cv2.rectangle(rgb, (x, y), (x + w, y + h), (255, 0, 0), 2)

			face_gray = gray[y :(y + h), x :(x + w)]
			scale = 480 / h
			face_gray_resized = cv2.resize(face_gray, dsize=None, fx=scale, fy=scale)

			face = dlib.rectangle(0, 0, face_gray_resized.shape[1], face_gray_resized.shape[0])
			face_parts = self.face_parts_detector(face_gray_resized, face)
			face_parts = face_utils.shape_to_np(face_parts)

			left_eye = face_parts[42:48]
			self.eye_marker(face_gray_resized, left_eye)

			left_eye_ear = self.calc_ear(left_eye)
			cv2.putText(rgb, "LEFT eye EAR:{} ".format(left_eye_ear), (10, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1, cv2.LINE_AA)

			right_eye = face_parts[36:42]
			self.eye_marker(face_gray_resized, right_eye)

			right_eye_ear = self.calc_ear(right_eye)
			cv2.putText(rgb, "RIGHT eye EAR:{} ".format(round(right_eye_ear, 3)), (10, 120), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1, cv2.LINE_AA)



			if (left_eye_ear + right_eye_ear) < 0.35:
				cv2.putText(rgb,"Sleepy eyes. Wake up!", (10,180), cv2.FONT_HERSHEY_PLAIN, 3, (0,0,255), 3, 1)
				self.dry_eye_time_stamp = self.time
				
			self.dry_eye_count = self.time - self.dry_eye_time_stamp
			
			if self.dry_eye_count > 3.0 and self.flag_blink == 0:
				s = 153
				self.key_pub.publish(int(s))
				self.flag_blink = 1
		else:
			self.dry_eye_time_stamp = self.time
			self.flag_blink = 0
			
		fps = cv2.getTickFrequency() / (cv2.getTickCount() - tick)
		cv2.putText(rgb, "FPS:{} ".format(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 1, cv2.LINE_AA)
		cv2.putText(rgb, "Count:{} ".format(int(self.dry_eye_count)), (10, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2, cv2.LINE_AA)
		
			
		return rgb
		
		
		
	def Timer(self, event=None):
		if self.time_start == 0:
			self.time_start = time.Time()
		else:
			self.time = time.Time() - self.time_start



if __name__ == "__main__":
    rospy.init_node("image")
    Img = Image_processing()
    try:
        #rospy.Timer(rospy.Duration(1.0/100.0), Img.Timer)
        rospy.spin()
    except KeyboardInterrupt:
        pass
