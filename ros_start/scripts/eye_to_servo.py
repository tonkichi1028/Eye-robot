#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import UInt8

def servo_param(deg):
  deg_r = deg
  deg_l = 180-deg_r-5 # left is -5
  
  r = UInt8()
  r = deg_r

  l = UInt8()
  l = deg_l
  
  return r,l

def keys_cb(msg, twist_pub):

  #if len(msg.data) == 0 or not key_mapping.has_key(msg.data):
  #if len(msg.data) == 0 or not key_mapping.get(msg.data):
  if msg.data == msg.data > 180 or msg.data < 0:
    return # unknown key.
    
  r,l = servo_param(msg.data)
  publisher_R.publish(r)
  publisher_L.publish(l)
  
  time.sleep(0.5)
  r,l = servo_param(62)
  
  publisher_R.publish(r)
  publisher_L.publish(l)
  print("go")

if __name__ == '__main__':
  #keys_to_servoノード作成
  rospy.init_node('keys_to_servo')
  #servoトピックにUInt16型のメッセージを送る
  
  publisher_R = rospy.Publisher('servo_R', UInt8, queue_size=1)
  publisher_L = rospy.Publisher('servo_L', UInt8, queue_size=1)
  rospy.Subscriber('keys', UInt8, keys_cb, publisher_R)
  rospy.spin()
