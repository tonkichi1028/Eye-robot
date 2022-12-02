#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8

key_mapping = { 'z': 0, 'x': 90, 'a': 170, "170": 170, "900": 90 }

def keys_cb(msg, twist_pub):

  #if len(msg.data) == 0 or not key_mapping.has_key(msg.data):
  #if len(msg.data) == 0 or not key_mapping.get(msg.data):
  if len(msg.data) == 0 or msg.data > 180 or msg.data < 90:
    return # unknown key.

  deg_r = key_mapping[msg.data]
  deg_l = 135-(135-deg_r)
        
  r = UInt8()
  r = deg_r

  l = UInt8()
  l = deg_l
   
  
  publisher_R.publish(r)
  publisher_L.publish(l)

if __name__ == '__main__':
  #keys_to_servoノード作成
  rospy.init_node('keys_to_servo')
  #servoトピックにUInt16型のメッセージを送る
  
  publisher_R = rospy.Publisher('servo_R', UInt8, queue_size=1)
  publisher_L = rospy.Publisher('servo_L', UInt8, queue_size=1)
  rospy.Subscriber('keys', String, keys_cb, publisher_R)
  rospy.spin()
