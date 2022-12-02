#!/usr/bin/env python3
import sys, select, tty, termios
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8

if __name__ == '__main__':

  #KeysトピックにString型のメッセージを毎秒一回送る
  #key_pub = rospy.Publisher('keys', String, queue_size=1)
  key_pub = rospy.Publisher('keys', UInt8, queue_size=1)
  #keyboard_driverノード作成
  rospy.init_node("keyboard_driver")
  rate = rospy.Rate(100)

  old_attr = termios.tcgetattr(sys.stdin)
  tty.setcbreak(sys.stdin.fileno())

  print ("Enter a Value between 000 and 180 degrees. Press Ctrl-C to exit...")

  while not rospy.is_shutdown():

    if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
        s=sys.stdin.read(3)
        if s.isdecimal() == True and int(s) >= 0 and int(s) <= 180:
            key_pub.publish(int(s))
            print(int(s))
        else:
            print('False')
    rate.sleep()

  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
