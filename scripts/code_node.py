#!/usr/bin/env python

"""Node for coding whether the user looked left or right in a video."""

import getch
import rosbag
import rospy
from std_msgs.msg import String
import sys

def main():
  data_dir = '/home/jstn/Dropbox/experiment_data'
  user = sys.argv[1]
  condition = sys.argv[2]
  filename = '{}_{}_code.bag'.format(user, condition)
  file_path = '/'.join([data_dir, filename])
  bag = rosbag.Bag(file_path, 'w')
  rospy.init_node('webcam_coder')
  rate = rospy.Rate(15)
  print(
    'Press \'l\' if the participant is looking left, '
    '\'r\' if the participant is looking right, and any other key for other. '
    'Press \'e\' to end.')
  state = 'other'
  time = rospy.get_time()
  print('{}: {}'.format(time, state))
  message = String()
  message.data = state
  bag.write('webcam_coding', message)

  while not rospy.is_shutdown():
    char = getch.getch()
    if char == 'l':
      new_state = 'left'
    elif char == 'r':
      new_state = 'right'
    elif char == 'e':
      print('Writing to file: {}'.format(file_path))
      bag.close()
      return
    else:
      new_state = 'other'
    if state != new_state:
      state = new_state
      time = rospy.get_time()
      print('{}: {}'.format(time, state))
      message = String()
      message.data = state
      bag.write('webcam_coding', message)
    rate.sleep()

if __name__ == '__main__':
  main()
