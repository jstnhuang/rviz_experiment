#!/usr/bin/env python

"""ROS node that wraps around Open Gazer."""

import os
import re
import rospy
from rviz_experiment.msg import ScreenPosition
import subprocess

OPENGAZER_PATH = './opengazer'

class EyeTracker():
  def __init__(self):
    self._publisher = rospy.Publisher(
      'user_gaze_position',
      ScreenPosition
    )

  def start(self):
    self._process = subprocess.Popen(
      [OPENGAZER_PATH],
      stderr=subprocess.PIPE
    )
    while self._process.returncode is None:
      line = self._process.stderr.readline()
      results = re.match('(\d+) (\d+)', line)
      if results is not None:
        message = ScreenPosition()
        message.x = int(results.group(1))
        message.y = int(results.group(2))
        self._publisher.publish(message)

      if self._process.poll() is not None:
        return
    
def main():
  rospy.init_node('eye_tracker')
  tracker = EyeTracker()
  tracker.start()

if __name__ == '__main__':
  main()
