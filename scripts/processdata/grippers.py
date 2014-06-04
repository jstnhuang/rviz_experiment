#!/usr/bin/env python
"""Processes rosbag data from the CSE 599K1 observational study.
"""
from __future__ import print_function

import message_factory
import topics

from collections import namedtuple
import os
import rosbag
import rospy
from std_msgs.msg import String
import subprocess
import sys
import yaml

def open_bag(filename):
  try:
    return rosbag.Bag(filename)
  except Exception as e:
    print(e)

Event = namedtuple('Event', ['start_time', 'duration', 'side'])
class GripperEventProcessor(object):
  EVENT_SPACING = 5 # Minimum time, in seconds, between events.
  START_EVENT = 'Start event'
  WAIT_FOR_END = 'Waiting for event to end'

  def __init__(self, bag_start_time):
    self._bag_start_time = bag_start_time
    self._state = 'waiting for start'
    self._events = []
    self._event_start = None
    self._last_message_time = None

  def update(self, topic, model, time):
    if not topic == topics.MARKER_FEEDBACK:
      return
    if not (model.marker_name == 'l_gripper_control' or model.marker_name ==
        'r_gripper_control'):
      return

    if self._event_start is None:
      self._event_start = time.to_sec()
    elif time.to_sec() - self._last_message_time > self.EVENT_SPACING:
      start_time = self._event_start - self._bag_start_time
      duration = time.to_sec() - self._bag_start_time - start_time
      side = 'left' if model.marker_name == 'l_gripper_control' else 'right'
      self._events.append(Event(start_time, duration, side))
      self._event_start = None

    self._last_message_time = time.to_sec()

  def events(self):
    return self._events

class GripperPlayback(object):
  def __init__(self, nocam_path, cam_path, output_path):
    rospy.init_node('gripper_playback')
    self._nocam_path = nocam_path
    self._cam_path = cam_path
    self._output_path = output_path
    self._subscriber = rospy.Subscriber(
      'gripper_playback/objects', String, self._object_callback
    )
    self._events = []
    self._num_written = 0

  def _object_callback(self, data):
    obj_name = data.data
    last_event = self._events[-1]
    output_file = open(self._output_path, 'a')
    print(
      '\t'.join(
        [
          str(last_event.start_time),
          str(last_event.duration),
          last_event.side,
          obj_name
        ]
      ),
      file=output_file
    )
    self._num_written += 1

  def _get_start_time(self, path):
    info_dict = yaml.load(
      subprocess.Popen(
        ['rosbag', 'info', '--yaml', path],
        stdout=subprocess.PIPE
      ).communicate()[0]
    )
    return float(info_dict['start'])

  def start(self):
    bag = open_bag(self._nocam_path)
    bag_start_time = self._get_start_time(self._cam_path)

    gripper_event_processor = GripperEventProcessor(bag_start_time)
    for topic, message, time in bag.read_messages():
      model = message_factory.model(message)
      if model is None:
        continue
      gripper_event_processor.update(topic, model, time) 
    bag.close()

    events = gripper_event_processor.events()
    for i, event in enumerate(events):
      self._events.append(event)
      rospy.logwarn('Event {} of {}'.format(i+1, len(events)))
      rospy.logwarn('Start time: {}'.format(event.start_time))
      rospy.logwarn('Duration: {}'.format(event.duration))
      subprocess.call(
        [
          'rosbag', 'play', self._cam_path,
          '--topics', topics.KINECT_IMAGE,
          '--start', str(event.start_time),
          '--duration', str(event.duration),
          '--rate=8'
        ]
      )
      has_prompted = False
      while self._num_written < i+1:
        if not has_prompted:
          rospy.logwarn('Publish the object name to gripper_playback/objects.')
          has_prompted = True

def main():
  nocam_path = sys.argv[1]
  cam_path = sys.argv[2]
  output_path = sys.argv[3]
  playback = GripperPlayback(nocam_path, cam_path, output_path)
  playback.start()

if __name__ == '__main__':
  main()
