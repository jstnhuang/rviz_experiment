#!/usr/bin/python

"""Processes rosbag data from the CSE 599K1 observational study.
"""
from __future__ import division
from __future__ import print_function

from collections import namedtuple
import genpy
import rosbag
import sys

NSECS_PER_SEC = 1000000000
SCREEN_WIDTH = 1920

GRIPPER_COMMAND_TYPE = 'pr2_controllers_msgs/Pr2GripperCommand'
MARKER_FEEDBACK_TYPE = 'visualization_msgs/InteractiveMarkerFeedback'
POSE_STAMPED_TYPE = 'geometry_msgs/PoseStamped'
POSE_TYPE = 'geometry_msgs/Pose'
SCREEN_POSITION_TYPE = 'rviz_experiment/ScreenPosition'

CAMERA_POSE_TOPIC = 'rviz_camera_publisher/camera_pose'
LEFT_GRASP_TOPIC = 'l_gripper_controller/command'
LEFT_POSITION_TOPIC = 'l_cart/command_pose'
RIGHT_GRASP_TOPIC = 'r_gripper_controller/command'
RIGHT_POSITION_TOPIC = 'r_cart/command_pose'
MARKER_FEEDBACK_TOPIC = 'pr2_marker_control_transparent/feedback'
GAZE_TOPIC = '/user_gaze_position'

Point = namedtuple('Point', ['x', 'y', 'z'])
Quaternion = namedtuple('Quaternion', ['w', 'x', 'y', 'z'])
Pose = namedtuple('Pose', ['position', 'orientation'])
Pr2GripperCommand = namedtuple('Pr2GripperCommand', ['position'])
InteractiveMarkerFeedback = namedtuple(
  'InteractiveMarkerFeedback',
  ['marker_name', 'control_name', 'event_type', 'pose']
)
ScreenPosition = namedtuple('ScreenPosition', ['x', 'y'])

def open_bag(filename):
  try:
    return rosbag.Bag(filename)
  except Exception as e:
    print(e)

def make_pose(message):
  position = Point(message.position.x, message.position.y, message.position.z)
  orientation = Quaternion(
    message.orientation.w,
    message.orientation.x,
    message.orientation.y,
    message.orientation.z
  )
  return Pose(position, orientation)

def model_factory(message):
  if message._type == POSE_TYPE:
    return make_pose(message)
  elif message._type == POSE_STAMPED_TYPE:
    return make_pose(message.pose)
  elif message._type == GRIPPER_COMMAND_TYPE:
    return Pr2GripperCommand(message.position)
  elif message._type == MARKER_FEEDBACK_TYPE:
    return InteractiveMarkerFeedback(
      message.marker_name,
      message.control_name,
      message.event_type,
      make_pose(message.pose)
    )
  elif message._type == SCREEN_POSITION_TYPE:
    return ScreenPosition(message.x, message.y)
  else:
    raise Exception('Unknown message type in model factory.')

class TimeTakenProcessor:
  def __init__(self):
    self._first_action = None
    self._first_time = None
    self._first_messages = {}
    self._last_action = None
    self._last_time = None
    self._last_messages = {}

  def update(self, topic, message, time):
    # The first message which is different from its predecessor on the same
    # topic is considered the first action.
    if self._first_action is None:
      if topic in self._first_messages:
        if self._first_messages[topic] != message:
          self._first_action = message
          self._first_time = time
      else:
        self._first_messages[topic] = message

    # The first message which is the same as all its successors on the same
    # topic is considered the last action on that topic. We keep track of the
    # last action across topics.
    if topic in self._last_messages:
      if self._last_messages[topic] != message:
        self._last_messages[topic] = message
        if self._last_time is None or time > self._last_time:
          self._last_action = message
          self._last_time = time
    else:
      self._last_messages[topic] = message

  def time_taken(self):
    return self._last_time - self._first_time

class CameraMovementTimeProcessor:
  def __init__(self):
    self._previous_message = None
    self._previous_time = None
    self._movement_time = genpy.Duration(0)

  def update(self, topic, message, time):
    if topic != CAMERA_POSE_TOPIC:
      return
    # Camera movement is updated continuously, so we just count up the time
    # between changes in camera position.
    if self._previous_message is not None and message != self._previous_message:
      self._movement_time += time - self._previous_time
    self._previous_message = message
    self._previous_time = time

  def movement_time(self):
    return self._movement_time

class MarkerMovementTimeProcessor:
  def __init__(self):
    self._pose_update_start = None
    self._pose_update_end = None
    self._movement_time = genpy.Duration(0)

  def update(self, topic, message, time):
    # The feedback topic is not updated continuously, so we look only at the
    # amount of time POSE_UPDATE events are published.
    if topic != MARKER_FEEDBACK_TOPIC:
      return
    if message.event_type == 1:
      if self._pose_update_start is None:
        self._pose_update_start = time
      self._pose_update_end = time
    else:
      if (
        self._pose_update_end is not None
        and self._pose_update_start is not None
      ):
        self._movement_time += self._pose_update_end - self._pose_update_start
        self._pose_update_start = None
        self._pose_update_end = None

  def movement_time(self):
    return self._movement_time

class GraspCountProcessor:
  def __init__(self):
    self.left_grasps = 0
    self.right_grasps = 0

  def update(self, topic, message, time):
    if topic == LEFT_GRASP_TOPIC:
      if message.position == 0:
        self.left_grasps += 1
    elif topic == RIGHT_GRASP_TOPIC:
      if message.position == 0:
        self.right_grasps += 1

  def num_grasps(self):
    return self.left_grasps + self.right_grasps

class GazeProcessor:
  def __init__(self):
    self._left_time = genpy.Duration(0)
    self._right_time = genpy.Duration(0)
    self._previous_gaze_time = None

  def update(self, topic, message, time):
    if topic != GAZE_TOPIC:
      return
    if self._previous_gaze_time is not None:
      duration = time - self._previous_gaze_time
      if message.x < SCREEN_WIDTH / 2:
        self._left_time += duration
      else:
        self._right_time += duration

    self._previous_gaze_time = time

  def left_time(self):
    return self._left_time

  def right_time(self):
    return self._right_time

def format_duration(duration, precision=2):
  """Returns the number of seconds in this duration as a string."""
  residual = round(duration.nsecs / NSECS_PER_SEC, precision)
  return '{}'.format(duration.secs + residual)

def process(bag):
  """Extract features of the bag file.

  Features include:
  - Time between first and last action
  - Number of direction switches for each gripper
  - Amount of time spent moving the camera
  - Amount of time spent moving a gripper
  - Amount of time spent idle
  - Number of gripper close actions
  """
  time_taken_processor = TimeTakenProcessor()
  camera_movement_time_processor = CameraMovementTimeProcessor()
  marker_movement_time_processor = MarkerMovementTimeProcessor()
  grasp_count_processor = GraspCountProcessor()
  gaze_processor = GazeProcessor()
  message_logs = {}
  for topic, message, time in bag.read_messages():
    model = model_factory(message)
    time_taken_processor.update(topic, model, time)
    camera_movement_time_processor.update(topic, model, time)
    marker_movement_time_processor.update(topic, model, time)
    grasp_count_processor.update(topic, model, time)
    gaze_processor.update(topic, model, time)
    if topic in message_logs:
      message_logs[topic].append((model, time))
    else:
      message_logs[topic] = [(model, time)]

  time_taken = time_taken_processor.time_taken()
  camera_movement_time = camera_movement_time_processor.movement_time()
  marker_time = marker_movement_time_processor.movement_time()
  num_grasps = grasp_count_processor.num_grasps()
  gaze_left_time = gaze_processor.left_time()
  gaze_right_time = gaze_processor.right_time()
  print('Time taken: {} seconds'.format(format_duration(time_taken)))
  print('Time spent moving the camera: {} seconds'.format(
    format_duration(camera_movement_time)))
  print('Time spent moving markers: {} seconds'.format(
    format_duration(marker_time)))
  print('Time spent staring: {} seconds'.format(
    format_duration(time_taken - camera_movement_time - marker_time)))
  print('Number of gripper grasps: {}'.format(num_grasps))
  print('Gaze left: {} seconds, right: {} seconds'.format(
    format_duration(gaze_left_time), format_duration(gaze_right_time)
  ))

def main():
  filename = sys.argv[1]
  bag = open_bag(filename)
  if bag is not None:
    data = process(bag)
    bag.close()

if __name__ == '__main__':
  main()
