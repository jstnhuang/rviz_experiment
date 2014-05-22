#!/usr/bin/python

"""Processes rosbag data from the CSE 599K1 observational study.
"""
from __future__ import division
from __future__ import print_function

from collections import namedtuple
from datetime import timedelta
import genpy
import rosbag
import sys

NSECS_PER_SEC = 1000000000
SECONDS_PER_MINUTE = 60
MINUTES_PER_HOUR = 60
HOURS_PER_DAY = 24

GRIPPER_COMMAND_TYPE = 'pr2_controllers_msgs/Pr2GripperCommand'
MARKER_FEEDBACK_TYPE = 'visualization_msgs/InteractiveMarkerFeedback'
POSE_STAMPED_TYPE = 'geometry_msgs/PoseStamped'
POSE_TYPE = 'geometry_msgs/Pose'
SCREEN_POSITION_TYPE = 'rviz_experiment/ScreenPosition'
STRING_TYPE = 'std_msgs/String'

CAMERA_POSE_TOPIC = 'rviz_camera_publisher/camera_pose'
LEFT_GRASP_TOPIC = 'l_gripper_controller/command'
LEFT_POSITION_TOPIC = 'l_cart/command_pose'
RIGHT_GRASP_TOPIC = 'r_gripper_controller/command'
RIGHT_POSITION_TOPIC = 'r_cart/command_pose'
MARKER_FEEDBACK_TOPIC = 'pr2_marker_control_transparent/feedback'
CODING_TOPIC = 'webcam_coding'

Point = namedtuple('Point', ['x', 'y', 'z'])
Quaternion = namedtuple('Quaternion', ['w', 'x', 'y', 'z'])
Pose = namedtuple('Pose', ['position', 'orientation'])
Pr2GripperCommand = namedtuple('Pr2GripperCommand', ['position'])
InteractiveMarkerFeedback = namedtuple(
  'InteractiveMarkerFeedback',
  ['marker_name', 'control_name', 'event_type', 'pose']
)
ScreenPosition = namedtuple('ScreenPosition', ['x', 'y'])
String = namedtuple('String', ['data'])

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
  elif message._type == STRING_TYPE:
    return String(message.data)
  else:
    return None

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
    self._is_left_closed = True
    self._is_right_closed = True
    self.left_grasps = 0
    self.right_grasps = 0

  def update(self, topic, message, time):
    if topic == LEFT_GRASP_TOPIC:
      if message.position == 0:
        if not self._is_left_closed:
          self.left_grasps += 1
        self._is_left_closed = True
      else:
        self._is_left_closed = False
    elif topic == RIGHT_GRASP_TOPIC:
      if message.position == 0:
        if not self._is_right_closed:
          self.right_grasps += 1
        self._is_right_closed = True
      else:
        self._is_right_closed = False

  def num_grasps(self):
    return self.left_grasps + self.right_grasps

def format_duration(duration, precision=2):
  """Returns the number of seconds in this duration as a string."""
  residual = round(duration.nsecs / NSECS_PER_SEC, precision)
  delta = timedelta(seconds=duration.secs + residual)
  return str(delta)
  #return '{}'.format(duration.secs + residual)

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
  message_logs = {}
  for topic, message, time in bag.read_messages():
    model = model_factory(message)
    if model is None:
      continue
    time_taken_processor.update(topic, model, time)
    camera_movement_time_processor.update(topic, model, time)
    marker_movement_time_processor.update(topic, model, time)
    grasp_count_processor.update(topic, model, time)
    if topic in message_logs:
      message_logs[topic].append((model, time))
    else:
      message_logs[topic] = [(model, time)]

  time_taken = time_taken_processor.time_taken()
  camera_movement_time = camera_movement_time_processor.movement_time()
  marker_time = marker_movement_time_processor.movement_time()
  num_grasps = grasp_count_processor.num_grasps()
  print('Time taken: {}'.format(format_duration(time_taken)))
  print('Time spent moving the camera: {}'.format(
    format_duration(camera_movement_time)))
  print('Time spent moving markers: {}'.format(
    format_duration(marker_time)))
  print('Time spent staring: {}'.format(
    format_duration(time_taken - camera_movement_time - marker_time)))
  print('Number of gripper grasps: {}'.format(num_grasps))

class CodeProcessor:
  BASE_HTML = '''
    <!doctype html>
    <html>
      <head>
        <title>{title}</title>
        <style>
          body {{
            font-family: Sans-serif;
          }}
          .graphbox {{
            align-items: flex-end;
            color: white;
            display: flex;
            justify-content: center;
            width: 200px;
          }}
          .left {{
            background-color: #1049A9;
          }}
          .other {{
            background-color: #707070;
          }}
          .right {{
            background-color: #680BAB;
          }}
        </style>
      </head>
      <body>
        <h1>{title}</h1>
        {stats_table}
        <h2>Visualization</h2>
        {graph}
      </body>
    </html>
  '''

  STATS_TABLE = '''
    <table class="stats">
      <tr><td>Time spent looking left</td><td>{left_time}</td></tr>
      <tr><td>Time spent looking right</td><td>{right_time}</td></tr>
      <tr><td># of left looks</td><td>{left_looks}</td></tr>
      <tr><td># of right looks</td><td>{right_looks}</td></tr>
      <tr>
        <td>Mean time looking left (stddev)</td>
        <td>{mean_left} ({left_stddev})</td>
      </tr>
      <tr>
        <td>Mean time looking right (stddev)</td>
        <td>{mean_right} ({right_stddev})</td>
      </tr>
    </table> 
  '''
  def __init__(self):
    self._left_time = genpy.Duration(0)
    self._right_time = genpy.Duration(0)
    self._squared_left = genpy.Duration(0)
    self._squared_right = genpy.Duration(0)
    self._num_left_looks = 0
    self._num_right_looks = 0
    self._state = 'other'
    self._prev_time = None
    self._graph_html = '<p>Left = blue, right = purple, other=gray.</p><br />'

  def update(self, topic, message, time):
    if topic != CODING_TOPIC:
      return
    if self._prev_time is not None and message.data != self._state:
      delta = time - self._prev_time
      if self._state == 'left':
        self._left_time += delta
        self._squared_left += genpy.Duration(delta.to_sec() * delta.to_sec())
        self._num_left_looks += 1
      elif self._state == 'right':
        self._right_time += delta
        self._squared_right += genpy.Duration(delta.to_sec() * delta.to_sec())
        self._num_right_looks += 1
      height = round(delta.to_sec(), 1) * 10
      box_text = (
        '' if height < 20
        else format_duration(self._left_time + self._right_time)
      )
      self._graph_html += (
        '<div class="graphbox {}" style="height: {}px;">'
        '{}</div>'.format(
          self._state,
          height,
          box_text
        )
      )
    self._state = message.data
    self._prev_time = time

  def _generate_stats_table_html(self):
    mean_left = self._left_time / self._num_left_looks
    squared_mean_left = genpy.Duration(mean_left.to_sec() * mean_left.to_sec())
    left_stddev = self._squared_left / self._num_left_looks - squared_mean_left
    mean_right = self._right_time / self._num_right_looks
    squared_mean_right = genpy.Duration(
      mean_right.to_sec() * mean_right.to_sec()
    )
    right_stddev = (
      self._squared_right / self._num_right_looks - squared_mean_right
    )
    return self.STATS_TABLE.format(
      left_time=format_duration(self._left_time),
      right_time=format_duration(self._right_time),
      left_looks=self._num_left_looks,
      right_looks=self._num_right_looks,
      mean_left=format_duration(self._left_time / self._num_left_looks),
      left_stddev=format_duration(left_stddev),
      mean_right=format_duration(self._right_time / self._num_right_looks),
      right_stddev=format_duration(right_stddev)
    )

  def _generate_graph_html(self):
    return self._graph_html

  def generate_visualization(self, filename):
    name_parts = filename.split('_')
    title = 'Visualization for {}'.format(filename)
    with open(filename, 'w') as output_file:
      stats_table_html = self._generate_stats_table_html()
      graph_html = self._generate_graph_html()
      html = self.BASE_HTML.format(
        title=title,
        stats_table=stats_table_html,
        graph=graph_html
      )
      print(html, file=output_file)

def process_code(bag):
  """Extract features from a webcam coding."""
  code_processor = CodeProcessor()
  for topic, message, time in bag.read_messages():
    model = model_factory(message)
    if model is None:
      continue
    code_processor.update(topic, model, time) 
  code_processor.generate_visualization(bag.filename.replace('.bag', '.html'))

def main():
  filename = sys.argv[1]
  bag = open_bag(filename)
  if bag is not None:
    if filename.endswith('_code.bag'):
      process_code(bag)
    else:
      process(bag)
    bag.close()

if __name__ == '__main__':
  main()
