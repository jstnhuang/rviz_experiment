from __future__ import division
from __future__ import print_function

import genpy
import topics

class TimeTaken:
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

class CameraMovementTime:
  def __init__(self):
    self._previous_message = None
    self._previous_time = None
    self._movement_time = genpy.Duration(0)

  def update(self, topic, message, time):
    if topic != topics.CAMERA_POSE:
      return
    # Camera movement is updated continuously, so we just count up the time
    # between changes in camera position.
    if self._previous_message is not None and message != self._previous_message:
      self._movement_time += time - self._previous_time
    self._previous_message = message
    self._previous_time = time

  def movement_time(self):
    return self._movement_time

class MarkerMovementTime:
  def __init__(self):
    self._pose_update_start = None
    self._pose_update_end = None
    self._movement_time = genpy.Duration(0)

  def update(self, topic, message, time):
    # The feedback topic is not updated continuously, so we look only at the
    # amount of time POSE_UPDATE events are published.
    if topic != topics.MARKER_FEEDBACK:
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

class GraspCount:
  def __init__(self):
    self._is_left_closed = True
    self._is_right_closed = True
    self.left_grasps = 0
    self.right_grasps = 0

  def update(self, topic, message, time):
    if topic == topics.LEFT_GRASP:
      if message.position == 0:
        if not self._is_left_closed:
          self.left_grasps += 1
        self._is_left_closed = True
      else:
        self._is_left_closed = False
    elif topic == topics.RIGHT_GRASP:
      if message.position == 0:
        if not self._is_right_closed:
          self.right_grasps += 1
        self._is_right_closed = True
      else:
        self._is_right_closed = False

  def num_grasps(self):
    return self.left_grasps + self.right_grasps

class Code:
  def __init__(self):
    self._left_time = genpy.Duration(0)
    self._right_time = genpy.Duration(0)
    self._other_time = genpy.Duration(0)
    self._squared_left = genpy.Duration(0)
    self._squared_right = genpy.Duration(0)
    self._num_left_looks = 0
    self._num_right_looks = 0
    self._state = 'other'
    self._prev_time = None
    self._timeline = []

  def update(self, topic, message, time):
    if topic != topics.CODING:
      return
    if self._prev_time is not None and message.data != self._state:
      delta = time - self._prev_time
      self._timeline.append((delta, self._state))
      if self._state == 'left':
        self._left_time += delta
        self._squared_left += genpy.Duration(delta.to_sec() * delta.to_sec())
        self._num_left_looks += 1
      elif self._state == 'right':
        self._right_time += delta
        self._squared_right += genpy.Duration(delta.to_sec() * delta.to_sec())
        self._num_right_looks += 1
      else:
        self._other_time += delta
    self._state = message.data
    self._prev_time = time

  def left_time(self):
    return self._left_time

  def right_time(self):
    return self._right_time

  def num_left_looks(self):
    return self._num_left_looks

  def num_right_looks(self):
    return self._num_right_looks

  def mean_left(self):
    return self._left_time / self._num_left_looks

  def left_stddev(self):
    mean_left = self.mean_left()
    squared_mean_left = genpy.Duration(
      mean_left.to_sec() * mean_left.to_sec()
    )
    left_stddev = self._squared_left / self._num_left_looks - squared_mean_left
    return left_stddev

  def mean_right(self):
    return self._right_time / self._num_right_looks

  def right_stddev(self):
    mean_right = self.mean_right()
    squared_mean_right = genpy.Duration(
      mean_right.to_sec() * mean_right.to_sec()
    )
    right_stddev = (
      self._squared_right / self._num_right_looks - squared_mean_right
    )
    return right_stddev

  def timeline(self):
    return self._timeline

class Objects:
  def __init__(self):
    self._timeline = []
    self._last_object = None
    self._last_start_time = 0
    self._last_end_time = 0

  def update(self, start_time, duration, side, obj):
    if self._last_object is not None and obj != self._last_object:
      self._timeline.append(
        (self._last_start_time, self._last_end_time, self._last_object)
      )
      self._last_start_time = start_time
    self._last_object = obj
    self._last_end_time = start_time + duration

  def update_last(self):
    self._timeline.append(
      (self._last_start_time, self._last_end_time, self._last_object)
    )

  def timeline(self):
    return self._timeline
