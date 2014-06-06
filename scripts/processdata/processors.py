from __future__ import division
from __future__ import print_function

import objects
from object_stats import ObjectStats
from object_timeline import ObjectTimeline

import topics

class TimeTaken(object):
  def __init__(self, object_timeline):
    self._first_action = None
    self._first_time = None
    self._first_messages = {}
    self._raw_first_time = None
    self._last_action = None
    self._last_time = None
    self._last_messages = {}
    self._raw_last_time = None
    self._object_stats = ObjectStats()
    self._object_timeline = object_timeline
    self._compute_object_stats()

  def _compute_object_stats(self):
    for obj in objects.OBJECTS:
      self._object_stats.add(obj, 'time_taken', 0)
    for event in self._object_timeline._timeline:
      duration = event.end_time - event.start_time
      self._object_stats.add(event.obj, 'time_taken', duration)

  def update(self, topic, message, time):
    time = time.to_sec()

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

    # Keep track of absolute first and last message.
    if self._raw_first_time is None:
      self._raw_first_time = time
    self._raw_last_time = time

  def update_last(self):
    first_obj = self._object_timeline.object_at(self._first_time -
        self._raw_first_time)
    leading_time = self._first_time - self._raw_first_time
    self._object_stats.add(first_obj, 'time_taken', -leading_time)

    last_obj = self._object_timeline.object_at(self._last_time -
        self._raw_first_time)
    trailing_time = self._last_time - self._raw_last_time
    self._object_stats.add(last_obj, 'time_taken', -trailing_time)

  def object_stats(self):
    return self._object_stats

class CameraMovementTime(object):
  def __init__(self, object_timeline):
    self._previous_message = None
    self._previous_time = None
    self._object_stats = ObjectStats()
    self._object_timeline = object_timeline
    self._raw_first_time = None
    for obj in objects.OBJECTS:
      self._object_stats.add(obj, 'camera_movement_time', 0)

  def update(self, topic, message, time):
    time = time.to_sec()
    if self._raw_first_time is None:
      self._raw_first_time = time

    if topic != topics.CAMERA_POSE:
      return
    # Camera movement is updated continuously, so we just count up the time
    # between changes in camera position.
    obj = self._object_timeline.object_at(time - self._raw_first_time)
    if self._previous_message is not None and message != self._previous_message:
      duration = time - self._previous_time
      self._object_stats.add(obj, 'camera_movement_time', duration)
    self._previous_message = message
    self._previous_time = time

  def object_stats(self):
    return self._object_stats

class MarkerMovementTime(object):
  def __init__(self, object_timeline):
    self._pose_update_start = None
    self._pose_update_end = None
    self._object_stats = ObjectStats()
    self._object_timeline = object_timeline
    self._raw_first_time = None
    for obj in objects.OBJECTS:
      self._object_stats.add(obj, 'marker_movement_time', 0)

  def update(self, topic, message, time):
    time = time.to_sec()
    if self._raw_first_time is None:
      self._raw_first_time = time

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
        obj = self._object_timeline.object_at(time - self._raw_first_time)
        duration = self._pose_update_end - self._pose_update_start
        self._object_stats.add(obj, 'marker_movement_time', duration)
        self._pose_update_start = None
        self._pose_update_end = None

  def object_stats(self):
    return self._object_stats

class GraspCount(object):
  def __init__(self, object_timeline):
    self._is_left_closed = True
    self._is_right_closed = True
    self.left_grasps = 0
    self.right_grasps = 0
    self._object_stats = ObjectStats()
    self._object_timeline = object_timeline
    self._raw_first_time = None
    for obj in objects.OBJECTS:
      self._object_stats.add(obj, 'grasp_count', 0)

  def update(self, topic, message, time):
    time = time.to_sec()
    if self._raw_first_time is None:
      self._raw_first_time = time

    obj = None
    if topic == topics.LEFT_GRASP or topic == topics.RIGHT_GRASP:
      obj = self._object_timeline.object_at(time - self._raw_first_time)
    if topic == topics.LEFT_GRASP:
      if message.position == 0:
        if not self._is_left_closed:
          self._object_stats.add(obj, 'grasp_count', 1)
        self._is_left_closed = True
      else:
        self._is_left_closed = False
    elif topic == topics.RIGHT_GRASP:
      if message.position == 0:
        if not self._is_right_closed:
          self._object_stats.add(obj, 'grasp_count', 1)
        self._is_right_closed = True
      else:
        self._is_right_closed = False

  def object_stats(self):
    return self._object_stats

class Code(object):
  def __init__(self, object_timeline):
    self._left_time = ObjectStats()
    self._right_time = ObjectStats()
    self._other_time = ObjectStats()
    self._squared_left = ObjectStats()
    for obj in objects.OBJECTS:
      self._squared_left.add(obj, 'squared_left', 0)
    self._squared_right = ObjectStats()
    for obj in objects.OBJECTS:
      self._squared_right.add(obj, 'squared_right', 0)
    self._num_left_looks = ObjectStats()
    self._num_right_looks = ObjectStats()
    self._state = 'other'
    self._prev_time = None
    self._timeline = []
    self._object_timeline = object_timeline
    self._raw_first_time = None

  def update(self, topic, message, time):
    time = time.to_sec()
    if self._raw_first_time is None:
      self._raw_first_time = time

    if topic != topics.CODING:
      return

    if self._prev_time is not None and message.data != self._state:
      obj = self._object_timeline.object_at(time - self._raw_first_time)
      delta = time - self._prev_time
      self._timeline.append((delta, self._state))
      if self._state == 'left':
        self._left_time.add(obj, 'left_time', delta)
        self._squared_left.add(obj, 'squared_left', delta*delta)
        self._num_left_looks.add(obj, 'num_left_looks', 1)
      elif self._state == 'right':
        self._right_time.add(obj, 'right_time', delta)
        self._squared_right.add(obj, 'squared_right', delta*delta)
        self._num_right_looks.add(obj, 'num_right_looks', 1)
      else:
        self._other_time.add(obj, 'other_time', delta)
    self._state = message.data
    self._prev_time = time

  def object_stats(self):
    object_stats = ObjectStats()
    object_stats.update(self._left_time)
    object_stats.update(self._num_left_looks)
    object_stats.update(self._right_time)
    object_stats.update(self._num_right_looks)
    init_features = [
      'num_left_looks', 'left_time', 'mean_left', 'left_stddev',
      'num_right_looks', 'right_time', 'mean_right', 'right_stddev'
    ]
    for obj in objects.OBJECTS:
      for feature in init_features:
        object_stats.add(obj, feature, 0)
    for obj, stats in object_stats.items():
      num_left_looks = stats['num_left_looks']
      left_time = stats['left_time']
      mean_left = left_time / num_left_looks if num_left_looks != 0 else 0
      stats['mean_left'] = mean_left

      squared_left = self._squared_left.get_stats(obj)['squared_left']
      squared_mean = mean_left * mean_left
      stddev = (
        squared_left / num_left_looks - squared_mean
        if num_left_looks != 0 else 0
      )
      stats['left_stddev'] = stddev

      num_right_looks = stats['num_right_looks']
      right_time = stats['right_time']
      mean_right = right_time / num_right_looks if num_right_looks != 0 else 0
      stats['mean_right'] = mean_right

      squared_right = self._squared_right.get_stats(obj)['squared_right']
      squared_mean = mean_right * mean_right
      stddev = (
        squared_right / num_right_looks - squared_mean
        if num_right_looks != 0 else 0
      )
      stats['right_stddev'] = stddev
    return object_stats

  def timeline(self):
    return self._timeline

class Objects(object):
  def __init__(self):
    self._timeline = ObjectTimeline()
    self._last_object = None
    self._last_start_time = 0
    self._last_end_time = 0

  def update(self, start_time, duration, side, obj):
    if self._last_object is not None and obj != self._last_object:
      self._timeline.add_event(
        self._last_start_time, self._last_end_time, self._last_object
      )
      self._last_start_time = start_time
    self._last_object = obj
    self._last_end_time = start_time + duration

  def update_last(self):
    self._timeline.add_event(
      self._last_start_time, self._last_end_time, self._last_object
    )

  def timeline(self):
    return self._timeline
