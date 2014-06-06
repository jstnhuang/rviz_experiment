from collections import namedtuple

Event = namedtuple('Event', ['start_time', 'end_time', 'obj'])

class ObjectTimeline(object):

  def __init__(self):
    self._timeline = []

  def add_event(self, start_time, end_time, obj):
    self._timeline.append(Event(start_time, end_time, obj))

  def object_at(self, time):
    """Returns the object the user was focusing on at the given time.

    Naively just walks the list of events. Could be a binary search or
    consistent hash if it's too slow.
    """
    for event in self._timeline: 
      if time >= event.start_time and time <= event.end_time:
        return event.obj
    return self._timeline[-1].obj
