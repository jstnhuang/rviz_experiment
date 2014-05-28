from __future__ import division
from datetime import timedelta

NSECS_PER_SEC = 1000000000

def format_duration(duration, precision=2):
  """Returns the number of seconds in this duration as a string."""
  residual = round(duration.nsecs / NSECS_PER_SEC, precision)
  delta = timedelta(seconds=duration.secs + residual)
  return str(delta.total_seconds())
