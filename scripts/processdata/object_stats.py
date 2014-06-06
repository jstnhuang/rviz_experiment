import objects

class ObjectStats(object):
  """A double dictionary, mapping object names to a dictionary of stats."""
  def __init__(self):
    self._stats = {}
    for obj in objects.OBJECTS:
      self._stats[obj] = {}
    self._stats['all'] = {}

  def update(self, object_stats):
    """Merge with another ObjectStats instance."""
    for obj, stats in object_stats.items():
      self._stats[obj].update(stats)

  def items(self):
    return self._stats.items()

  def add(self, obj, key, value):
    if key in self._stats[obj]:
      self._stats[obj][key] += value
    else:
      self._stats[obj][key] = value

  def get_stats(self, obj):
    return self._stats[obj]
