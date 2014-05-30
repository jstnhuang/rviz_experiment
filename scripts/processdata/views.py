from __future__ import division

import features
import genpy
import utils

class Page(object):
  BASE_HTML = '''
    <!doctype html>
    <html>
      <head>
        <title>{title}</title>
        <link rel="stylesheet"
        href="http://netdna.bootstrapcdn.com/bootstrap/3.1.1/css/bootstrap.min.css">
        <link rel="stylesheet"
        href="http://netdna.bootstrapcdn.com/bootstrap/3.1.1/css/bootstrap-theme.min.css">
        <script src="http://ajax.googleapis.com/ajax/libs/jquery/1.11.1/jquery.min.js"></script>
        <script
        src="http://netdna.bootstrapcdn.com/bootstrap/3.1.1/js/bootstrap.min.js"></script>
        <style>
          .progress-bar {{
            color: black;
          }}
        </style>
      </head>
      <body>
        <div class="container">
          <h1>{title}</h1>
          {data_section}
          {personal_section}
          {trouble_section}
          {pcl_section}
          {cam_section}
          {timeline_section}
        </div>
      </body>
    </html>
  '''
  def __init__(
    self, title, data_section, personal_section, trouble_section, pcl_section,
    cam_section, timeline_section
  ):
    self._title = title
    self._data_section = data_section
    self._personal_section = personal_section
    self._trouble_section = trouble_section
    self._pcl_section = pcl_section
    self._cam_section = cam_section
    self._timeline_section = timeline_section

  def generate(self):
    return Page.BASE_HTML.format(
      title=self._title,
      data_section=self._data_section.generate(),
      personal_section=self._personal_section.generate(),
      trouble_section=self._trouble_section.generate(),
      pcl_section=self._pcl_section.generate(),
      cam_section=self._cam_section.generate(),
      timeline_section=self._timeline_section.generate()
    )

class Section(object):
  """A section consists of a section title and a data table."""
  SECTION_HTML = '''
  <h2>{title}</h2>
  {table}
  '''
  def __init__(self, title, table):
    self._title = title
    self._table = table

  def generate(self):
    return Section.SECTION_HTML.format(
      title=self._title,
      table=self._table.generate()
    )

class Table(object):
  TABLE_HTML = '''
    <table class="table">
      {header}
      {rows}
    </table>
  '''
  ROW_HTML = '<tr>{cells}</tr>'
  HEADER_HTML = '<th>{title}</th>'
  CELL_HTML = '<td>{value}</td>'

  def __init__(self, table_spec, data):
    self._table_spec = table_spec
    self._data = data

  def _generate_rows(self):
    rows = []
    for data_row in self._data:
      rows.append(self._table_spec.generate_row(data_row))
    return ''.join(rows)

  def generate(self):
    header = self._table_spec.generate_header()
    rows = self._generate_rows()
    return Table.TABLE_HTML.format(header=header, rows=rows)

class TableSpec(object):
  ROW_HTML = '<tr>{cells}</tr>'
  def __init__(self, keys):
    self._columns = []
    for key in keys:
      self._columns.append(column_factory(key))

  def generate_header(self):
    headers = []
    for col in self._columns:
      headers.append(col.generate_header())
    return TableSpec.ROW_HTML.format(cells=''.join(headers))

  def generate_row(self, data_row):
    cells = []
    for col in self._columns:
      cells.append(col.generate_cell(data_row))
    return TableSpec.ROW_HTML.format(cells=''.join(cells))

class DistributionSpec(object):
  def __init__(self, keys):
    self.keys = keys

class SurveySpec(object):
  def __init__(self, keys):
    self.keys = keys

class ObjectCountSpec(object):
  def __init__(self, keys):
    self.keys = keys
      
def column_factory(key):
  if type(key) == str:
    _, feature_type = features.FEATURES[key]
    if feature_type == 'id':
      return IdColumn(key)
    elif feature_type == 'duration':
      return DurationColumn(key)
    elif feature_type == 'count':
      return CountColumn(key)
    elif feature_type == 'timeline':
      return TimelineColumn(key)
    elif feature_type == 'timestamp':
      return TimestampColumn(key)
    elif feature_type == 'string':
      return StringColumn(key)
    elif feature_type == 'objectcount':
      return ObjectCountColumn(key)
    elif feature_type == 'yesno':
      return YesNoColumn(key)
  elif type(key) == DistributionSpec:
    return DistributionColumn(key.keys)
  elif type(key) == SurveySpec:
    return SurveyColumn(key.keys)
  elif type(key) == ObjectCountSpec:
    return ObjectCountColumn(key.keys)
  
class DataColumn(object):
  HEADER_HTML = '<th>{}</th>'
  CELL_HTML = '<td>{}</td>'
  def __init__(self, key):
    self._key = key
    self._title, _ = features.FEATURES[key]

  def generate_header(self):
    return DataColumn.HEADER_HTML.format(self._title)

  def generate_cell(self, data):
    return DataColumn.CELL_HTML.format(data[self._key])

class IdColumn(DataColumn):
  pass

class DurationColumn(DataColumn):
  def generate_cell(self, data):
    return DataColumn.CELL_HTML.format(utils.format_duration(data[self._key]))

class CountColumn(DataColumn):
  pass

class TimelineColumn(DataColumn):
  EVENT_HTML = '''
    <div class="progress-bar progress-bar-{}" style="width: {}%">
    </div>
  '''
  TIMELINE_HTML = '''
  <div class="progress">
    {events}
  </div>
  '''
  
  def generate_header(self):
    return DataColumn.HEADER_HTML.format(
      'Webcam timeline. Left=green, Right=blue, Other=yellow'
    )

  def generate_cell(self, data):
    timeline = data[self._key]
    timeline_events = []
    total_time = sum([delta.to_sec() for delta, state in timeline])
    for delta, state in timeline:
      color = None
      if state == 'left':
        color = 'success'
      elif state == 'right':
        color = 'info'
      else:
        color = 'warning'
      percentage = 100 * delta.to_sec() / total_time
      timeline_event = TimelineColumn.EVENT_HTML.format(color, percentage)
      timeline_events.append(timeline_event)
    timeline_html = TimelineColumn.TIMELINE_HTML.format(
      events=''.join(timeline_events)
    )
    return DataColumn.CELL_HTML.format(timeline_html)

class TimestampColumn(DataColumn):
  pass

class StringColumn(DataColumn):
  pass

class SurveyColumn(DataColumn):
  LIKERT_HTML = '''
    <td>
      <div class="progress">
        <div class="progress-bar progress-bar-{cls}"
          style="width: {percentage}%">
          {content}
        </div>
      </div>
    </td>
  '''
  CLASSES = ['success', 'info', 'warning', 'danger']

  def __init__(self, keys):
    self._keys = keys

  def generate_header(self):
    headers = []
    for key in self._keys:
      title, _ = features.FEATURES[key]
      header = DataColumn.HEADER_HTML.format(title)
      headers.append(header)
    return ''.join(headers)

  def generate_cell(self, data):
    cells = []
    for i, key in enumerate(self._keys):
      value = data[key]
      class_index = i % len(SurveyColumn.CLASSES)
      cell = SurveyColumn.LIKERT_HTML.format(
        cls=SurveyColumn.CLASSES[class_index],
        percentage=100 * value / 4,
        content=value
      )
      cells.append(cell)
    return ''.join(cells)

class ObjectCountColumn(DataColumn):
  COUNT_HTML = '''
    <td>
      <div class="progress">
        <div class="progress-bar progress-bar-{cls}"
          style="width: {percentage}%">
          {content}
        </div>
      </div>
    </td>
  '''
  CLASSES = ['success', 'info']

  def __init__(self, keys):
    self._keys = keys

  def generate_header(self):
    headers = []
    for key in self._keys:
      title, _ = features.FEATURES[key]
      header = DataColumn.HEADER_HTML.format(title)
      headers.append(header)
    return ''.join(headers)

  def generate_cell(self, data):
    cells = []
    for i, key in enumerate(self._keys):
      value = data[key]
      class_index = i % len(ObjectCountColumn.CLASSES)
      cell = ObjectCountColumn.COUNT_HTML.format(
        cls=ObjectCountColumn.CLASSES[class_index],
        percentage=100 * value / 6,
        content=value
      )
      cells.append(cell)
    return ''.join(cells)

class YesNoColumn(DataColumn):
  pass

class DistributionColumn(DataColumn):
  DISTRIBUTION_HTML = '''
    <td colspan={num_parts}>
      <div class="progress">
        {distribution}
      </div>
    </td>
  '''
  PART_HTML = '''
    <div class="progress-bar progress-bar-{cls}" style="width: {percentage}%">
      {content}
    </div>
  '''
  CLASSES = ['success', 'info', 'danger']

  def __init__(self, keys):
    self._keys = keys

  def generate_header(self):
    headers = []
    for key in self._keys:
      title, _ = features.FEATURES[key]
      header = DataColumn.HEADER_HTML.format(title)
      headers.append(header)
    return ''.join(headers)

  def generate_cell(self, data):
    data = [data[key] for key in self._keys]

    total = genpy.Duration(0)
    for duration in data:
      total += duration

    parts = []
    for i, duration in enumerate(data):
      class_index = i % len(DistributionColumn.CLASSES)
      part_class = DistributionColumn.CLASSES[class_index]
      percentage = 100 * duration.to_sec() / total.to_sec()
      content = utils.format_duration(duration)
      parts.append(
        DistributionColumn.PART_HTML.format( cls=part_class,
          percentage=percentage,
          content=content
        )
      )
    num_parts = len(parts)
    distribution_html = ''.join(parts)
    return DistributionColumn.DISTRIBUTION_HTML.format(
      num_parts=num_parts,
      distribution=distribution_html
    )

DATA_SPEC = TableSpec(
  [
    'user_id',
    'time_taken',
    DistributionSpec(
      ['camera_movement_time', 'marker_movement_time', 'other_time']
    ),
    'grasp_count',
    DistributionSpec(['left_time', 'right_time']),
    DistributionSpec(['mean_left', 'mean_right']),
    DistributionSpec(['left_stddev', 'right_stddev']),
    'num_left_looks',
    'num_right_looks'
  ]
)

TIMELINE_SPEC = TableSpec(['user_id', 'timeline'])

PERSONAL_SPEC = TableSpec(
  [
    'user_id',
    ObjectCountSpec(
      [
        'num_collected',
        'num_possible'
      ]
    ),
    SurveySpec(
      [
        'prior_gaming',
        'prior_modeling',
        'prior_remote_control',
        'prior_robotics'
      ]
    ),
    'prior_study',
    'other_comments'
  ]
)

TROUBLE_SPEC = TableSpec(
  [
    'user_id',
    SurveySpec(
      [
        'trouble_moving_gripper',
        'trouble_slow_interface',
        'trouble_arm_blocking',
        'trouble_unrecognizable',
        'trouble_locating'
      ]
    ),
    'strategy',
    'most_trouble'
  ]
)

PCL_SPEC = TableSpec(
  [
    SurveySpec(
      [
        'pcl_could_see',
        'pcl_could_recognize',
        'pcl_could_reach',
        'pcl_could_judge_dist',
        'pcl_used_more',
        'pcl_could_go_without'
      ]
    ),
    'pcl_advantages'
  ]
)

CAM_SPEC = TableSpec(
  [
    SurveySpec(
      [
        'cam_could_see',
        'cam_could_recognize',
        'cam_could_reach',
        'cam_could_judge_dist',
        'cam_used_more',
        'cam_could_go_without'
      ]
    ),
    'cam_advantages'
  ]
)

def generate(data):
  title = 'Robot teleoperation interface data'
  data_table = Table(DATA_SPEC, data)
  data_section = Section('Experiment data', data_table)
  timeline_table = Table(TIMELINE_SPEC, data)
  timeline_section = Section('Webcam timeline', timeline_table)
  personal_table = Table(PERSONAL_SPEC, data)
  personal_section = Section('User info', personal_table)
  trouble_table = Table(TROUBLE_SPEC, data)
  trouble_section = Section('Troubles and strategy', trouble_table)
  pcl_table = Table(PCL_SPEC, data)
  pcl_section = Section('Point cloud view', pcl_table)
  cam_table = Table(CAM_SPEC, data)
  cam_section = Section('Camera view', cam_table)
  page = Page(
    title, data_section, personal_section, trouble_section, pcl_section,
    cam_section, timeline_section
  )
  return page.generate()
