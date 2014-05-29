from __future__ import division

import features
import utils

class Page:
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

class Section:
  """A section consists of a section title and a data table."""
  SECTION_HTML = '''
  <h2>{title}</h2>
  {table}
  '''
  def __init__(self, title, table):
    self._title = title
    self._table = table

  def generate(self):
    return Section.SECTION_HTML.format(title=self._title, table=self._table)

class Table:
  TABLE_HTML = '''
    <table class="table">
      {header}
      {rows}
      {average}
    </table>
  '''

  def __init__(self):
    pass

  def generate(self):
    pass

TABLE_HEADER = '<tr>{cols}</tr>'.format(
  cols=''.join([
    '<th>{col}</th>'.format(col=description)
    for name, description in features.FEATURES
  ])
)

TIMELINE_HEADER = '''
<tr>
  <th>User ID</th><th>Webcam timeline. Left=green, Right=blue, Other=yellow</th>
</tr>
'''


PROGRESS_CLASSES = ['success', 'info', 'warning', 'danger']

PERSONAL_FEATURES = [features.SURVEY_FEATURES[x] for x in [
  26, 27, 23, 24, 25, 28, 30, 29
]]

TROUBLE_FEATURES = [features.SURVEY_FEATURES[x] for x in [
  17, 18, 19, 20, 21, 2, 22
]]

PCL_FEATURES = [features.SURVEY_FEATURES[x] for x in [
  3, 4, 5, 6, 7, 8, 15
]]

CAM_FEATURES = [features.SURVEY_FEATURES[x] for x in [
  9, 10, 11, 12, 13, 14, 16
]]

def generate_data_table(all_data):
  rows = []
  for data, timeline, survey in all_data:
    left_right_time = data.left_time + data.right_time
    mean_total = data.mean_left + data.mean_right
    stddev_total = data.left_stddev + data.right_stddev
    looks_total = (
      data.num_left_looks + data.num_right_looks
    )

    values = ''.join([
      '<td>{}</td>'.format(data.user_id),
      '<td>{}</td>'.format(utils.format_duration(data.time_taken)),
      '''<td colspan=3>
        <div class="progress">
          <div class="progress-bar progress-bar-success" style="width: {}%">
            {}
          </div>
          <div class="progress-bar progress-bar-info" style="width: {}%">
            {}
          </div>
          <div class="progress-bar progress-bar-danger" style="width: {}%">
            {}
          </div>
        </div>
      </td>'''.format(
        100 * data.camera_movement_time.to_sec() / data.time_taken.to_sec(),
        utils.format_duration(data.camera_movement_time),
        100 * data.marker_movement_time.to_sec() / data.time_taken.to_sec(),
        utils.format_duration(data.marker_movement_time),
        100 * data.other_time.to_sec() / data.time_taken.to_sec(),
        utils.format_duration(data.other_time)
      ),
      '<td>{}</td>'.format(data.grasp_count),
      '''<td colspan=2>
        <div class="progress">
          <div class="progress-bar progress-bar-success" style="width: {}%">
            {}
          </div>
          <div class="progress-bar progress-bar-info" style="width: {}%">
            {}
          </div>
        </div>
      </td>'''.format(
        100 * data.left_time.to_sec() / left_right_time.to_sec(),
        utils.format_duration(data.left_time),
        100 * data.right_time.to_sec() / left_right_time.to_sec(),
        utils.format_duration(data.right_time)
      ),
      '''<td colspan=2>
        <div class="progress">
          <div class="progress-bar progress-bar-success" style="width: {}%">
            {}
          </div>
          <div class="progress-bar progress-bar-info" style="width: {}%">
            {}
          </div>
        </div>
      </td>'''.format(
        100 * data.mean_left.to_sec() / mean_total.to_sec(),
        utils.format_duration(data.mean_left),
        100 * data.mean_right.to_sec() / mean_total.to_sec(),
        utils.format_duration(data.mean_right)
      ),
      '''<td colspan=2>
        <div class="progress">
          <div class="progress-bar progress-bar-success" style="width: {}%">
            {}
          </div>
          <div class="progress-bar progress-bar-info" style="width: {}%">
            {}
          </div>
        </div>
      </td>'''.format(
        100 * data.left_stddev.to_sec() / stddev_total.to_sec(),
        utils.format_duration(data.left_stddev),
        100 * data.right_stddev.to_sec() / stddev_total.to_sec(),
        utils.format_duration(data.right_stddev)
      ),
      '<td>{}</td>'.format(data.num_left_looks),
      '<td>{}</td>'.format(data.num_right_looks)
    ])
    row = '<tr>{values}</tr>'.format(values=values)
    rows.append(row)
  table = TABLE_HTML.format(header=TABLE_HEADER, rows=''.join(rows))
  return table

def generate_timeline_table(all_data):
  rows = []
  for data, timeline, survey in all_data:
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
      timeline_event = '''
        <div class="progress-bar progress-bar-{}" style="width: {}%">
        </div>
      '''.format(color, percentage)
      timeline_events.append(timeline_event)
    timeline_html = '''
    <div class="progress">
      {events}
    </div>
    '''.format(events=''.join(timeline_events))
    row = '<tr><td>{user_id}</td><td>{timeline_html}</td></tr>'.format(
      user_id=data.user_id,
      timeline_html=timeline_html
    )
    rows.append(row)
  table = TABLE_HTML.format(header=TIMELINE_HEADER, rows=''.join(rows))
  return table

def generate_survey_table(all_data, section_features):
  header = '<tr><th>User ID</th>{cols}</tr>'.format(
    cols=''.join([
      '<th>{col}</th>'.format(col=description)
      for name, description, is_likert in section_features
    ])
  )
  rows = []
  for data, timeline, survey in all_data:
    survey_dict = survey._asdict()
    values = []
    class_index = 0 # Cycle through CSS classes for progress bars.
    for name, description, is_likert in section_features:
      value = survey_dict[name]
      if is_likert:
        color = PROGRESS_CLASSES[class_index]
        class_index = (class_index + 1) % len(PROGRESS_CLASSES)
        value = '''
          <div class="progress">
            <div class="progress-bar progress-bar-{}" style="width: {}%">
              {}
            </div>
          </div>
        '''.format(color, 100 * survey_dict[name] / 4, survey_dict[name])
      elif type(value) == type(0): # Number of objects collected/possible.
        color = PROGRESS_CLASSES[class_index]
        class_index = (class_index + 1) % len(PROGRESS_CLASSES)
        value = '''
          <div class="progress">
            <div class="progress-bar progress-bar-{}" style="width: {}%">
              {}
            </div>
          </div>
        '''.format(color, 100 * survey_dict[name] / 6, survey_dict[name])
      values.append('<td>{}</td>'.format(value))
    row = '<tr><td>{}</td>{}</tr>'.format(data.user_id, ''.join(values))
    rows.append(row)
  table = TABLE_HTML.format(header=header, rows=''.join(rows))
  return table

def generate(all_data):
  title = 'Robot teleoperation interface data'
  data_table = generate_data_table(all_data)
  data_section = Section('Experiment data', data_table)
  timeline_table = generate_timeline_table(all_data)
  timeline_section = Section('Webcam timeline', timeline_table)
  personal_table = generate_survey_table(all_data, PERSONAL_FEATURES)
  personal_section = Section('User info', personal_table)
  trouble_table = generate_survey_table(all_data, TROUBLE_FEATURES)
  trouble_section = Section('Troubles and strategy', trouble_table)
  pcl_table = generate_survey_table(all_data, PCL_FEATURES)
  pcl_section = Section('Point cloud view', pcl_table)
  cam_table = generate_survey_table(all_data, CAM_FEATURES)
  cam_section = Section('Camera view', cam_table)
  page = Page(
    title, data_section, personal_section, trouble_section, pcl_section, cam_section,
    timeline_section
  )
  return page.generate()
