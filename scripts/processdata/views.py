from __future__ import division

import features
import utils

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
    </head>
    <body>
      <div class="container">
        {data_area}
        {timeline_area}
      </div>
    </body>
  </html>
'''

DATA_AREA = '''
<h2>{title}</h2>
{table}
'''

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

TABLE_HTML = '''
<table class="table">
  {header}
  {rows}
</table>'''

def generate_data_table(all_data):
  rows = []
  for data, timeline in all_data:
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
  for data, timeline in all_data:
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

def generate(all_data):
  title = 'Robot teleoperation interface data'
  data_table = generate_data_table(all_data)
  data_area = DATA_AREA.format(title=title, table=data_table)
  timeline_table = generate_timeline_table(all_data)
  html = BASE_HTML.format(
    title=title,
    data_area=data_area,
    timeline_area=timeline_table
  )
  return html
