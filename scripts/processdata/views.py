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
        {body}
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

TABLE_HTML = '''
<table class="table">
  {header}
  {{rows}}
</table>'''.format(header=TABLE_HEADER)

def generate(all_data):
  title = 'Robot teleoperation interface data'
  rows = []
  for data in all_data:
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
  table = TABLE_HTML.format(rows=''.join(rows))
  data_area = DATA_AREA.format(title=title, table=table)
  html = BASE_HTML.format(title=title, body=data_area)
  return html
