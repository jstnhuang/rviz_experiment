import features

BASE_HTML = '''
  <!doctype html>
  <html>
    <head>
      <title>{title}</title>
    </head>
    <body>{body}</body>
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

TABLE_HTML = '<table>{header}{{rows}}</table>'.format(header=TABLE_HEADER)

def generate(all_data):
  title = 'Robot teleoperation interface data'
  rows = []
  for experiment_data in all_data:
    experiment_dict = experiment_data._asdict()
    values = ''.join([
      '<td>{value}</td>'.format(value=experiment_dict[name])
      for name, description in features.FEATURES
    ])
    row = '<tr>{values}</tr>'.format(values=values)
    rows.append(row)
  table = TABLE_HTML.format(rows=''.join(rows))
  data_area = DATA_AREA.format(title=title, table=table)
  html = BASE_HTML.format(title=title, body=data_area)
  return html
