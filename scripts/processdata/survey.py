"""Processes the user study survey."""
import features

SURVEY_MAP = [
  'timestamp',
  'user_id',
  'strategy',
  'pcl_could_see',
  'pcl_could_recognize',
  'pcl_could_reach',
  'pcl_could_judge_dist',
  'pcl_used_more',
  'pcl_could_go_without',
  'cam_could_see',
  'cam_could_recognize',
  'cam_could_reach',
  'cam_could_judge_dist',
  'cam_used_more',
  'cam_could_go_without',
  'pcl_advantages',
  'cam_advantages',
  'trouble_moving_gripper',
  'trouble_slow_interface',
  'trouble_arm_blocking',
  'trouble_unrecognizable',
  'trouble_locating',
  'most_trouble',
  'prior_gaming',
  'prior_modeling',
  'prior_remote_control',
  'num_collected',
  'num_possible',
  'prior_robotics',
  'other_comments',
  'prior_study',
  'email',
  'notes'
]

def convert_likert(value):
  if value == 'Strongly disagree':
    return 0
  elif value == 'Disagree':
    return 1
  elif value == 'Neither agree nor disagree':
    return 2
  elif value == 'Agree':
    return 3
  elif value == 'Strongly agree':
    return 4

def transform(line_parts):
  row_dict = {}
  user_id = None
  for part, key in zip(line_parts, SURVEY_MAP):
    _, key_type = features.FEATURES[key]
    if key_type == 'yesno' and part != 'No':
      part = 'Yes'
    if key_type == 'count' or key_type == 'objectcount':
      part = int(part)
    if key_type == 'likert':
      part = convert_likert(part)
    if key != 'user_id':
      row_dict[key] = part
    else:
      user_id = part
  return (user_id, row_dict)

def read(path):
  survey_data = []
  with open(path) as survey:
    next(survey)
    for line in survey:
      line_parts = [part.strip() for part in line.split('\t')]
      entry = transform(line_parts)
      survey_data.append(entry)
  return survey_data
