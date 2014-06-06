"""Processes rosbag data from the CSE 599K1 observational study.
"""
from __future__ import division
from __future__ import print_function

import features
import message_factory
from object_stats import ObjectStats
import processors
import survey
import topics
import views

from collections import namedtuple
import multiprocessing
import os
import rosbag
import sys

EXPERIMENT_TOPICS = [
  topics.CAMERA_POSE,
  topics.LEFT_GRASP,
  topics.LEFT_POSITION,
  topics.RIGHT_GRASP,
  topics.RIGHT_POSITION,
  topics.MARKER_FEEDBACK
]

UserData = namedtuple('UserData',
  ['user_id', 'experiment_file', 'webcam_file', 'object_file']
)

def collate_files(filenames):
  """Collate data files by user ID."""
  experiment_data = {}
  webcam_data = {}
  object_data = {}
  for filename in filenames:
    name_parts = filename.split('_')
    user_id = name_parts[0]
    end = name_parts[-1]
    if end == 'experiment.bag':
      experiment_data[user_id] = filename
    elif end == 'code.bag':
      webcam_data[user_id] = filename
    elif end == 'grasps.tsv':
      object_data[user_id] = filename

  user_data = []
  for user_id, experiment_file in experiment_data.items():
    if user_id in webcam_data and user_id in object_data:
      webcam_file = webcam_data[user_id]
      object_file = object_data[user_id]
      user_data.append(
        UserData(user_id, experiment_file, webcam_file, object_file)
      )
  return user_data

def open_bag(filename):
  try:
    return rosbag.Bag(filename)
  except Exception as e:
    print(e)

def process_objects(path):
  print('Processing', path)
  with open(path) as timeline:
    object_processor = processors.Objects()
    for line in timeline:
      line = line.strip()
      parts = line.split('\t')
      start_time = float(parts[0])
      duration = float(parts[1])
      side = parts[2]
      obj = parts[3]
      object_processor.update(start_time, duration, side, obj)
    object_processor.update_last()
  return {
    'object_timeline': object_processor.timeline()
  }

def process_experiment((path, object_timeline)):
  """Extract features of the bag file."""
  print('Processing', path)
  bag = open_bag(path)
  if bag is None:
    return None
  object_stats = ObjectStats()
  time_taken_processor = processors.TimeTaken(object_timeline)
  camera_movement_processor = processors.CameraMovementTime(object_timeline)
  marker_movement_processor = processors.MarkerMovementTime(object_timeline)
  grasp_count_processor = processors.GraspCount(object_timeline)
  for topic, message, time in bag.read_messages(topics=EXPERIMENT_TOPICS):
    model = message_factory.model(message)
    if model is None:
      continue
    time_taken_processor.update(topic, model, time)
    camera_movement_processor.update(topic, model, time)
    marker_movement_processor.update(topic, model, time)
    grasp_count_processor.update(topic, model, time)
  bag.close()
  time_taken_processor.update_last()
  object_stats.update(time_taken_processor.object_stats())
  object_stats.update(camera_movement_processor.object_stats())
  object_stats.update(marker_movement_processor.object_stats())
  object_stats.update(grasp_count_processor.object_stats())
  for obj, stats in object_stats.items():
    if not 'time_taken' in stats:
      print(obj, stats)
    stats['other_time'] = (
      stats['time_taken'] - stats['camera_movement_time'] -
      stats['marker_movement_time']
    )
  
  return object_stats

def process_code((path, object_timeline)):
  """Extract features from a webcam coding."""
  print('Processing', path)
  bag = open_bag(path)
  if bag is None:
    return None
  code_processor = processors.Code(object_timeline)
  for topic, message, time in bag.read_messages():
    model = message_factory.model(message)
    if model is None:
      continue
    code_processor.update(topic, model, time)
  bag.close()
  return code_processor.object_stats(), code_processor.timeline()
  #return {
  #  'left_time': code_processor.left_time(),
  #  'right_time': code_processor.right_time(),
  #  'mean_left': code_processor.mean_left(),
  #  'mean_right': code_processor.mean_right(),
  #  'left_stddev': code_processor.left_stddev(),
  #  'right_stddev': code_processor.right_stddev(),
  #  'num_left_looks': code_processor.num_left_looks(),
  #  'num_right_looks': code_processor.num_right_looks(),
  #  'timeline': code_processor.timeline()
  #}

def process(data_dir, user_data, survey_data):
  pool = multiprocessing.Pool(processes=12)
  user_ids = [user.user_id for user in user_data]

  object_paths = [
    '/'.join([data_dir, user.object_file]) for user in user_data
  ]
  object_features = pool.map(process_objects, object_paths)
  object_timelines = [x['object_timeline'] for x in object_features]
  experiment_paths = [
    '/'.join([data_dir, user.experiment_file]) for user in user_data
  ]
  args = zip(experiment_paths, object_timelines)
  experiment_features = pool.map(process_experiment, args)
  webcam_paths = [
    '/'.join([data_dir, user.webcam_file]) for user in user_data
  ]
  args = zip(webcam_paths, object_timelines)
  webcam_features = pool.map(process_code, args)

  all_data = []
  for user_id, exp, (cam, cam_timeline), obj, survey in zip(
    user_ids, experiment_features, webcam_features, object_features,
    survey_data):
    object_stats = ObjectStats()
    object_stats.update(exp)
    object_stats.update(cam)
    data = (user_id, object_stats, cam_timeline, obj, survey)
    #data = {'user_id': user_id}
    #data.update(exp)
    #data.update(cam)
    #data.update(obj)
    #data.update(survey)
    all_data.append(data)

  html = views.generate(all_data)
  output_path = '/'.join([data_dir, 'index.html'])
  with open(output_path, 'w') as output_file:
    print(html, file=output_file)

def main():
  data_dir = sys.argv[1]
  user_data = collate_files(os.listdir(data_dir))
  user_data = sorted(user_data, key=lambda x: x.user_id)
  survey_data = survey.read(os.sep.join([data_dir, 'survey.tsv']))
  survey_data = sorted(survey_data)
  survey_data = [data for user_id, data in survey_data]
  process(data_dir, user_data, survey_data)

if __name__ == '__main__':
  main()
