"""Processes rosbag data from the CSE 599K1 observational study.
"""
from __future__ import division
from __future__ import print_function

import features
import message_factory
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

UserData = namedtuple('UserData', ['user_id', 'experiment_file', 'webcam_file'])
ExperimentData = namedtuple('ExperimentData',
  [name for name, description in features.FEATURES]
)

def collate_files(filenames):
  """Collate data files by user ID."""
  experiment_data = {}
  webcam_data = {}
  for filename in filenames:
    name_parts = filename.split('_')
    user_id = name_parts[0]
    end = name_parts[-1]
    if end == 'experiment.bag':
      experiment_data[user_id] = filename
    elif end == 'code.bag':
      webcam_data[user_id] = filename

  user_data = []
  for user_id, experiment_file in experiment_data.items():
    if user_id in webcam_data:
      webcam_file = webcam_data[user_id]
      user_data.append(UserData(user_id, experiment_file, webcam_file))
  return user_data

def open_bag(filename):
  try:
    return rosbag.Bag(filename)
  except Exception as e:
    print(e)

def process_experiment(path):
  """Extract features of the bag file."""
  print('Processing', path)
  bag = open_bag(path)
  if bag is None:
    return None
  time_taken_processor = processors.TimeTaken()
  camera_movement_time_processor = processors.CameraMovementTime()
  marker_movement_time_processor = processors.MarkerMovementTime()
  grasp_count_processor = processors.GraspCount()
  message_logs = {}
  for topic, message, time in bag.read_messages(topics=EXPERIMENT_TOPICS):
    model = message_factory.model(message)
    if model is None:
      continue
    time_taken_processor.update(topic, model, time)
    camera_movement_time_processor.update(topic, model, time)
    marker_movement_time_processor.update(topic, model, time)
    grasp_count_processor.update(topic, model, time)
    if topic in message_logs:
      message_logs[topic].append((model, time))
    else:
      message_logs[topic] = [(model, time)]
  bag.close()
  time_taken = time_taken_processor.time_taken()
  camera_movement_time = camera_movement_time_processor.movement_time()
  marker_time = marker_movement_time_processor.movement_time()
  num_grasps = grasp_count_processor.num_grasps()
  return (
    time_taken,
    camera_movement_time,
    marker_time,
    time_taken - camera_movement_time - marker_time,
    num_grasps
  )

def process_code(path):
  """Extract features from a webcam coding."""
  print('Processing', path)
  bag = open_bag(path)
  if bag is None:
    return None
  code_processor = processors.Code()
  for topic, message, time in bag.read_messages():
    model = message_factory.model(message)
    if model is None:
      continue
    code_processor.update(topic, model, time) 
  bag.close()
  return (
    code_processor.left_time(),
    code_processor.right_time(),
    code_processor.mean_left(),
    code_processor.mean_right(),
    code_processor.left_stddev(),
    code_processor.right_stddev(),
    code_processor.num_left_looks(),
    code_processor.num_right_looks(),
    code_processor.timeline()
  )

def process(data_dir, user_data, survey_data):
  all_data = []
  user_ids = [(user.user_id,) for user in user_data]
  experiment_paths = [
    '/'.join([data_dir, user.experiment_file]) for user in user_data
  ]
  pool = multiprocessing.Pool(processes=7)
  experiment_features = pool.map(process_experiment, experiment_paths)
  webcam_paths = [
    '/'.join([data_dir, user.webcam_file]) for user in user_data
  ]
  webcam_features = pool.map(process_code, webcam_paths)
  webcam_data = [features[:-1] for features in webcam_features]
  timelines = [features[-1] for features in webcam_features]

  all_data = sorted([
    (ExperimentData(*(a + b + c)), d, e)
    for a, b, c, d, e in zip(
      user_ids, experiment_features, webcam_data, timelines, survey_data
    )
  ])

  html = views.generate(all_data)
  output_path = '/'.join([data_dir, 'index.html'])
  with open(output_path, 'w') as output_file:
    print(html, file=output_file)

def main():
  data_dir = sys.argv[1]
  user_data = collate_files(sorted(os.listdir(data_dir)))
  survey_data = survey.read(os.sep.join([data_dir, 'survey.tsv']))
  process(data_dir, user_data, survey_data)

if __name__ == '__main__':
  main()
