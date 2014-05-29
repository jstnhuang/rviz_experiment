"""Processes the user study survey."""
import features

from collections import namedtuple

SurveyData = namedtuple('SurveyData',
  [name for name, description, is_likert in features.SURVEY_FEATURES]
)

def convert_likert(value):
  if value == 'Strongly disagree':
    return 0
  elif value == 'Disagree':
    return 1
  elif value == 'Neither agree nor disagree':
    return 2
  elif value == 'Agree':
    return 3
  else:
    return 4

def transform(entry):
  """Change Likert scale items to be 0-4, handle other special cases."""
  entry_dict = entry._asdict()
  # Normalize "other" for the prior study question.
  if entry_dict['prior_study'] != 'No':
    entry_dict['prior_study'] = 'Yes'

  entry_dict['num_collected'] = int(entry.num_collected)
  entry_dict['num_possible'] = int(entry.num_possible)

  for name, description, is_likert in features.SURVEY_FEATURES:
    if is_likert:
      entry_dict[name] = convert_likert(entry_dict[name])
  return SurveyData(**entry_dict)

def read(path):
  survey_data = []
  with open(path) as survey:
    next(survey)
    for line in survey:
      line_parts = [part.strip() for part in line.split('\t')]
      entry = transform(SurveyData(*line_parts))
      survey_data.append(entry)
  return survey_data
