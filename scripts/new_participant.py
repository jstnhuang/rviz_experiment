#!/usr/bin/python
from __future__ import print_function

import random

def participant_id():
  return random.randint(0, 999)

def main():
  print('Participant ID: {}'.format(participant_id()))

if __name__ == '__main__':
  main()
