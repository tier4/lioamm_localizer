#!/usr/bin/env python3
import os
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math
import matplotlib
import matplotlib.cm as cm
import matplotlib.patches as patches
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable, get_cmap
import argparse

plt.style.use('seaborn')
plt.rcParams["font.size"] = 12


def get_result(score_list):
  result = []
  for i in range(0, 5000):
    count = 0
    for score in score_list:
      if score < i:
        count += 1
    result.append(count / len(score_list))
  return result

def main(input):
  filesize = sum(os.path.isfile(os.path.join(input,name)) for name in os.listdir(input) if '.csv' in name)

  true_score = []
  offset_score0 = []
  offset_score1 = []
  offset_score2 = []
  offset_score3 = []
  offset_score4 = []
  offset_score5 = []
  offset_score6 = []
  offset_score7 = []
  for file_num in range(filesize-1):
    df = pd.read_csv(input+"/"+str(file_num)+".csv")
    true_score.append(df['score'][0])
    offset_score0.append(df['offset score'][0]) # -0.1, -0.1
    offset_score1.append(df['offset score'][1]) # -0.1, 0
    offset_score2.append(df['offset score'][2]) # -0.1, 0.1
    offset_score3.append(df['offset score'][3]) #  0.0, -0.1
    offset_score4.append(df['offset score'][4]) #  0.0, 0.1
    offset_score5.append(df['offset score'][5]) #  0.1, -0.1
    offset_score6.append(df['offset score'][6]) # -0.1, 0.0
    offset_score7.append(df['offset score'][7]) #  0.1, 0.1

  plt.figure(figsize=(10, 10))
  score_threshold = [i for i in range(0, 5000)]

  true_result = get_result(true_score)
  plt.plot(score_threshold, true_result, label='true score')

  offset_score_result0 = get_result(offset_score0)
  plt.plot(score_threshold, offset_score_result0, label='offset score (x, y)=(-0.1, -0.1)')

  offset_score_result1 = get_result(offset_score1)
  plt.plot(score_threshold, offset_score_result1, label='offset score (x, y)=(-0.1, 0.0)')

  offset_score_result2 = get_result(offset_score2)
  plt.plot(score_threshold, offset_score_result2, label='offset score (x, y)=(-0.1, 0.1)')

  offset_score_result3 = get_result(offset_score3)
  plt.plot(score_threshold, offset_score_result3, label='offset score (x, y)=(0.0, -0.1)')

  offset_score_result4 = get_result(offset_score4)
  plt.plot(score_threshold, offset_score_result4, label='offset score (x, y)=(0.0, 0.1)')

  offset_score_result5 = get_result(offset_score5)
  plt.plot(score_threshold, offset_score_result5, label='offset score (x, y)=(0.1, -0.1)')

  offset_score_result6 = get_result(offset_score6)
  plt.plot(score_threshold, offset_score_result6, label='offset score (x, y)=(-0.1, 0.0)')

  offset_score_result7 = get_result(offset_score7)
  plt.plot(score_threshold, offset_score_result7, label='offset score (x, y)=(0.1, 0.1)')

  plt.xlabel('Score Threshold')
  plt.ylabel('Ratio [0~1.0]')
  plt.title('Score Ratio')
  plt.legend()
  plt.show()

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--input')

	args = parser.parse_args()
	main(args.input)
