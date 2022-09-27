import numpy as np
import bagpy
from bagpy import bagreader
import pandas as pd

def main():
  bagpath = '/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/bagfiles/2022-09-27-07-23-59_rw3.bag'
  b = bagreader(bagpath)

  # get the list of topics
  print(b.topic_table)

  # load data to csv files
  csvfiles = []
  for t in b.topics:
    data = b.message_by_topic(t)
    csvfiles.append(data)

  print(csvfiles[0])
  data = pd.read_csv(csvfiles[0])


if __name__ == "__main__":
  main()
