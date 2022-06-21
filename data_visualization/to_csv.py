import numpy as np
import bagpy
from bagpy import bagreader
import pandas as pd

def main():
	max_nr = 19
	for i in range(1, max_nr+1):
		b = bagreader('exp_data/exp'+str(i)+'.bag')

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