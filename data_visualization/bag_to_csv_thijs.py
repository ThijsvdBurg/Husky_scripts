#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""convert rosbag to csv to analyze the raw data"""

# scripts expects the bagfiles to have the following name: 'sequencei.bag with i being the experiment number

import os
import argparse
import numpy as np
import bagpy
from bagpy import bagreader
import pandas as pd

def main():
    """Extract info from bagfile and append to csv datafile
    """
    parser = argparse.ArgumentParser(description="Extract bagfile to csv")
    parser.add_argument("--source_dir", help="Directory containing rosbags. Absolute paths are preferred")
    parser.add_argument("--output_dir", help="Output directory.")
    parser.add_argument("--start", type=int, help="Start index number: 001 to 099 are just denoted as 1 to 99")
    parser.add_argument("--end", type=int, help="End index number; same syntax as start index number")
    parser.add_argument("--date", help="Filename date prefix.")
    parser.add_argument("--suffix", help="Filename suffix, either edit, crop, synced or omit argument for unedited rosbag")

    args = parser.parse_args()

    print("Extract data from %s into %s" %(args.source_dir,args.output_dir))
    max_num = 999
    width=len(str(max_num))

    for i in range(args.start, args.end+1):

        # select the proper bagname depending on whether a suffix is supplied in the arguments
        if args.suffix==None:
            bagname="%s_sequence_{i:0{width}}.bag".format(i=i,width=width) %(args.date)
        else:
            bagname="%s_sequence_{i:0{width}}_%s.bag".format(i=i,width=width) %(args.date, args.suffix)
        file = os.path.join(args.source_dir,bagname)
        print(file)
        b = bagreader(file)

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

