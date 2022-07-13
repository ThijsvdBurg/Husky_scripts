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

    parser.add_argument("--source_dir", help="Directory containing rosbags.")
    parser.add_argument("--output_dir", help="Output directory.")
    parser.add_argument("--start", type=int, help="Start index number.")
    parser.add_argument("--end", type=int, help="End index number.")
    parser.add_argument("--date", help="Filename date prefix.")
    parser.add_argument("--suffix", help="Filename suffix, edit, crop or empty for raw unedited rosbag")

    args = parser.parse_args()

    print("Extract data from %s into %s" %(args.source_dir,args.output_dir))

    for i in range(args.start, args.end+1):
        if args.suffix=="edit":
            bagname="%s_sequence_%01i_edit.bag" % (args.date,i)
        elif args.suffix=="crop":
            bagname="%s_sequence_%01i_crop.bag" % (args.date,i)
        else:
            bagname="%s_sequence_%01i.bag" % (args.date,i)

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

