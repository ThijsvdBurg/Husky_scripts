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
    parser.add_argument("--suffix", help="Filename suffix, edit, crop, synced or empty for raw unedited rosbag")

    args = parser.parse_args()

    print("Extract data from %s into %s" %(args.source_dir,args.output_dir))
    max_num = 999
    width=len(str(max_num))

    for i in range(args.start, args.end+1):
        if args.suffix==None:
            bagname="%s_sequence_{i:0{width}}.bag".format(i=i,width=width) %(args.date)
        else:
            bagname="%s_sequence_{i:0{width}}_%s.bag".format(i=i,width=width) %(args.date, args.suffix)
        #bagname="%s_sequence_.bag" % (args.date,i)
        #property = 'name'
        #formatter = "my {} is {{}}".format(property) # => "my name is {}"
        #print formatter.format('wong')

        file = os.path.join(args.source_dir,bagname)
        print(file)

if __name__ == "__main__":
	main()

