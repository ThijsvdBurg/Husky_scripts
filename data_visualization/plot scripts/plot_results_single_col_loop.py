import matplotlib.pyplot as plt
# from mpl_toolkits.axes_grid1 import ImageGrid
# import numpy as np
from os import listdir
from os import chdir
from os import path
from PIL import Image
from os import access
from os import mkdir
from os import W_OK

from PIL import Image
# import matplotlib.gridspec as gridspec
import argparse

parser = argparse.ArgumentParser(description="generate plot for report")
parser.add_argument("--input_dir", required=True, help="Input ROS bag.")
# parser.add_argument("--rows", required=True, help="numer of rows in figure")
# parser.add_argument("--cols", required=True, help="number of columns in figure")

args = parser.parse_args()
indir=args.input_dir
# chdir('/Volumes/macOS Big Sur/Users/pmvanderburg/matplotlib_test/')
# chdir(args.input_dir)
folders=listdir(indir)

for i, f in enumerate(folders):
    if f=='.DS_Store' or f=='plots':
        del folders[i]
folders.sort()
for fol in folders:
    # print(fol)

    chdir(path.join(args.input_dir,fol))
    files = listdir(path.join(args.input_dir,fol))
    for i, fil in enumerate(files):
        if fil=='.DS_Store':
            del files[i]
    files.sort()
    # print(files)
        #
    # for i, f in enumerate(files):
    #     if f!='.DS_Store':
    #         print(i,f)
    #     else:
    #         del files[i]
    #

    images = [Image.open(fil) for fil in files]
    print('images length',len(images))
    #
    # max_rows = args.rows
    # max_cols = args.cols
    max_rows = 4
    max_cols = 1

    methods=['Input image',
             '640x480 N+FT',
             '832x256 K+FT',
             '640x480 N',
             '832x256 N',
             '640x480 K',
             '832x256 K']

    fig, axes = plt.subplots(nrows=max_rows, ncols=max_cols, figsize=(6,20) ,sharex=True, sharey=True)
    # fig, axes = plt.subplots(nrows=max_rows, ncols=max_cols, gridspec_kw = {'height_ratios':[1,1,1,1,1, 1]})
    for idx, image in enumerate(images):
        # print(files[idx])
        # print(idx)
        row = idx // max_cols
        col = idx % max_cols
        # if col>0:
        if image.size==(1280, 720):
            image = image.resize((640,480))

        axes[row].spines['bottom'].set_color('#ffffff')
        axes[row].spines['top'].set_color('#ffffff')
        axes[row].spines['right'].set_color('#ffffff')
        axes[row].spines['left'].set_color('#ffffff')
        axes[row].imshow(image, cmap="gray", aspect="auto")

        axes[row].set_ylabel(methods[row])


    plt.subplots_adjust(wspace=.05, hspace=.05)
    plt.xticks([])
    plt.yticks([])
    # plt.setp(axes[:, 0], ylabel='y axis label')
    filename="single_column_plot_{}.png".format(fol)
    plotpath=path.join(indir,'plots')
    if access(plotpath, W_OK)!=True:
        mkdir(plotpath)

    savepath=path.join(plotpath,filename)
    print(savepath)
    fig.savefig(savepath)
    # plt.show()

    #
    # savepath=path.join(indir,'plots',filename)
    # # print(savepath)
    # fig.savefig(savepath)
    # # plt.show()
