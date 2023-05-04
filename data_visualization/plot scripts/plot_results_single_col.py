import matplotlib.pyplot as plt
# from mpl_toolkits.axes_grid1 import ImageGrid
# import numpy as np
from os import listdir
from os import chdir
from PIL import Image
# import matplotlib.gridspec as gridspec
import argparse

parser = argparse.ArgumentParser(description="generate plot for report")
parser.add_argument("--input_dir", required=True, help="Input ROS bag.")
# parser.add_argument("--rows", required=True, help="numer of rows in figure")
# parser.add_argument("--cols", required=True, help="number of columns in figure")

args = parser.parse_args()

# chdir('/Volumes/macOS Big Sur/Users/pmvanderburg/matplotlib_test/')
chdir(args.input_dir)
# folders=listdir(args.input_dir)
# for f in folders:
    # print(f)

files = listdir(args.input_dir)
files.sort()

for i, f in enumerate(files):
    if f!='.DS_Store':
        print(i,f)
    else:
        del files[i]

images = [Image.open(f) for f in files]

# max_rows = args.rows
# max_cols = args.cols
max_rows = 7
max_cols = 1

methods=['Input image',
         '640x480 N+FT',
         '832x256 K+FT',
         '640x480 N',
         '832x256 N',
         '640x480 K',
         '832x256 K']

fig, axes = plt.subplots(nrows=max_rows, ncols=max_cols, figsize=(3,10) ,sharex=True, sharey=True)
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
fig.savefig('myplot.png')
plt.show()
