import matplotlib.pyplot as plt
# from mpl_toolkits.axes_grid1 import ImageGrid
# import numpy as np
from os import listdir
from os import chdir
from os import path
from PIL import Image
# import matplotlib.gridspec as gridspec
import argparse

parser = argparse.ArgumentParser(description="generate plot for report")
parser.add_argument("--input_dir", required=True, help="Input ROS bag.")
parser.add_argument("--rows", required=True, help="numer of rows in figure")
parser.add_argument("--cols", required=True, help="number of columns in figure")

args = parser.parse_args()

# chdir('/Volumes/macOS Big Sur/Users/pmvanderburg/matplotlib_test/')
chdir(args.input_dir)
files = listdir(args.input_dir)
files.sort()


for i, f in enumerate(files):
    if f!='.DS_Store':
        print(i,f)
    else:
        del files[i]

images = [Image.open(f) for f in files]
print(len(images))
max_rows = 7
max_cols = 3
# max_rows = 3
# max_cols = 2

methods=['Input image',
         '640x480 N+FT',
         '832x256 K+FT',
         '640x480 N',
         '832x256 N',
         '640x480 K',
         '832x256 K']

fig, axes = plt.subplots(nrows=7, ncols=3, figsize=(9,10),sharex=True, sharey=True)
for idx, image in enumerate(images):
    # print(files[idx])
    print(idx)
    row = idx % max_rows
    col = idx // max_rows
    print(row,' row')
    print(col,' col')
    # if col>0:
    # axes[row, col].axis("off")
    axes[row,col].spines['bottom'].set_color('#ffffff')
    axes[row,col].spines['top'].set_color('#ffffff')
    axes[row,col].spines['right'].set_color('#ffffff')
    axes[row,col].spines['left'].set_color('#ffffff')

    if image.size==(1280, 720):
        image = image.resize((640,480))

    axes[row, col].imshow(image, cmap="gray", aspect="auto")

    axes[row, 0].set_ylabel(methods[row])

plt.subplots_adjust(wspace=.05, hspace=.05)
plt.xticks([])
plt.yticks([])
# fig.savefig(path.join)
plt.show()
