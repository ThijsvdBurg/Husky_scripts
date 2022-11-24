#!/bin/bash


for j in {0..136..1}
do

  rm -rv $BOP_PATH/husky/train/$(printf %06i $j)/rgb_right

done
