#!/bin/bash

for i in {10..15..1}
do
  cp -v "/home/pmvanderburg/noetic-husky/datasets/bop_datasets/husky/train/$(printf "%06i" $i)/delay_100/scene_gt.json" "/home/pmvanderburg/noetic-husky/datasets/bop_datasets/husky/train/$(printf "%06i" $i)/"
done

