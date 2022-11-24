#!/bin/bash

CONVERSION_DIR="/home/pmvanderburg/noetic-husky/datasets/bop_datasets/vis_gt_poses/husky/train/"
obj_id=5
framerate=10

#pushd . > /dev/null 2>&1

cd $CONVERSION_DIR

for i in {130..136}
do
  exp_nr=$(printf "%06i" $((i+100000*$obj_id))) #exp_nr)($i+100000*$obj_id)
  abspath="${CONVERSION_DIR}${exp_nr}"
  echo 'checking existence of' $abspath
  if [[ -d $abspath ]]; then
    echo "folder of sequence $exp_nr exists."
    cd ./$exp_nr
    ffmpeg -framerate $framerate -pattern_type glob -i '*.jpg' -c:v libx264 -r $framerate ../$(printf "%i" $framerate)_fps_rgb_vis_gt_$(printf "%06i" $exp_nr).mp4
    cd ../
  fi
done

#popd > /dev/null 2>&1
