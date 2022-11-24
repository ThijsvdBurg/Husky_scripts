#!/bin/bash

PYTHON="/usr/bin/python3"
SCRIPT_ROOT="/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/pybop/scripts"
SCRIPT="roslaunch_gt_json.py"

#pushd . > /dev/null 2>&1

#source $HOME/.bashrc

cd $SCRIPT_ROOT

for i in {0..136} #30}
do
  export sync_delay=0
  export exp_nr=$i
  export split_type='train_tmp'
  $PYTHON "$SCRIPT"
done

#popd > /dev/null 2>&1
