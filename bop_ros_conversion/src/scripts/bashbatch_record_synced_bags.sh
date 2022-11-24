#!/bin/bash

PYTHON="/usr/bin/python3"
SCRIPT_ROOT="/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/pybop/scripts"
SCRIPT="roslaunch_syncrecord_consecutive_v2.py"

#pushd . > /dev/null 2>&1

cd $SCRIPT_ROOT

for i in {130..140}
do
  echo "========================================================"
  echo "This is the $i th run"
  echo "========================================================"
  export sync_delay=0
  export exp_nr=$i
  export sync_slop=0.02
  $PYTHON "$SCRIPT"
done

#popd > /dev/null 2>&1
