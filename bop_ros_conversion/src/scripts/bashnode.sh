#!/bin/bash

PYTHON="/usr/bin/python3"
SCRIPT_ROOT="/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/pybop/scripts"
SCRIPT="roslaunch_odom_consecutive_v2.py"

#pushd . > /dev/null 2>&1

source $HOME/.bashrc

cd $SCRIPT_ROOT

for i in {57..75}
do
  echo "========================================================"
  echo "This is the $i th run"
  echo "========================================================"
  #source /home/timo/catkin_ws/devel/setup.bash
  export sync_delay=0
  export exp_nr=$i
  export sleepbf=1.2
  export bagdelay=1.9 #not in use right now
  $PYTHON "$SCRIPT"
done

#popd > /dev/null 2>&1
