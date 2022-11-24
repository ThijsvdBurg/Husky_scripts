#!/bin/bash

PYTHON="/usr/bin/python3"
SCRIPT_ROOT="/home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/pybop/scripts"
SCRIPT="roslaunch_odom_consecutive_v2.py"

#pushd . > /dev/null 2>&1

cd $SCRIPT_ROOT

#for i in {4,6,68,69,70,77,78,97,105,108,109}
for i in {72,77}
#for i in {105,108,109}
#for i in {120..136}
#for i in {30..40}
do
  echo "========================================================"
  echo "This is the $i th run"
  echo "========================================================"
  export sync_delay=0
  export exp_nr=$i
  export dummynr=31
  export sleepbf=1.25
  export bagdelay=2.1
  $PYTHON "$SCRIPT"
  export bagdelay=2.2
  $PYTHON "$SCRIPT"
  export bagdelay=2.3
  $PYTHON "$SCRIPT"
  export bagdelay=2.4
  $PYTHON "$SCRIPT"
  export bagdelay=2.5
  $PYTHON "$SCRIPT"
  #export sleepbf=1.2
  #export bagdelay=2.2
  #$PYTHON "$SCRIPT"
  #export sleepbf=1.25
  #export bagdelay=2.2
  #$PYTHON "$SCRIPT"
  #export sleepbf=1.3
  #export bagdelay=2.2
  #$PYTHON "$SCRIPT"
  #export sleepbf=1.35
  #export bagdelay=2.2
  #$PYTHON "$SCRIPT"
  #export sleepbf=1.4
  #export bagdelay=2.2
  #$PYTHON "$SCRIPT"
  #export sleepbf=1.45
  #export bagdelay=2.2
  #$PYTHON "$SCRIPT"
  #export sleepbf=1.5
  #export bagdelay=2.2
  #$PYTHON "$SCRIPT"
  #export sleepbf=1.55
  #export bagdelay=2.2
  #$PYTHON "$SCRIPT"
  #export sleepbf=1.3
  #export bagdelay=2.2
  #$PYTHON "$SCRIPT"
  #export sleepbf=1.3
  #export bagdelay=2.2
  #$PYTHON "$SCRIPT"

done

#popd > /dev/null 2>&1
