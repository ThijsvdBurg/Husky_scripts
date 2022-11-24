#!/bin/bash

for j in {10..50..1}
do

  rm -rv $BOP_PATH/husky/train/$(printf %06i $j)/rgb*
  rm -rv $BOP_PATH/husky/train/$(printf %06i $j)/scene_*.json

done
