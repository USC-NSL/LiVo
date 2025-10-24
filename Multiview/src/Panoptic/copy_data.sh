#!/bin/bash

# Copy data from hard drive to local machine
in_path="/media/rajrup/Expansion/Christina/panoptic-toolbox"
out_path="/data/panoptic-toolbox/data"
seq="170307_dance5"

echo "Creating folder: $out_path/$seq"
mkdir -p $out_path/$seq

cp -r $in_path/$seq/*.json $out_path/$seq/

cam_arr=("50_01" "50_02" "50_03" "50_04" "50_05" "50_06" "50_07" "50_08" "50_09" "50_10")

for cam in "${cam_arr[@]}"
do
    echo "Input path: $in_path/$seq/kinectImgs/$cam"
    echo "Creating folder: $out_path/$seq/kinectImgs/$cam"
    mkdir -p $out_path/$seq/kinectImgs/$cam
    for file in $in_path/$seq/kinectImgs/$cam/*_transformed.png; do
        echo "Copying file: $file"
        cp -u $file $out_path/$seq/kinectImgs/$cam/
    done
    # cp -r $in_path/$seq/kinectImgs/$cam/*_transformed.png $out_path/$seq/kinectImgs/$cam/
done

for cam in "${cam_arr[@]}"
do
    echo "Creating folder: $out_path/$seq/kinectDepthImgs/$cam"
    mkdir -p $out_path/$seq/kinectDepthImgs/$cam
    for file in $in_path/$seq/kinectDepthImgs/$cam/*_transformed.bin; do
        echo "Copying file: $file"
        cp -u $file $out_path/$seq/kinectDepthImgs/$cam/
    done
done