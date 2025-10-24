#!/bin/bash

# This script is used to run the standalone version of the the pipeline without communication. It generates ground truth images for evaluation, rendered in RGB and/or Ptcl formats.

CONFIG_PATH=/home/lei/rajrup/KinectStream/Multiview/config

####### 160317_moonbaby1 ######## -> TESTING
# START_FRAME=59
# NFRAMES=6100
# SEQ_NAME=160317_moonbaby1_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_170307_dance5.json

####### 160224_ultimatum2 ########
# START_FRAME=60
# NFRAMES=13800        
# SEQ_NAME=160224_ultimatum2_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160224_ultimatum2.json

####### 160226_mafia1 ########
# START_FRAME=1
# NFRAMES=5000
# SEQ_NAME=160226_mafia1_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160226_mafia1.json

####### 160422_haggling1 ########
# START_FRAME=150
# NFRAMES=13000
# SEQ_NAME=160422_haggling1_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160422_haggling1.json

####### 160906_band2 ########
# START_FRAME=2
# END_FRAME=5517
# SEQ_NAME=160906_band2_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160906_band2.json

####### 160906_ian3 ########
# START_FRAME=780
# NFRAMES=9000
# SEQ_NAME=160906_ian3_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160906_ian3.json

####### 160906_pizza1 ########
# START_FRAME=1
# END_FRAME=1413
# SEQ_NAME=160906_pizza1_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160906_pizza1.json

####### 170307_dance5 ########
# START_FRAME=34
# END_FRAME=10000
# SEQ_NAME=170307_dance5_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_170307_dance5.json

####### 170915_office1 ########
# START_FRAME=19
# END_FRAME=5600
# SEQ_NAME=170915_office1_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_170915_office1.json

####### 170915_toddler4 ########
START_FRAME=1
END_FRAME=3800
SEQ_NAME=170915_toddler4_with_ground
CONFIG_FILE=$CONFIG_PATH/panoptic_170915_toddler4.json

####### EXPERIMENT #############
# render_image_path="/datassd/pipeline/client_tiled/pipeline_new/$SEQ_NAME/o3d_gt_no_ground_s_nocull_c_nocull/"
# save_ptcl_path="/datassd/pipeline/client_tiled/pipeline_new/$SEQ_NAME/o3d_gt_with_ground_s_nocull_c_nocull/"
render_image_path="/datassd/pipeline_ptcl/client_ptcl/$SEQ_NAME/o3d_gt_with_ground/"
# save_ptcl_path="/datassd/pipeline_ptcl/client_ptcl/$SEQ_NAME/o3d_gt_with_ground_s_nocull_c_nocull/"
# render_image_path=""
# save_ptcl_path=""

../build/Multiview/PanopticGT \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --view_ptcl=3 \
        --server_cull=3 \
        --client_cull=3 \
        --ground=true \
        --render_image_path=$render_image_path \
        --save_ptcl_path=$save_ptcl_path