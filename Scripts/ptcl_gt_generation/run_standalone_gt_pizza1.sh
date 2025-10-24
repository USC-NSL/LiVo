#!/bin/bash

# This script is used to run the standalone version of the the pipeline without communication. It generates ground truth images for evaluation, rendered in RGB and/or Ptcl formats.

CONFIG_PATH=/home/lei/rajrup/KinectStream/Multiview/config

####### 160906_pizza1 ########
START_FRAME=1
END_FRAME=1486
SEQ_NAME=160906_pizza1_with_ground
CONFIG_FILE=$CONFIG_PATH/panoptic_160906_pizza1.json

####### EXPERIMENT #############
# render_image_path="/datassd/pipeline_ptcl/client_ptcl/o3d_gt/$SEQ_NAME/o3d_gt_with_ground_c_cull/"
# save_ptcl_path="/datassd/pipeline_ptcl/client_ptcl/o3d_gt/$SEQ_NAME/o3d_gt_with_ground_c_cull/"
render_image_path="/hdd/pipeline_ptcl/client_ptcl/o3d_gt/$SEQ_NAME/o3d_gt_with_ground_c_nocull/"
save_ptcl_path="/hdd/pipeline_ptcl/client_ptcl/o3d_gt/$SEQ_NAME/o3d_gt_with_ground_c_nocull/"
# render_image_path=""
# save_ptcl_path=""

SERVER_CULL=0
CLIENT_CULL=0

taskset --cpu-list 9-11 ../../build/Multiview/PanopticGT \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --view_ptcl=3 \
        --server_cull=$SERVER_CULL \
        --client_cull=$CLIENT_CULL \
        --ground=true \
        --render_image_path=$render_image_path \
        --save_ptcl_path=$save_ptcl_path