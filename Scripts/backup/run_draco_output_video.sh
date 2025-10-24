CONFIG_PATH=/home/lei/rajrup/KinectStream/Multiview/config
MM_TRACE_NAME=tracep1-scaled10.0
# MM_TRACE_NAME=wifi-25-scaled15.0

####### 160906_band2 ########
# START_FRAME=1964
# END_FRAME=5517
# SEQ_NAME=160906_band2_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160906_band2.json

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

####### 160906_pizza1 ########
START_FRAME=1
END_FRAME=1413
SEQ_NAME=160906_pizza1_with_ground
CONFIG_FILE=$CONFIG_PATH/panoptic_160906_pizza1.json

####### 170915_toddler4 ########
# START_FRAME=1
# END_FRAME=3800
# SEQ_NAME=170915_toddler4_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_170915_toddler4.json

python ../Multiview/src/Panoptic/panoptic_ptcl_draco_output_video_generation.py \
        --seq_name $SEQ_NAME \
        --mm_trace_name $MM_TRACE_NAME \
        --config_file $CONFIG_FILE \
        --start_frame_id $START_FRAME \
        --end_frame_id $END_FRAME
