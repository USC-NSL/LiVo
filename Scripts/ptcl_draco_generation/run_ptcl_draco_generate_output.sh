CONFIG_PATH=/home/lei/rajrup/KinectStream/Multiview/config
# MM_TRACE_NAME=tracep1-scaled10.0
MM_TRACE_NAME=wifi-25-scaled15.0

####### 160224_ultimatum2 ########
# START_FRAME=1
# END_FRAME=5108     
# SEQ_NAME=160224_ultimatum2_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160224_ultimatum2.json

####### 160226_mafia1 ########
# START_FRAME=1
# END_FRAME=1914
# SEQ_NAME=160226_mafia1_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160226_mafia1.json

####### 160422_haggling1 ########
# START_FRAME=150
# END_FRAME=12751
# SEQ_NAME=160422_haggling1_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160422_haggling1.json

####### 160906_band2 ########
# START_FRAME=2
# END_FRAME=5517
# SEQ_NAME=160906_band2_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160906_band2.json

####### 160906_ian3 ########
# START_FRAME=765
# END_FRAME=8399
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

render_image_path="/datassd/pipeline_ptcl/client_ptcl/draco/$SEQ_NAME/"
# save_ptcl_path="/datassd/pipeline_ptcl/client_ptcl/draco/$SEQ_NAME/"
# render_image_path=""
save_ptcl_path=""

taskset --cpu-list 3-5 ../build/Multiview/PanopticPtclDracoOutputGeneration \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --view_ptcl=3 \
        --server_cull=3 \
        --client_cull=3 \
        --render_image_path=$render_image_path \
        --save_ptcl_path=$save_ptcl_path \
        --mm_trace_name=$MM_TRACE_NAME