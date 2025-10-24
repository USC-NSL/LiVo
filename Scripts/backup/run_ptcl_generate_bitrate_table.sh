CONFIG_PATH=/home/lei/rajrup/KinectStream/Multiview/config

# render_image_path="/datassd/pipeline_ptcl/client_ptcl/$SEQ_NAME/o3d_ptcl_draco_test/"
# save_ptcl_path="/datassd/pipeline_ptcl/client_ptcl/$SEQ_NAME/o3d_ptcl_draco_test/"
render_image_path=""
save_ptcl_path=""

####### 160317_moonbaby1 ######## -> TESTING
START_FRAME=59
END_FRAME=6193
SEQ_NAME=160317_moonbaby1_with_ground
CONFIG_FILE=$CONFIG_PATH/panoptic_170307_dance5.json

####### 160224_ultimatum2 ######## [DONE]
# START_FRAME=1
# END_FRAME=5108      
# SEQ_NAME=160224_ultimatum2_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160224_ultimatum2.json

# taskset --cpu-list 0-2 ../build/Multiview/PanopticPtclDracoBitrateTable \
#         --seq_name=$SEQ_NAME \
#         --config_file=$CONFIG_FILE \
#         --start_frame_id=$START_FRAME \
#         --end_frame_id=$END_FRAME \
#         --view_ptcl=0 \
#         --server_cull=3 \
#         --client_cull=3 \
#         --render_image_path=$render_image_path \
#         --save_ptcl_path=$save_ptcl_path > draco_$SEQ_NAME.log 2>&1

####### 160226_mafia1 ######## [DONE]
# START_FRAME=1
# END_FRAME=1914
# SEQ_NAME=160226_mafia1_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160226_mafia1.json

# taskset --cpu-list 0-2 ../build/Multiview/PanopticPtclDracoBitrateTable \
#         --seq_name=$SEQ_NAME \
#         --config_file=$CONFIG_FILE \
#         --start_frame_id=$START_FRAME \
#         --end_frame_id=$END_FRAME \
#         --view_ptcl=0 \
#         --server_cull=3 \
#         --client_cull=3 \
#         --render_image_path=$render_image_path \
#         --save_ptcl_path=$save_ptcl_path > draco_$SEQ_NAME.log 2>&1

# ####### 160422_haggling1 ########
# START_FRAME=4870
# END_FRAME=12751
# SEQ_NAME=160422_haggling1_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160422_haggling1.json

taskset --cpu-list 3-5 ../build/Multiview/PanopticPtclDracoBitrateTable \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --view_ptcl=0 \
        --server_cull=3 \
        --client_cull=3 \
        --render_image_path=$render_image_path \
        --save_ptcl_path=$save_ptcl_path

####### 160906_band2 ######## [DONE]
# START_FRAME=2
# END_FRAME=5000
# SEQ_NAME=160906_band2_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160906_band2.json

# taskset --cpu-list 6-8 ../build/Multiview/PanopticPtclDracoBitrateTable \
#         --seq_name=$SEQ_NAME \
#         --config_file=$CONFIG_FILE \
#         --start_frame_id=$START_FRAME \
#         --end_frame_id=$END_FRAME \
#         --view_ptcl=0 \
#         --server_cull=3 \
#         --client_cull=3 \
#         --render_image_path=$render_image_path \
#         --save_ptcl_path=$save_ptcl_path > draco_$SEQ_NAME.log 2>&1

####### 160906_ian3 ########
# START_FRAME=765
# END_FRAME=8399
# SEQ_NAME=160906_ian3_with_ground
# CONFIG_FILE=$CONFIG_PATH/panoptic_160906_ian3.json

# taskset --cpu-list 9-11 ../build/Multiview/PanopticPtclDracoBitrateTable \
#         --seq_name=$SEQ_NAME \
#         --config_file=$CONFIG_FILE \
#         --start_frame_id=$START_FRAME \
#         --end_frame_id=$END_FRAME \
#         --view_ptcl=0 \
#         --server_cull=3 \
#         --client_cull=3 \
#         --render_image_path=$render_image_path \
#         --save_ptcl_path=$save_ptcl_path > draco_$SEQ_NAME.log 2>&1


###################################################################################################################

compression_type=draco
# compression_type=zstd
# render_image_path="/datassd/pipeline/client_ptcl/$SEQ_NAME/o3d_ptcl_${compression_type}_wo_cull/"
# render_image_path="/datassd/pipeline/client_ptcl/$SEQ_NAME/o3d_ptcl_${compression_type}_server_cull/"
# render_image_path="/datassd/pipeline/client_ptcl/$SEQ_NAME/o3d_ptcl_${compression_type}_client_cull/"
# render_image_path="/datassd/pipeline/client_ptcl/$SEQ_NAME/o3d_ptcl_${compression_type}_server_cull_client_cull/"

# ../build/Multiview/PanotpicPtclGenBitrateTable \
#         --seq_name=$SEQ_NAME \
#         --start_frame_id=$START_FRAME \
#         --ncaptures=$NFRAMES \
#         --view_ptcl=0 \
#         --server_cull=3 \
#         --client_cull=3 \
#         --compression_type=$compression_type > zstd.log 2>&1

# ../build/Multiview/PanotpicPtclTestCompression \
#         --seq_name=$SEQ_NAME \
#         --config_file=$CONFIG_FILE \
#         --start_frame_id=$START_FRAME \
#         --ncaptures=$NFRAMES \
#         --view_ptcl=3 \
#         --server_cull=0 \
#         --client_cull=0 \
#         --compression_type=$compression_type \
#         --render_image_path=$render_image_path \
#         --save_ptcl_path=$save_ptcl_path
#         # > draco.log 2>&1