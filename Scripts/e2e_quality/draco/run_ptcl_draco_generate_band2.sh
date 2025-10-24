CONFIG_PATH=/home/lei/rajrup/KinectStream/Multiview/config
MM_TRACE_NAME=tracep1-scaled10.0
# MM_TRACE_NAME=wifi-25-scaled15.0

####### 160906_band2 ########
START_FRAME=2
END_FRAME=5970
SEQ_NAME=160906_band2_with_ground
CONFIG_FILE=$CONFIG_PATH/panoptic_160906_band2.json

# render_image_path="/datassd/pipeline_ptcl/client_ptcl/draco/$SEQ_NAME/"
# save_ptcl_path="/datassd/pipeline_ptcl/client_ptcl/draco/$SEQ_NAME/"
render_image_path=""
save_ptcl_path=""

taskset --cpu-list 0-3 ../../../build/Multiview/PanopticPtclDracoOutputGeneration \
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