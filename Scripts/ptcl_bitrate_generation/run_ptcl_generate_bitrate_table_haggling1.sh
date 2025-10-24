CONFIG_PATH=/home/lei/rajrup/KinectStream/Multiview/config

# render_image_path="/datassd/pipeline_ptcl/client_ptcl/$SEQ_NAME/o3d_ptcl_draco_test/"
# save_ptcl_path="/datassd/pipeline_ptcl/client_ptcl/$SEQ_NAME/o3d_ptcl_draco_test/"
render_image_path=""
save_ptcl_path=""

####### 160422_haggling1 ########
START_FRAME=150
END_FRAME=13000
SEQ_NAME=160422_haggling1_with_ground
CONFIG_FILE=$CONFIG_PATH/panoptic_160422_haggling1.json

taskset --cpu-list 6-8 ../../build/Multiview/PanopticPtclDracoBitrateTable \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --view_ptcl=0 \
        --server_cull=3 \
        --client_cull=3 \
        --render_image_path=$render_image_path \
        --save_ptcl_path=$save_ptcl_path