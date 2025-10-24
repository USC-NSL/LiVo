# CONFIG
START_FRAME=34
END_FRAME=10000
FPS=30
COLOR_BITRATE=60000
DEPTH_BITRATE=60000
BITRATE=$(($COLOR_BITRATE+$DEPTH_BITRATE))
# D2C_SPLIT=0.95
D2C_SPLIT=0.875     # 7/8

SEQ_NAME=170307_dance5_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_170307_dance5.json
LOG_ID=0

# output_dir="/datassd/pipeline_cpp/client_tiled/ablation/bitrate_static_split_$D2C_SPLIT/$SEQ_NAME/"
output_dir="/datassd/pipeline_cpp/client_tiled/ablation/bitrate_dynamic_split/$SEQ_NAME/"
save_ptcl_path="o3d_nvenc_${BITRATE}k_fps${FPS}_s_nocull_c_nocull/"
# save_ptcl_path="o3d_nvenc_${BITRATE}k_fps${FPS}_s_cull_c_cull/"
# output_dir="/home/lei/data/pipeline/client_tiled/pipeline_new/test/"
# save_ptcl_path=""

render_image_path=$save_ptcl_path
save_cbitrate_file=""
save_dbitrate_file=""

# Remove old render images if exists
if [ -d "$render_image_path" ]; then
    echo "Removing old render images"
    rm -rf $render_image_path
fi

# Remove old render images if exists
if [ -d "$save_ptcl_path" ]; then
    echo "Removing old render images"
    rm -rf $save_ptcl_path
fi

taskset --cpu-list 0-19 ../../../build/Multiview/MultiviewClientPoolNew \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --client_fps=$FPS \
        --cb=$COLOR_BITRATE \
        --db=$DEPTH_BITRATE \
        --use_client_bitrate=true \
        --use_split_adapt=true \
        --d2c_split=$D2C_SPLIT \
        --send_ptcl=0 \
        --save_frame=false \
        --view_ptcl=3 \
        --client_cull=0 \
        --output_dir=$output_dir \
        --render_image_path=$render_image_path \
        --save_ptcl_path=$save_ptcl_path \
        --save_cbitrate_file=$save_cbitrate_file \
        --save_dbitrate_file=$save_dbitrate_file \
        > log/client_dance5.log 2>&1

# For tailing the log file with important information
# tail -f log/client_dance5.log | grep -Ei "FPS:|RMSE|Failed.*QR|QR.*Failed|lost"

