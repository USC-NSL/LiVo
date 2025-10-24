# CONFIG
START_FRAME=1
END_FRAME=1413
FPS=30
COLOR_BITRATE=20000
# DEPTH_BITRATE=120000
DEPTH_BITRATE=60000

# pipeline_method=starline++          # starline++ = livo_nocull
# culling=o3d_nvenc_s_nocull_c_cull

pipeline_method=livo
culling=o3d_nvenc_s_kpcull10_c_cull

# pipeline_method=starline
# culling=o3d_nvenc_s_nocull_c_cull

mm_type=tracep1
# mm_type=wifi-25

SEQ_NAME=160906_pizza1_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_160906_pizza1.json

COLOR_BITRATE_M=$(($COLOR_BITRATE/1000))
DEPTH_BITRATE_M=$(($DEPTH_BITRATE/1000))
# render_image_path="/datassd/pipeline/client_tiled/pipeline_new/$SEQ_NAME/o3d_nvenc_c${COLOR_BITRATE_M}m_d_yuv${DEPTH_BITRATE_M}m_fps${FPS}_w_kpcull/"
# render_image_path="/datassd/pipeline/client_tiled/pipeline_new/$SEQ_NAME/o3d_nvenc_c${COLOR_BITRATE_M}m_d_yuv${DEPTH_BITRATE_M}m_fps${FPS}_w_kpcull_l10/"
# save_ptcl_path="/datassd/pipeline/client_tiled/pipeline_new/$SEQ_NAME/o3d_nvenc_c${COLOR_BITRATE_M}m_d_yuv${DEPTH_BITRATE_M}m_fps${FPS}_s_nocull_c_nocull/"
# render_image_path="/datassd/pipeline/client_tiled/pipeline_new/$SEQ_NAME/o3d_nvenc_c${COLOR_BITRATE_M}m_d_yuv${DEPTH_BITRATE_M}m_fps${FPS}_s_nocull_c_nocull/"
# render_image_path="/datassd/pipeline/client_tiled/$pipeline_method/$SEQ_NAME/$culling/$mm_type/"
# save_ptcl_path="/datassd/pipeline/client_tiled/$pipeline_method/$SEQ_NAME/$culling/$mm_type/"
save_ptcl_path=""
render_image_path=""

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

taskset --cpu-list 4-19 ../build/Multiview/MultiviewClientPool \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --send_ptcl=0 \
        --save_frame=false \
        --view_ptcl=3 \
        --client_cull=3 \
        --render_image_path=$render_image_path \
        --save_ptcl_path=$save_ptcl_path &

sleep 1
taskset --cpu-list 0-3 python3.8 ../WebRTC/receiver_new.py \
        -seq_name $SEQ_NAME \
        -sid $START_FRAME \
        -eid $END_FRAME \
        -v 0 \
        -f $FPS \
        -cb $COLOR_BITRATE \
        -db $DEPTH_BITRATE > ./Experiments/pipeline_log/receiver_tile.out &
