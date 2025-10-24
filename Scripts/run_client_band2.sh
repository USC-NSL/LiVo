# CONFIG
START_FRAME=2
END_FRAME=5517
FPS=30
COLOR_BITRATE=40000
DEPTH_BITRATE=40000
BITRATE=$(($COLOR_BITRATE+$DEPTH_BITRATE))
# D2C_SPLIT=0.85
D2C_SPLIT=0.875     # 7/8

SEQ_NAME=160906_band2_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_160906_band2.json
LOG_ID=1

COLOR_BITRATE_M=$(($COLOR_BITRATE/1000))
DEPTH_BITRATE_M=$(($DEPTH_BITRATE/1000))
COLOR_QP=0
DEPTH_QP=0

# MM_TRACE_NAME=mh-120-240-new
MM_TRACE_NAME=tracep1-scaled10.0
# MM_TRACE_NAME=wifi-25-scaled15.0

# output_dir="/datassd/pipeline_cpp/client_tiled/ablation/bitrate_study/$SEQ_NAME/"
# save_ptcl_path="/datassd/pipeline_cpp/client_tiled/ablation/bitrate_study/$SEQ_NAME/o3d_nvenc_c${COLOR_BITRATE_M}m_d_yuv${DEPTH_BITRATE_M}m_fps${FPS}_s_nocull_c_nocull/"
# render_image_path="/datassd/pipeline_cpp/client_tiled/ablation/bitrate_study/$SEQ_NAME/o3d_nvenc_c${COLOR_BITRATE_M}m_d_yuv${DEPTH_BITRATE_M}m_fps${FPS}_s_nocull_c_nocull/"
# save_ptcl_path="o3d_nvenc_cqp${COLOR_QP}_d_yuvqp${DEPTH_QP}_fps${FPS}_s_nocull_c_nocull/"
# render_image_path="o3d_nvenc_cqp${COLOR_QP}_d_yuvqp${DEPTH_QP}_fps${FPS}_s_nocull_c_nocull/"
# save_cbitrate_file="cbitrate_cqp${COLOR_QP}_d_yuvqp${DEPTH_QP}_fps${FPS}_s_nocull_c_nocull.csv"
# save_dbitrate_file="dbitrate_cqp${COLOR_QP}_d_yuvqp${DEPTH_QP}_fps${FPS}_s_nocull_c_nocull.csv"

# output_dir="/datassd/pipeline_cpp/client_tiled/ablation/bitrate_static_split_$D2C_SPLIT/$SEQ_NAME/"
# output_dir="/datassd/pipeline_cpp/client_tiled/ablation/bitrate_dynamic_split/$SEQ_NAME/"
# save_ptcl_path="o3d_nvenc_${BITRATE}k_fps${FPS}_s_nocull_c_nocull/"
# save_ptcl_path="o3d_nvenc_${BITRATE}k_fps${FPS}_s_cull_c_cull/"

output_dir="/home/lei/data/pipeline/client_tiled/pipeline_new/test/"
save_ptcl_path=""
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

# taskset --cpu-list 0-19 ../build/Multiview/MultiviewClientPoolNew \
#         --seq_name=$SEQ_NAME \
#         --config_file=$CONFIG_FILE \
#         --start_frame_id=$START_FRAME \
#         --end_frame_id=$END_FRAME \
#         --client_fps=$FPS \
#         --cb=$COLOR_BITRATE \
#         --db=$DEPTH_BITRATE \
#         --mm_trace_name=$MM_TRACE_NAME \
#         --use_split_adapt=true \
#         --d2c_split=$D2C_SPLIT \
#         --send_ptcl=0 \
#         --save_frame=false \
#         --view_ptcl=3 \
#         --client_cull=3 \
#         --output_dir=$output_dir \
#         --render_image_path=$render_image_path \
#         --save_ptcl_path=$save_ptcl_path \
#         --save_cbitrate_file=$save_cbitrate_file \
#         --save_dbitrate_file=$save_dbitrate_file \
#         > client_band2.log 2>&1

taskset --cpu-list 0-19 ../build/Multiview/MultiviewClientPoolNew \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --use_split_adapt=true \
        --d2c_split=$D2C_SPLIT \
        --send_ptcl=0 \
        --save_frame=false \
        --view_ptcl=3 \
        --client_cull=3 \
        --output_dir=$output_dir \
        --render_image_path=$render_image_path \
        --save_ptcl_path=$save_ptcl_path \
        --save_cbitrate_file=$save_cbitrate_file \
        --save_dbitrate_file=$save_dbitrate_file \
        > client_band2.log 2>&1

# For tailing the log file with important information
# tail -f client_band2.log | grep -Ei "FPS:|Frame:|Failed.*QR|QR.*Failed|lost"

