# CONFIG
START_FRAME=2
END_FRAME=5517
FPS=30
COLOR_BITRATE=40000
DEPTH_BITRATE=40000
COLOR_QP=0
DEPTH_QP=0

SEQ_NAME=160906_band2_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_160906_band2.json

# MM_TRACE_NAME=mh-120-240-new
MM_TRACE_NAME=tracep1-scaled10.0
# MM_TRACE_NAME=wifi-25-scaled15.0

# output_dir="/datassd/pipeline_cpp/server_tiled/ablation/bitrate_static_split_$D2C_SPLIT/$SEQ_NAME/"
# output_dir="/datassd/pipeline_cpp/server_tiled/ablation/bitrate_dynamic_split/$SEQ_NAME/"
output_dir="/home/lei/data/pipeline/server_tiled/pipeline_new/test/"

# Run Pipeline without mahimahi, abr, and save frames
taskset --cpu-list 0-11 ../build/Multiview/MultiviewServerPoolNew \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --ground=true \
        --send_ptcl=0 \
        --use_mm=true \
        --use_server_bitrate_est=true \
        --use_client_bitrate_split=true \
        --cb=$COLOR_BITRATE \
        --db=$DEPTH_BITRATE \
        --mm_trace_name=$MM_TRACE_NAME \
        --cqp=$COLOR_QP \
        --dqp=$DEPTH_QP \
        --server_fps=$FPS \
        --save_frame=false \
        --output_dir=$output_dir \
        --server_cull=3 > server_band2.log 2>&1

# For tailing the log file with important information
# tail -f server_band2.log | grep -Ei "bwe|bitrate"
