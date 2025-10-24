MM_TRACE_NAME=tracep1-scaled10.0
# MM_TRACE_NAME=wifi-25-scaled15.0

SERVER_CULLING=5
sculling="kpcull10"
METHOD="livo"

# CONFIG
LOG_ID=0
START_FRAME=34
END_FRAME=10000

FPS=30

SEQ_ID=170307
SEQ_NAME_SHORT=dance5
SEQ_NAME=${SEQ_ID}_${SEQ_NAME_SHORT}_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_${SEQ_ID}_${SEQ_NAME_SHORT}.json

output_dir="/home/lei/data/pipeline/server_tiled/pipeline_new/test/"

# Run Pipeline without mahimahi, abr, and save frames
taskset --cpu-list 0-11 ../../../build/Multiview/MultiviewServerPoolNew \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --ground=true \
        --use_mm=true \
        --use_server_bitrate_est=true \
        --use_client_bitrate_split=true \
        --mm_trace_name=$MM_TRACE_NAME \
        --server_fps=$FPS \
        --output_dir=$output_dir \
        --server_cull=$SERVER_CULLING \
        > ./log/server_${SEQ_NAME_SHORT}_s_${sculling}_logID_${LOG_ID}_trace_${MM_TRACE_NAME}.log 2>&1

# For tailing the log file with important information
# tail -f server_band2.log | grep -Ei "fps|bwe|bitrate"
