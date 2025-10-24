# CONFIG
START_FRAME=2
END_FRAME=5970
FPS=30
COLOR_BITRATE=100000
DEPTH_BITRATE=100000

SEQ_NAME_SHORT=band2
SEQ_NAME=160906_${SEQ_NAME_SHORT}_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_160906_${SEQ_NAME_SHORT}.json
LOG_ID=2

output_dir="/home/lei/data/pipeline/server_tiled/pipeline_new/test/"

SERVER_CULLING=3

views_subfolder="test"
if [ $SERVER_CULLING -eq 3 ]; then
    views_subfolder=culled_views_logID_${LOG_ID}
elif [ $SERVER_CULLING -eq 5 ]; then
    views_subfolder=kpculled_views_logID_${LOG_ID}
fi

# Run Pipeline without mahimahi, abr, and save frames
taskset --cpu-list 0-11 ../../build/Multiview/MultiviewServerPoolNew \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --ground=true \
        --use_mm=false \
        --save_views=true \
        --save_views_subfolder=$views_subfolder \
        --use_server_bitrate_est=false \
        --use_client_bitrate_split=false \
        --cb=$COLOR_BITRATE \
        --db=$DEPTH_BITRATE \
        --server_fps=$FPS \
        --output_dir=$output_dir \
        --server_cull=$SERVER_CULLING \
        > ./log/server_${SEQ_NAME_SHORT}_logID_${LOG_ID}.log 2>&1

# For tailing the log file with important information
# tail -f server_band2.log | grep -Ei "fps|bwe|bitrate"
