# CONFIG
START_FRAME=2
END_FRAME=5517
FPS=30
COLOR_BITRATE=20000
DEPTH_BITRATE=60000
LOG_ID=1

SEQ_NAME=160906_band2_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_160906_band2.json

# Run Pipeline without mahimahi, abr, and save frames
taskset --cpu-list 0-11 ../../../build/Multiview/MultiviewServerPoolNew \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --ground=true \
        --send_ptcl=0 \
        --use_mm=false \
        --use_server_bitrate=false \
        --use_client_bitrate=true \
        --cb=$COLOR_BITRATE \
        --db=$DEPTH_BITRATE \
        --server_fps=$FPS \
        --save_frame=false \
        --server_cull=0 > log/server_band2.log 2>&1

# For tailing the log file with important information
# tail -f log/server_band2.log | grep -Ei "bwe|bitrate"
