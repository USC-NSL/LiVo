# CONFIG
START_FRAME=1
END_FRAME=3800
FPS=30
COLOR_QP=0
DEPTH_QP=0
LOG_ID=0

SEQ_NAME=170915_toddler4_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_170915_toddler4.json

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
        --use_client_bitrate=false \
        --cqp=$COLOR_QP \
        --dqp=$DEPTH_QP \
        --server_fps=$FPS \
        --save_frame=false \
        --server_cull=0 \
        > log/server_toddler4.log 2>&1