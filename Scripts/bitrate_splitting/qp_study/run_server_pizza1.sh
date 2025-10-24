# CONFIG
START_FRAME=1
END_FRAME=1413
FPS=30
COLOR_QP=50
DEPTH_QP=0
LOG_ID=1

SEQ_NAME=160906_pizza1_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_160906_pizza1.json

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
        > log/server_pizza1.log 2>&1