# CONFIG
START_FRAME=150
END_FRAME=12751
FPS=30
COLOR_BITRATE=20000
DEPTH_BITRATE=120000

SEQ_NAME=160422_haggling1_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_160422_haggling1.json

taskset --cpu-list 0-7 ../build/Multiview/MultiviewServerPool \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --ground=true \
        --send_ptcl=0 \
        --use_mm=true \
        --use_server_bitrate=true \
        --use_client_bitrate=false \
        --cb=$COLOR_BITRATE \
        --db=$DEPTH_BITRATE \
        --server_fps=$FPS \
        --save_frame=false \
        --server_cull=5 &

sleep 3
taskset --cpu-list 8-11 python3.8 ../WebRTC/sender_new.py \
        -seq_name $SEQ_NAME \
        -sid $START_FRAME \
        -eid $END_FRAME \
        -v 0 \
        -f $FPS \
        -abr 1 \
        -mm 1 \
        -cb $COLOR_BITRATE \
        -db $DEPTH_BITRATE > ./Experiments/pipeline_log/sender_tile.out &
