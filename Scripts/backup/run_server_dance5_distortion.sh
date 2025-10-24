# CONFIG
START_FRAME=34
END_FRAME=2000
FPS=30
COLOR_BITRATE=30000
DEPTH_BITRATE=130000

SEQ_NAME=170307_dance5_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_170307_dance5.json

## Mahimahi trace reset to start
rm -rf mm_time
up_file=mahi_traces/mh-240
down_file=mahi_traces/mh-240
date +%s%N >> mm_time
echo $up_file >> mm_time
echo $down_file >> mm_time

taskset --cpu-list 0-7 ../build/Multiview/MultiviewServerPool \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --ground=true \
        --send_ptcl=0 \
        --use_mm=false \
        --use_server_bitrate=false \
        --use_client_bitrate=false \
        --cb=$COLOR_BITRATE \
        --db=$DEPTH_BITRATE \
        --server_fps=$FPS \
        --save_frame=false \
        --server_cull=0 &

sleep 3
taskset --cpu-list 8-11 python3.8 ../WebRTC/sender_new.py \
        -seq_name $SEQ_NAME \
        -sid $START_FRAME \
        -eid $END_FRAME \
        -v 0 \
        -f $FPS \
        -abr 0 \
        -mm 0 \
        -cb $COLOR_BITRATE \
        -db $DEPTH_BITRATE > ./Experiments/pipeline_log/sender_tile.out &
