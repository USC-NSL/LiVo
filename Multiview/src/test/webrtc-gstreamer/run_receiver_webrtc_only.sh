#!/bin/bash

# CONFIG
NFRAMES=4000
FPS=7
COLOR_BITRATE=100000
DEPTH_BITRATE=200000
SEQ_NAME=160317_moonbaby1_with_ground
START_FRAME=59

taskset --cpu-list 0-3 ./build/receiver_webrtc_only --seq_name $SEQ_NAME --start_frame_id $START_FRAME --ncaptures $NFRAMES --color_bitrate $COLOR_BITRATE --depth_bitrate $DEPTH_BITRATE --fps $FPS 