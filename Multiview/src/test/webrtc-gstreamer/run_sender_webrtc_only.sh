#!/bin/bash

NFRAMES=4000
FPS=7
COLOR_BITRATE=100000
DEPTH_BITRATE=200000
START_FRAME=59
SEQ_NAME=160317_moonbaby1_with_ground

# Run the sender
taskset --cpu-list 0-3 ./build/sender_webrtc_only --seq_name $SEQ_NAME --start_frame_id $START_FRAME --ncaptures $NFRAMES --color_bitrate $COLOR_BITRATE --depth_bitrate $DEPTH_BITRATE --fps $FPS 