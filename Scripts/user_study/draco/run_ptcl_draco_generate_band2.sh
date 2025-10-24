MM_TRACE_NAME=tracep1-scaled10.0
# MM_TRACE_NAME=wifi-25-scaled15.0

SERVER_CULLING=3
sculling="pcull"
METHOD="livo"

# CONFIG
LOG_ID=1
START_FRAME=1000
END_FRAME=5517

FPS=30
D2C_SPLIT=0.875     # 7/8

SEQ_ID=160906
SEQ_NAME_SHORT=band2
SEQ_NAME=${SEQ_ID}_${SEQ_NAME_SHORT}_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_${SEQ_ID}_${SEQ_NAME_SHORT}.json

CLIENT_CULLING=3
cculling="cull"

taskset --cpu-list 0-3 ../../../build/Multiview/PanopticPtclDracoOutputGeneration \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --view_ptcl=3 \
        --server_cull=$SERVER_CULLING \
        --client_cull=$CLIENT_CULLING \
        --mm_trace_name=$MM_TRACE_NAME