MM_TRACE_NAME=tracep1-scaled10.0
# MM_TRACE_NAME=wifi-25-scaled15.0

SERVER_CULLING=5
sculling="kpcull10"
METHOD="livo"

# CONFIG
LOG_ID=1
START_FRAME=1
END_FRAME=1413

FPS=30
D2C_SPLIT=0.875     # 7/8

SEQ_ID=160906
SEQ_NAME_SHORT=pizza1
SEQ_NAME=${SEQ_ID}_${SEQ_NAME_SHORT}_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_${SEQ_ID}_${SEQ_NAME_SHORT}.json

CLIENT_CULLING=3
cculling="cull"

output_dir="/home/lei/data/pipeline/client_tiled/pipeline_new/test/"

taskset --cpu-list 0-19 ../../../build/Multiview/MultiviewClientPoolNew \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --use_split_adapt=true \
        --d2c_split=$D2C_SPLIT \
        --view_ptcl=3 \
        --client_cull=$CLIENT_CULLING \
        --output_dir=$output_dir \
        > ./log/client_${SEQ_NAME_SHORT}_s_${sculling}_c_${cculling}_logID_${LOG_ID}_trace_${MM_TRACE_NAME}.log 2>&1

# For tailing the log file with important information
# tail -f client_band2.log | grep -Ei "FPS:|Frame:|Failed.*QR|QR.*Failed|lost"

