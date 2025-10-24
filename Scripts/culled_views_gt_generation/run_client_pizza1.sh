# CONFIG
START_FRAME=1
END_FRAME=1029
FPS=30
D2C_SPLIT=0.875     # 7/8

SEQ_NAME_SHORT=pizza1
SEQ_NAME=160906_${SEQ_NAME_SHORT}_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_160906_${SEQ_NAME_SHORT}.json
LOG_ID=2

output_dir="/home/lei/data/pipeline/client_tiled/pipeline_new/test/"

CLIENT_CULLING=3

taskset --cpu-list 0-19 ../../build/Multiview/MultiviewClientPoolNew \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --use_split_adapt=false \
        --d2c_split=$D2C_SPLIT \
        --send_ptcl=0 \
        --view_ptcl=3 \
        --client_cull=$CLIENT_CULLING \
        --output_dir=$output_dir \
        --render_image_path=$render_image_path \
        --save_ptcl_path=$save_ptcl_path \
        --save_cbitrate_file=$save_cbitrate_file \
        --save_dbitrate_file=$save_dbitrate_file \
        > ./log/client_${SEQ_NAME_SHORT}_logID_${LOG_ID}.log 2>&1

# For tailing the log file with important information
# tail -f client_pizza1.log | grep -Ei "FPS:|Frame:|Failed.*QR|QR.*Failed|lost"

