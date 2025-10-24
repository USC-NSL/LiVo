# CONFIG
START_FRAME=19
END_FRAME=5600
FPS=30
D2C_SPLIT=0.875     # 7/8

SEQ_NAME_SHORT=office1
SEQ_NAME=170915_${SEQ_NAME_SHORT}_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_170915_${SEQ_NAME_SHORT}.json
LOG_ID=1

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
# tail -f client_office1.log | grep -Ei "FPS:|Frame:|Failed.*QR|QR.*Failed|lost"

