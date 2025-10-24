# CONFIG
START_FRAME=2
END_FRAME=5517
FPS=30
COLOR_QP=0
DEPTH_QP=0
LOG_ID=1

SEQ_NAME=160906_band2_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_160906_band2.json

output_dir="/datassd/pipeline_cpp/client_tiled/ablation/bitrate_study/$SEQ_NAME/"
save_ptcl_path="o3d_nvenc_cqp${COLOR_QP}_d_yuvqp${DEPTH_QP}_fps${FPS}_s_nocull_c_nocull/"
render_image_path="o3d_nvenc_cqp${COLOR_QP}_d_yuvqp${DEPTH_QP}_fps${FPS}_s_nocull_c_nocull/"
save_cbitrate_file="cbitrate_cqp${COLOR_QP}_d_yuvqp${DEPTH_QP}_fps${FPS}_s_nocull_c_nocull.csv"
save_dbitrate_file="dbitrate_cqp${COLOR_QP}_d_yuvqp${DEPTH_QP}_fps${FPS}_s_nocull_c_nocull.csv"

# save_ptcl_path=""
# render_image_path=""
# save_cbitrate_file=""
# save_dbitrate_file=""

taskset --cpu-list 0-19 ../../../build/Multiview/MultiviewClientPoolNew \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --client_fps=$FPS \
        --send_ptcl=0 \
        --save_frame=true \
        --view_ptcl=3 \
        --client_cull=0 \
        --output_dir=$output_dir \
        --render_image_path=$render_image_path \
        --save_ptcl_path=$save_ptcl_path \
        --save_cbitrate_file=$save_cbitrate_file \
        --save_dbitrate_file=$save_dbitrate_file \
        > log/client_band2.log 2>&1
