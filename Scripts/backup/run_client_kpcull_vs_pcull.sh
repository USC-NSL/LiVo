# Band2
START_FRAME=2
END_FRAME=5517
FPS=30
COLOR_BITRATE=20000
# DEPTH_BITRATE=120000
DEPTH_BITRATE=60000

SEQ_NAME=160906_band2_with_ground
CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_160906_band2.json

# dance5
# START_FRAME=34
# END_FRAME=10000
# FPS=30
# COLOR_BITRATE=20000
# # DEPTH_BITRATE=120000
# DEPTH_BITRATE=60000

# SEQ_NAME=170307_dance5_with_ground
# CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_170307_dance5.json

# # office1
# START_FRAME=19
# END_FRAME=5600
# FPS=30
# COLOR_BITRATE=20000
# # DEPTH_BITRATE=120000
# DEPTH_BITRATE=60000

# SEQ_NAME=170915_office1_with_ground
# CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_170915_office1.json

# # pizza1
# START_FRAME=1
# END_FRAME=1413
# FPS=30
# COLOR_BITRATE=20000
# # DEPTH_BITRATE=120000
# DEPTH_BITRATE=60000

# SEQ_NAME=160906_pizza1_with_ground
# CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_160906_pizza1.json

# # toddler4
# START_FRAME=1
# END_FRAME=3800
# FPS=30
# COLOR_BITRATE=20000
# # DEPTH_BITRATE=120000
# DEPTH_BITRATE=60000

# SEQ_NAME=170915_toddler4_with_ground
# CONFIG_FILE=/home/lei/rajrup/KinectStream/Multiview/config/panoptic_170915_toddler4.json

COLOR_BITRATE_M=$(($COLOR_BITRATE/1000))
DEPTH_BITRATE_M=$(($DEPTH_BITRATE/1000))

save_ptcl_path="/datassd/pipeline/client_tiled/ablation/$SEQ_NAME/o3d_nvenc_s_kpcull_c_cull/tracep1/"
render_image_path="/datassd/pipeline/client_tiled/ablation/$SEQ_NAME/o3d_nvenc_s_kpcull_c_cull/tracep1/"
# save_ptcl_path=""
# render_image_path=""

# Remove old render images if exists
if [ -d "$render_image_path" ]; then
    echo "Removing old render images"
    rm -rf $render_image_path
fi

# Remove old render images if exists
if [ -d "$save_ptcl_path" ]; then
    echo "Removing old render images"
    rm -rf $save_ptcl_path
fi

taskset --cpu-list 4-19 ../build/Multiview/MultiviewClientPool \
        --seq_name=$SEQ_NAME \
        --config_file=$CONFIG_FILE \
        --start_frame_id=$START_FRAME \
        --end_frame_id=$END_FRAME \
        --send_ptcl=0 \
        --save_frame=false \
        --view_ptcl=3 \
        --client_cull=3 \
        --render_image_path=$render_image_path \
        --save_ptcl_path=$save_ptcl_path &

# taskset --cpu-list 4-19 ../build/Multiview/MultiviewClientPool \
#         --seq_name=$SEQ_NAME --start_frame_id=$START_FRAME --ncaptures=$NFRAMES --view_ptcl=3 --client_cull=3 --send_ptcl=0 --save_frame=false --render_image_path=$render_image_path &> MultiviewClientPool.log &

sleep 1
taskset --cpu-list 0-3 python3.8 ../WebRTC/receiver_new.py \
        -seq_name $SEQ_NAME \
        -sid $START_FRAME \
        -eid $END_FRAME \
        -v 0 \
        -f $FPS \
        -cb $COLOR_BITRATE \
        -db $DEPTH_BITRATE > ./Experiments/pipeline_log/receiver_tile.out &
