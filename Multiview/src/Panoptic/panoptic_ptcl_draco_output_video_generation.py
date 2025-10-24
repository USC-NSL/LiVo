import os
import cv2
import numpy as np
import time
import argparse
import json

FPS = 15

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--seq_name', type=str, help='Sequence name', required=True)
    parser.add_argument('--mm_trace_name', type=str, help='MM trace name', required=True)
    parser.add_argument('--start_frame_id', type=int, help='Start frame id', required=True)
    parser.add_argument('--end_frame_id', type=int, help='End frame id', required=True)
    parser.add_argument('--config_file', type=str, help='Path to Config file', required=True)
    options = parser.parse_args()
    seq_name = options.seq_name
    mm_trace_name = options.mm_trace_name
    START_FRAME = options.start_frame_id
    END_FRAME = options.end_frame_id

    config_file_path = options.config_file
    with open(config_file_path) as f:
        config = json.load(f)
        logID = int(config["log_id"])

    compression_type = "draco"
    output_path = os.path.join("/datassd/pipeline_ptcl/client_ptcl/draco/", seq_name, mm_trace_name, f"log{logID}")

    print(f"Generating video for {seq_name} {mm_trace_name} log{logID} from frame {START_FRAME} to {END_FRAME} with {compression_type} compression")

    start_img_path = os.path.join(output_path, f"{START_FRAME}.png")
    if not os.path.exists(start_img_path):
        filelist = os.listdir(output_path)
        filelist.sort()
        start_img_path = os.path.join(output_path, filelist[0])
    img = cv2.imread(start_img_path)
    height, width, _ = img.shape

    # Dispaly images on opencv window at FPS
    cv2.namedWindow('Display', cv2.WINDOW_GUI_NORMAL)
    cv2.resizeWindow('Display', width, height)
    
    inter_frame_delay = 1000//FPS
    for i in range(START_FRAME+1, END_FRAME+1):
        start_time = time.time()
        frame_path = os.path.join(output_path, f"{i}.png")
        if os.path.exists(frame_path):
            img = cv2.imread(frame_path)
        delay = max(1, inter_frame_delay - int((time.time() - start_time)*1000))

        print(f"Frame {i} delay: {delay}")
        cv2.imshow('Display', img)
        cv2.waitKey(delay)
    cv2.destroyAllWindows()