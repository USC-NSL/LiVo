import cv2
import os
from matplotlib import pyplot as plt
from pyzbar.pyzbar import decode
import shutil
import glob
from skimage import metrics
import numpy as np
import argparse

# Input images are color images

def ssim_psnr_rmse(img1, img2, multi_channel=False):
    with_alpha = False
    assert(img1.shape == img2.shape)
    if img1.shape[2] == 4:
        with_alpha = True
    
    if not multi_channel:
        if not with_alpha:
            img1_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            img2_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        else:
            img1_gray = cv2.cvtColor(img1, cv2.COLOR_BGRA2GRAY)
            img2_gray = cv2.cvtColor(img2, cv2.COLOR_BGRA2GRAY)
        ssim, diff = metrics.structural_similarity(img1_gray, img2_gray, full=True)
        psnr = metrics.peak_signal_noise_ratio(img1_gray, img2_gray)
        if(np.isinf(psnr)):
            psnr = 100.0
        rmse = np.sqrt(metrics.mean_squared_error(img1_gray, img2_gray))
        
    else:
        if not with_alpha:
            img1_color = cv2.cvtColor(img1, cv2.COLOR_BGRA2BGR)
            img2_color = cv2.cvtColor(img2, cv2.COLOR_BGRA2BGR)
        ssim, diff = metrics.structural_similarity(img1_color, img2_color, full=True, multichannel=True)
        psnr = metrics.peak_signal_noise_ratio(img1_color, img2_color)
        if(np.isinf(psnr)):
            psnr = 100.0
        rmse = np.sqrt(metrics.mean_squared_error(img1_color, img2_color))
    
    return ssim, psnr, rmse, diff

def eval_quality(start_id, end_id, gt_path, gen_path, out_file, diff_path=""):
    avg_ssim = 0.0
    avg_psnr = 0.0
    avg_rmse = 0.0
    num_frames = 0

    f = open(out_file, "w")
    f.write("FrameID,SSIM,PSNR,RMSE\n")

    for frame_id in range(start_id, end_id + 1):
        gt_file = os.path.join(gt_path, str(frame_id) + ".png")
        gen_file = os.path.join(gen_path, str(frame_id) + ".png")
        if not os.path.exists(gt_file):
            print("Ground truth file does not exist: ", gt_file)
            continue
        if not os.path.exists(gen_file):
            print("Generated file does not exist: ", gen_file)
            continue
        img1 = cv2.imread(gt_file, cv2.IMREAD_COLOR)
        img2 = cv2.imread(gen_file, cv2.IMREAD_COLOR)
        if img1 is None or img2 is None:
            print("Could not read the image: ", gt_file, gen_file)
            continue
        ssim, psnr, rmse, diff = ssim_psnr_rmse(img1, img2, multi_channel=True)
        if diff_path != "":
            diff = (diff * 255).astype("uint8")
            cv2.imwrite(os.path.join(diff_path, str(frame_id) + ".png"), diff)
        print("Frame: ", frame_id, " SSIM: ", ssim, " PSNR: ", psnr, " RMSE: ", rmse)
        f.write(str(frame_id) + "," + str(ssim) + "," + str(psnr) + "," + str(rmse) + "\n")
        avg_ssim += ssim
        avg_psnr += psnr
        avg_rmse += rmse
        num_frames += 1
    
    avg_ssim /= num_frames
    avg_psnr /= num_frames
    avg_rmse /= num_frames
    print("Average SSIM: ", avg_ssim, " Average PSNR: ", avg_psnr, " Average RMSE: ", avg_rmse)
    return avg_ssim, avg_psnr, avg_rmse
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Evaluate the quality of the images')
    parser.add_argument('--start_id', type=int, help='Start id of the images', required=True)
    parser.add_argument('--end_id', type=int, help='End id of the images', required=True)
    parser.add_argument('--gt_path', type=str, help='Path to the path containing ground truth images', required=True)
    parser.add_argument('--gen_path', type=str, help='Path to the path containing the generated images', required=True)
    parser.add_argument('--out_file', type=str, help='Path to the output file containing quality metrics', required=True)
    parser.add_argument('--diff_path', default="", type=str, help='Path to the path containing output images, such as diff images', required=False)
    args = parser.parse_args()
    
    gt_path = args.gt_path
    gen_path = args.gen_path
    diff_path = args.diff_path
    out_file = args.out_file

    print("Ground truth path: ", gt_path)
    print("Generated path: ", gen_path)
    print("Output file: ", out_file)
    print("Diff path: ", diff_path)

    if not os.path.exists(gt_path):
        print("Ground truth path does not exist")
        exit()
    
    if not os.path.exists(gen_path):
        print("Generated path does not exist")
        exit()

    if diff_path != "" and not os.path.exists(diff_path):
        os.makedirs(diff_path)
    
    eval_quality(args.start_id, args.end_id, gt_path, gen_path, out_file, diff_path)

# Example usage
# python evaluate_quality.py --start_id 59 --end_id 4059 --gt_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_gt_client_cull --gen_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_nvenc_c15m_d_yuv30m_fps30_wo_cull --out_file /home/lei/rajrup/KinectStream/Scripts/Experiments/output/160317_moonbaby1_no_ground/gt_vs_o3d_nvenc_c15m_d_yuv30m_fps30_wo_cull.csv
    
# python evaluate_quality.py --start_id 59 --end_id 4059 --gt_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_gt_client_cull --gen_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_nvenc_c100m_d_yuv200m_fps30_wo_cull --out_file /home/lei/rajrup/KinectStream/Scripts/Experiments/output/160317_moonbaby1_no_ground/gt_vs_o3d_nvenc_c100m_d_yuv200m_fps30_wo_cull.csv
    
# python evaluate_quality.py --start_id 59 --end_id 4059 --gt_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_gt_client_cull --gen_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_nvenc_c15m_d_yuv30m_fps30_w_pcull --out_file /home/lei/rajrup/KinectStream/Scripts/Experiments/output/160317_moonbaby1_no_ground/gt_vs_o3d_nvenc_c15m_d_yuv30m_fps30_w_pcull.csv

# python evaluate_quality.py --start_id 59 --end_id 4059 --gt_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_gt_client_cull --gen_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_nvenc_c100m_d_yuv200m_fps30_w_pcull --out_file /home/lei/rajrup/KinectStream/Scripts/Experiments/output/160317_moonbaby1_no_ground/gt_vs_o3d_nvenc_c100m_d_yuv200m_fps30_w_pcull.csv
    
# python evaluate_quality.py --start_id 59 --end_id 4059 --gt_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_gt_client_cull --gen_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_nvenc_c15m_d_yuv30m_fps30_w_kpcull --out_file /home/lei/rajrup/KinectStream/Scripts/Experiments/output/160317_moonbaby1_no_ground/gt_vs_o3d_nvenc_c15m_d_yuv30m_fps30_w_kpcull.csv

# python evaluate_quality.py --start_id 59 --end_id 4059 --gt_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_gt_client_cull --gen_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_nvenc_c100m_d_yuv200m_fps30_w_kpcull --out_file /home/lei/rajrup/KinectStream/Scripts/Experiments/output/160317_moonbaby1_no_ground/gt_vs_o3d_nvenc_c100m_d_yuv200m_fps30_w_kpcull.csv
    
# python evaluate_quality.py --start_id 59 --end_id 4059 --gt_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_nvenc_c15m_d_yuv30m_fps30_w_pcull --gen_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_nvenc_c15m_d_yuv30m_fps30_w_kpcull --out_file /home/lei/rajrup/KinectStream/Scripts/Experiments/output/160317_moonbaby1_no_ground/pcull_vs_o3d_nvenc_c15m_d_yuv30m_fps30_w_kpcull.csv

# python evaluate_quality.py --start_id 59 --end_id 4059 --gt_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_nvenc_c100m_d_yuv200m_fps30_w_pcull --gen_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_no_ground/o3d_nvenc_c100m_d_yuv200m_fps30_w_kpcull --out_file /home/lei/rajrup/KinectStream/Scripts/Experiments/output/160317_moonbaby1_no_ground/pcull_vs_o3d_nvenc_c100m_d_yuv200m_fps30_w_kpcull.csv
    


# python evaluate_quality.py --start_id 59 --end_id 4059 --gt_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_with_ground/o3d_gt_client_cull --gen_path /datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_with_ground/o3d_nvenc_c15m_d_yuv30m_fps30_w_kpcull_l3 --out_file /home/lei/rajrup/KinectStream/Scripts/Experiments/output/160317_moonbaby1_with_ground/gt_vs_o3d_nvenc_c15m_d_yuv30m_fps30_w_kpcull_l3.csv