'''
Visualize cutouts and IoU
'''

import lego_online_infer
import cv2
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import minimum_filter, maximum_filter
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

import sys
import os
import time
from tqdm import tqdm
import subprocess
import argparse
from datetime import datetime


def _combine(img, transformed_cutout, color, size=5):
    if transformed_cutout is None:
        return img
    
    preprocessed_img = np.copy(img)

    min_vals = minimum_filter(transformed_cutout.astype(np.uint8), size=size)
    max_vals = maximum_filter(transformed_cutout.astype(np.uint8), size=size)
    mask = (min_vals == 0) & (max_vals == 255)

    preprocessed_img[mask == True] = color

    return preprocessed_img


def _color_gradient(score):
    colors = [(0.0, "#FF0000"), (0.5, "#FF0000"), (0.87, "#FFE600"), (0.95, "#00FF00"), (1.0, "#00FF00")]
    cmap = LinearSegmentedColormap.from_list("hex_gradient", colors)
    return [c*255 for c in cmap(score)[:3]]


def visualize(results, cur_assembling_step=-1, save_path=""):
    best_id = results.best_sim_id
    best_score = results.best_score
    details = results.details

    if best_id is not None:
        color1 = _color_gradient(details[best_id]['cam1_iou'])
        color2 = _color_gradient(details[best_id]['cam2_iou'])
    else:
        color1 = color2 = [255, 0, 0]
        
    img1_segmented = results.live_crop_cam1
    img1_segmented = _combine(img1_segmented, results.live_cutout_cam1, [255,255,255], size=3)
    img1_segmented = _combine(img1_segmented, results.transformed_cutout_sim_cam1, color1, size=7)

    img2_segmented = results.live_crop_cam2
    img2_segmented = _combine(img2_segmented, results.live_cutout_cam2, [255,255,255], size=3)
    img2_segmented = _combine(img2_segmented, results.transformed_cutout_sim_cam2, color2, size=7)

    combined_img = np.hstack((img1_segmented, img2_segmented))


    text_lines = [f'Step: {best_id}', f'Expected: {cur_assembling_step}', f'Score: {best_score:.4f}']

    (text_width, text_height), _ = cv2.getTextSize(text_lines[0], cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
    padding = 20
    total_text_height = len(text_lines) * text_height + (len(text_lines) + 1) * padding
    white_bg = np.ones((total_text_height, combined_img.shape[1], 3), dtype=np.uint8) * 255

    for i, text in enumerate(text_lines):
        text_x = 20
        text_y = (padding + text_height) * (i+1)
        cv2.putText(white_bg, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
    
    final_img = np.vstack((combined_img, white_bg))
    
    if save_path:
        cv2.imwrite(save_path, cv2.cvtColor(final_img, cv2.COLOR_RGB2BGR))

    return final_img
    

def parse_expected_step_file(expected_step_txt_path):
    with open(expected_step_txt_path) as f:
        expected_steps = [*map(int, f.read().strip().split('\n'))]
    return lambda i: expected_steps[i]


def num_frames(task, SIM_DATA_ROOT):
    assembly_key = task
    live_image_dir1 = Path(SIM_DATA_ROOT, "cam1", assembly_key)
    live_image_dir2 = Path(SIM_DATA_ROOT, "cam2", assembly_key)

    # open the live image folders and count the number of images
    live_images_cam1 = list(live_image_dir1.glob("*.jpg"))
    live_images_cam2 = list(live_image_dir2.glob("*.jpg"))
    length = min(len(live_images_cam1), len(live_images_cam2))  # Ensure both cameras have the same number of images
    assert(length > 0)
    
    return length


def load_frame(task, frame_idx, SIM_DATA_ROOT):
    assembly_key = task
    live_image_dir1 = Path(SIM_DATA_ROOT, "cam1", assembly_key)
    live_image_dir2 = Path(SIM_DATA_ROOT, "cam2", assembly_key)

    img1_name = f'{frame_idx:06d}.jpg'
    img2_name = f'{frame_idx:06d}.jpg'
    live_image_cam1_np = cv2.cvtColor(cv2.imread(str(live_image_dir1 / img1_name), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)  # Convert to RGB
    live_image_cam2_np = cv2.cvtColor(cv2.imread(str(live_image_dir2 / img2_name), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)  # Convert to RGB
    live_depth_cam1_np = np.load(str(live_image_dir1 / img1_name).replace('.jpg', '_depth.npz'))['arr_0']
    live_depth_cam2_np = np.load(str(live_image_dir2 / img2_name).replace('.jpg', '_depth.npz'))['arr_0']

    if live_image_cam1_np is None or live_image_cam2_np is None:
        print(f"Error: Failed to read one of the live images for assembly key '{assembly_key}'.")
        exit(0)

    return live_image_cam1_np, live_image_cam2_np, live_depth_cam1_np, live_depth_cam2_np


'''
Example:

python3 lego_visualize.py --task=$task
python3 lego_visualize.py --task=$task --expected_path='outputs/videos/lego_sideview_'$task'_expected.txt'

'''

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Example script using argparse")
    parser.add_argument("--task", type=str, help='cliff, R, faucet, etc.', required=True)
    parser.add_argument("--use_expected", action='store_true')

    args = parser.parse_args()

    task = args.task
    temp_video_dir = f'output_videos/temp'
    video_dir = f'output_videos'
    step_path = f'outputs/assembling_step/{args.task}.txt'

    cleanup = True

    ################

    if args.use_expected:
        get_expected_step = parse_expected_step_file(step_path)
        output_filename = f'{video_dir}/lego_sideview_{task}_withstep_{datetime.now().strftime("%m%d%y")}'
    else:
        get_expected_step = lambda _: -1
        output_filename = f'{video_dir}/lego_sideview_{task}_nostep_{datetime.now().strftime("%m%d%y")}'
    
    ################

    print('Initializing inferer...')
    SIM_DATA_ROOT = "outputs" # Root dir where sim_cam1/, sim_cam2/ exist
    SAM2_CHECKPOINT = "./checkpoints/sam2.1_hiera_large.pt" # Path to your SAM2 model
    SAM2_CONFIG = "configs/sam2.1/sam2.1_hiera_l.yaml"    # Path to your SAM2 config

    inferer = lego_online_infer.OnlineLegoInferer(
        sim_data_root_dir=SIM_DATA_ROOT,
        sam2_checkpoint_path=SAM2_CHECKPOINT,
        sam2_model_config_path=SAM2_CONFIG,
        device="cuda" # Use "cuda" if available and desired
    )
    
    # ################

    if cleanup:
        inferer.cleanup_all_temp_dirs()
    os.makedirs(temp_video_dir, exist_ok=True)

    predictions_str = ''

    length = num_frames(task, SIM_DATA_ROOT)
    for frame_idx in tqdm(range(length), total=length):
        cur_assembling_step = get_expected_step(frame_idx)
        
        live_image_cam1_np, live_image_cam2_np, live_depth_cam1_np, live_depth_cam2_np = load_frame(task, frame_idx, SIM_DATA_ROOT)

        results = inferer.infer_dual_camera(
            live_image_cam1_np,
            live_image_cam2_np,
            task,
            live_depth_cam1_np=live_depth_cam1_np,
            live_depth_cam2_np=live_depth_cam2_np,
            cur_assembling_step=cur_assembling_step,
            compute_new_T=False,
            save=False
        )

        visualize(results, cur_assembling_step, save_path=f'{temp_video_dir}/{frame_idx:06d}.png')

        with open(output_filename + '.txt', 'a') as f:
            f.write(str(results.best_sim_id) + '\n')

    subprocess.call(f'ffmpeg -y -framerate 2 -i {temp_video_dir}/%06d.png -c:v libx264 -r 2 -pix_fmt yuv420p {output_filename}.mp4', shell=True, text=True)