"""
Class for detecting Lego assembly state geometrically by comparing against a library of simulated images
"""
import os
import glob
import json
import cv2
import numpy as np
from pathlib import Path
import time
import shutil
from PIL import Image # For saving numpy array to pass to segmenter
import matplotlib.pyplot as plt
from types import SimpleNamespace

# Assuming lego_segmenter.py is in the Python path or same directory
from lego_segmenter import LegoSegmenter
# For a real setup, you'd import these from lego_match_sim.py:
from lego_match_sim import calculate_mask_iou, extract_sim_id_from_filename

class OnlineLegoInferer:
    """
    Handles online inference for Lego assembly state detection using dual cameras.
    It segments live images, compares them with simulation masks via IoU,
    and determines the best matching simulation state.
    """

    def __init__(self,
                 sim_data_root_dir: str,
                 sam2_checkpoint_path: str,
                 sam2_model_config_path: str,
                 temp_base_dir: str = "temp_online_inference",
                 device: str = "cuda",
                 dino_box_threshold: float = 0.3,
                 dino_text_threshold: float = 0.2,
                 segmenter_text_prompt: str = "assembled lego structure."
                 ):
        self.sim_data_root_dir = Path(sim_data_root_dir)
        self.temp_base_dir = Path(temp_base_dir)
        self.device = device
        self.IMAGE_EXTENSIONS = ('.png',)

        shutil.rmtree(self.temp_base_dir, ignore_errors=True) # Clean up from previous runs
        self.temp_base_dir.mkdir(parents=True, exist_ok=True)
        
        #self.temp_input_image_dir = self.temp_base_dir / "live_inputs"
        #self.temp_input_image_dir.mkdir(parents=True, exist_ok=True)

        segmenter_cam1_output_dir = self.temp_base_dir / "segmenter_cam1_out"
        segmenter_cam1_output_dir.mkdir(parents=True, exist_ok=True)
        self.segmenter_cam1 = LegoSegmenter(
            sam2_checkpoint_path=sam2_checkpoint_path,
            sam2_model_config_path=sam2_model_config_path,
            camera_name="cam1",
            output_dir=str(segmenter_cam1_output_dir),
            device=device,
            dump_json_results=False,
            dino_box_threshold=dino_box_threshold,
            dino_text_threshold=dino_text_threshold,
            default_text_prompt=segmenter_text_prompt
        )

        segmenter_cam2_output_dir = self.temp_base_dir / "segmenter_cam2_out"
        segmenter_cam2_output_dir.mkdir(parents=True, exist_ok=True)
        self.segmenter_cam2 = LegoSegmenter(
            sam2_checkpoint_path=sam2_checkpoint_path,
            sam2_model_config_path=sam2_model_config_path,
            camera_name="cam2",
            output_dir=str(segmenter_cam2_output_dir),
            device=device,
            dump_json_results=False,
            dino_box_threshold=dino_box_threshold,
            dino_text_threshold=dino_text_threshold,
            default_text_prompt=segmenter_text_prompt
        )
        self.sim_mask_info_cache = {} # Cache: {assembly_key: {sim_cam_name: {sim_id: path}}}
        self.count = 0

        # based on 05/26/2025 calibration
        self.calibrated_T = {'sim_cam1': (np.array([[0.9277678728103638, 0.0, 39.141693115234375], [0.0, 0.9277678728103638, 60.713409423828125], [0.0, 0.0, 1.0]]),(0.8585859110425732, 68.25756399151715)),
		'sim_cam2': (np.array([[0.9247972369194031, 0.0, 126.83160400390625], [0.0, 0.9247972369194031, 13.537094116210938], [0.0, 0.0, 1.0]]),(1.281410217285156, 25.6279296875))}
        
    def _get_simulation_masks_for_key_and_cam(self, assembly_key: str, sim_camera_name: str): # -> dict[int, str]:
        """
        Loads and caches paths to simulation masks for a given assembly_key and sim_camera_name.
        Returns a dictionary mapping sim_id (int) to mask_path (str).
        sim_camera_name: "sim_cam1" or "sim_cam2".
        """
        if assembly_key in self.sim_mask_info_cache and \
           sim_camera_name in self.sim_mask_info_cache[assembly_key]:
            return self.sim_mask_info_cache[assembly_key][sim_camera_name]

        sim_ref_dir = self.sim_data_root_dir / sim_camera_name / f"{assembly_key}"
        id_to_mask = {}
        id_to_box = {}
        id_to_depth = {}

        if not sim_ref_dir.is_dir():
            print(f"Warning: Simulation reference directory not found: {sim_ref_dir}")
        else:
            sim_files_in_dir = []
            print(sim_ref_dir)
            for ext in self.IMAGE_EXTENSIONS:
                for path in sim_ref_dir.glob(f"cutout_*{ext}"):
                    if "depth." not in path.name:
                        sim_files_in_dir.append(path)
                # sim_files_in_dir.extend(list(sim_ref_dir.glob(f"cutout_*{ext}"))) # More specific glob

            for sim_path in sim_files_in_dir:
                sim_id = extract_sim_id_from_filename(sim_path.name)
                if sim_id is not None:
                    # sim_grayscale = cv2.imread(str(sim_path), cv2.IMREAD_GRAYSCALE)
                    sim_bgra = cv2.imread(str(sim_path), cv2.IMREAD_UNCHANGED)
                    sim_bw = self.rgba_to_bw(sim_bgra)
                    id_to_mask[sim_id] = sim_bw
                    id_to_box[sim_id] = self.get_nontransparent_bbox(sim_bw, T=self.calibrated_T[sim_camera_name][0])
                    sim_depth = cv2.imread(str(sim_path).replace('.', '_depth.'), cv2.IMREAD_UNCHANGED)
                    id_to_depth[sim_id] = sim_depth

        if assembly_key not in self.sim_mask_info_cache:
            self.sim_mask_info_cache[assembly_key] = {}
        self.sim_mask_info_cache[assembly_key][sim_camera_name] = (id_to_mask, id_to_box, id_to_depth)
        return id_to_mask, id_to_box, id_to_depth

    def _segment_live_image(self, image_data_np: np.ndarray, camera_name: str, depth_data_np: np.ndarray =None, sim_cam_box=None, display_plt=False): # -> np.ndarray | None:
        """
        Segments a live image (NumPy array, RGB) using LegoSegmenter in-memory.
        Returns the primary cutout mask as a NumPy array (uint8, 0 or 255), or None.
        """
        segmenter = self.segmenter_cam1 if camera_name == "cam1" else self.segmenter_cam2
        
        cropped_img, mask_np = segmenter.generate_single_mask_from_data(image_data_np, sim_cam_box, text_prompt=segmenter.default_text_prompt, save_id = self.count, display_plt=display_plt) 
        
        if mask_np is None:
            # print(f"Debug: Segmentation returned None for live image from {camera_name}.")
            pass # Error already printed by LegoSegmenter or this method earlier
        
        if depth_data_np is not None:
            cropped_depth = segmenter.load_and_preprocess_depth_image(depth_data_np)
            masked_depth = segmenter.cutout_depth(np.asarray(cropped_depth), cutout=None)
        else:
            masked_depth = None

        return cropped_img, mask_np, masked_depth

    # box from sim image for SAM prompt
    # transform to align with live image
    def get_nontransparent_bbox(self, bw_img, T=None, margin=5):
        non_zero = np.argwhere(bw_img > 0)
        if non_zero.size == 0:
            return (0,0,0,0)  # Fully transparent image
        (y_min, x_min), (y_max, x_max) = non_zero.min(axis=0), non_zero.max(axis=0)
        
        if T is not None:
            x_h = np.array([x_min, y_min, 1])
            x_transformed = T @ x_h 
            x_min, y_min = x_transformed[:2]

            x_h = np.array([x_max, y_max, 1])
            x_transformed = T @ x_h 
            x_max, y_max = x_transformed[:2]

        return (x_min-margin, y_min-margin, x_max+margin, y_max+margin)

    def rgba_to_bw(self, img):
        if img is None:
            return None
        
        assert(len(img.shape) == 3 and img.shape[2] == 4)
        alpha = img[:, :, 3]

        output = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8) 
        output[alpha > 0] = 255

        return output

    def infer_dual_camera(self,
                          live_image_cam1_np: np.ndarray, # Expected RGB
                          live_image_cam2_np: np.ndarray, # Expected RGB
                          assembly_key: str,
                          live_depth_cam1_np: np.ndarray =None,
                          live_depth_cam2_np: np.ndarray =None,
                          cur_assembling_step: int =-1,
                          compute_new_T=True,
                          save=True,
                          display=False,
                          display_ids=[]): # -> tuple[int | None, float, dict]:
        """
        Performs inference using two live camera images against simulation masks.

        Args:
            live_image_cam1_np: NumPy array for camera 1 image (RGB).
            live_image_cam2_np: NumPy array for camera 2 image (RGB).
            assembly_key: The key for the current assembly stage (e.g., "S").

            compute_new_T: detect and match features to align images
            save: save images to temp directory

        Returns:
            Tuple: (best_overall_sim_id, max_combined_iou_score, per_sim_id_details)
            - best_overall_sim_id (int | None): The sim_id with the highest combined IoU.
            - max_combined_iou_score (float): The score itself.
            - per_sim_id_details (dict): Detailed IoUs for each sim_id.
              { sim_id: {'cam1_iou': float, 'cam2_iou': float, 'combined_iou': float,
                         'cam1_sim_mask': str|None, 'cam2_sim_mask': str|None,
                         'live_cutout_cam1': str|None, 'live_cutout_cam2': str|None }, ... }
        """
        live_image_unique_id = f"{assembly_key}_{(time.time_ns()//100000000):d}"

        self.count += 1

        # get sim cutout for each sim id
        sim_masks_cam1_map, sim_boxes_cam1_map, sim_depth_cam1_map = self._get_simulation_masks_for_key_and_cam(assembly_key, "sim_cam1")
        sim_masks_cam2_map, sim_boxes_cam2_map, sim_depth_cam2_map = self._get_simulation_masks_for_key_and_cam(assembly_key, "sim_cam2")

        if not sim_masks_cam1_map and not sim_masks_cam2_map:
            print(f"Warning: No simulation masks found for assembly key '{assembly_key}' for any camera.")
            return None, None, None, None, None, -1.0, {}
        unique_sim_ids = set(sim_masks_cam1_map.keys()).union(set(sim_masks_cam2_map.keys()))
        if not unique_sim_ids:
            print(f"Warning: No sim_ids could be extracted for assembly key '{assembly_key}'.")
            return None, None, None, None, None, -1.0, {}

        # segment using expected box from simulation, if cur_assembling_step provided
        sim_cam1_box = sim_boxes_cam1_map.get(cur_assembling_step) # None if cur_assembling_step is -1
        sim_cam2_box = sim_boxes_cam2_map.get(cur_assembling_step)
        live_crop_cam1_data, live_cutout_cam1_data_rgba, live_depth_cam1_data = self._segment_live_image(live_image_cam1_np, "cam1", 
                                                                                                         depth_data_np=live_depth_cam1_np,
                                                                                                         sim_cam_box=sim_cam1_box, display_plt=display)
        live_crop_cam2_data, live_cutout_cam2_data_rgba, live_depth_cam2_data = self._segment_live_image(live_image_cam2_np, "cam2", 
                                                                                                         depth_data_np=live_depth_cam2_np,
                                                                                                         sim_cam_box=sim_cam2_box, display_plt=display)

        if live_cutout_cam1_data_rgba is None and live_cutout_cam2_data_rgba is None:
            print(f"Error: Segmentation failed or no objects found for both cameras for assembly key '{assembly_key}'.")
            return None, None, None, None, None, -1.0, {}
        assert((live_cutout_cam1_data_rgba is not None and live_cutout_cam1_data_rgba.shape[2] == 4) 
               or (live_cutout_cam2_data_rgba is not None and live_cutout_cam2_data_rgba.shape[2] == 4))

        # Save the segmented cutouts to temporary files (image is rgb)
        if save:
            if live_cutout_cam1_data_rgba is not None:
                live_cutout_cam1_path = self.temp_base_dir / f"live_cutout_{self.count:06d}_cam1.png"
                bgr_cutout_cam1_data = cv2.cvtColor(live_cutout_cam1_data_rgba, cv2.COLOR_RGBA2BGRA)
                cv2.imwrite(str(live_cutout_cam1_path), bgr_cutout_cam1_data)
            if live_cutout_cam2_data_rgba is not None:
                live_cutout_cam2_path = self.temp_base_dir / f"live_cutout_{self.count:06d}_cam2.png"
                bgr_cutout_cam2_data = cv2.cvtColor(live_cutout_cam2_data_rgba, cv2.COLOR_RGBA2BGRA)
                cv2.imwrite(str(live_cutout_cam2_path), bgr_cutout_cam2_data)

        # calculate iou to get best sim_id
        best_overall_sim_id = None
        max_combined_iou = -np.inf
        max_combined_depthscore = -np.inf
        best_cam1_transform_results = (None, None) # (transformed_cutout_sim_cam, T)
        best_cam2_transform_results = (None, None)
        all_sim_id_results = {}

        live_cutout_cam1_data = self.rgba_to_bw(live_cutout_cam1_data_rgba)
        live_cutout_cam2_data = self.rgba_to_bw(live_cutout_cam2_data_rgba)

        for sim_id in unique_sim_ids:
            # Skip sim_ids that are beyond the current assembly step
            if cur_assembling_step > -1 and sim_id > cur_assembling_step:
                continue

            display_plt = display and (display_ids == [] or sim_id in display_ids)
            if display_plt:
                print("Sim_id", sim_id)

            ious = []
            depthscores = []

            sim_mask1 = sim_masks_cam1_map.get(sim_id)
            sim_depth1 = sim_depth_cam1_map.get(sim_id)
            if sim_mask1 is not None and live_cutout_cam1_data_rgba is not None: # Ensure sim exist
                # live_cutout_cam1_data can be None, _calculate_mask_iou handles it
                # convert to grayscale for IoU calculation
                iou_cam1, depthscore_cam1, transformed_cutout_sim_cam1, T1 = calculate_mask_iou(live_cutout_cam1_data, sim_mask1, 
                                                                            live_depth_cam1_data, sim_depth1,
                                                                            ref_T=self.calibrated_T['sim_cam1'], 
                                                                            compute_new_T=compute_new_T,
                                                                            display_plt=display_plt)
                if iou_cam1 >= 0:
                    ious.append(iou_cam1)
                depthscores.append(depthscore_cam1)
            else:
                transformed_cutout_sim_cam1 = T1 = None

            sim_mask2 = sim_masks_cam2_map.get(sim_id)
            sim_depth2 = sim_depth_cam2_map.get(sim_id)
            if sim_mask2 is not None and live_cutout_cam2_data_rgba is not None: # Ensure sim exist
                # live_cutout_cam2_data can be None, _calculate_mask_iou handles it
                iou_cam2, depthscore_cam2, transformed_cutout_sim_cam2, T2 = calculate_mask_iou(live_cutout_cam2_data, sim_mask2, 
                                                                            live_depth_cam2_data, sim_depth2,
                                                                            ref_T=self.calibrated_T['sim_cam2'], 
                                                                            compute_new_T=compute_new_T,
                                                                            display_plt=display_plt)
                if iou_cam2 >= 0:
                    ious.append(iou_cam2)
                depthscores.append(depthscore_cam2)
            else:
                transformed_cutout_sim_cam2 = T2 = None
             
            combined_iou = max(ious)
            combined_depthscore = sum(depthscores)/len(depthscores)

            all_sim_id_results[sim_id] = {
                'cam1_iou': iou_cam1, # Will be 0 if live_cutout_cam1_data was None or sim_mask_p1 was None
                'cam2_iou': iou_cam2,
                'combined_iou': combined_iou,
                'cam1_depthscore': depthscore_cam1, # Will be 0 if live_cutout_cam1_data was None or sim_mask_p1 was None
                'cam2_depthscore': depthscore_cam2,
                'combined_depthscore': combined_depthscore,
                'cam1_sim_mask': sim_mask1,
                'cam2_sim_mask': sim_mask2,
            }

            # if iou slightly lower than max, still update as long depth score is better
            if (combined_iou > max_combined_iou) or \
                (max_combined_iou - combined_iou < 0.01 and max_combined_depthscore < combined_depthscore): 
                
                max_combined_iou = combined_iou
                max_combined_depthscore = combined_depthscore
                best_overall_sim_id = sim_id
                best_cam1_transform_results = (transformed_cutout_sim_cam1, T1)
                best_cam2_transform_results = (transformed_cutout_sim_cam2, T2)

        best_transformed_cutout_sim_cam1, best_T1 = best_cam1_transform_results
        best_transformed_cutout_sim_cam2, best_T2 = best_cam2_transform_results

        results = {'live_crop_cam1': live_crop_cam1_data,
                   'live_crop_cam2': live_crop_cam2_data,
                   'live_cutout_cam1': live_cutout_cam1_data,
                   'live_cutout_cam2': live_cutout_cam2_data,
                   'transformed_cutout_sim_cam1': best_transformed_cutout_sim_cam1,
                   'transformed_cutout_sim_cam2': best_transformed_cutout_sim_cam2,
                   'T1': best_T1,
                   'T2': best_T2,
                   'best_sim_id': best_overall_sim_id,
                   'best_score': max_combined_iou,
                   'details': all_sim_id_results}
        return SimpleNamespace(**results)

    def cleanup_all_temp_dirs(self):
        """Cleans up the base temporary directory used by this instance."""
        print(f"Cleaning up base temporary directory: {self.temp_base_dir}")
        shutil.rmtree(self.temp_base_dir, ignore_errors=True)

# Example Usage (Illustrative - not for ROS yet):
if __name__ == "__main__":
    # --- Configuration ---
    # Adjust these paths based on your project structure
    SIM_DATA_ROOT = "outputs" # Root dir where sim_cam1/, sim_cam2/ exist
    SAM2_CHECKPOINT = "./checkpoints/sam2.1_hiera_large.pt" # Path to your SAM2 model
    SAM2_CONFIG = "configs/sam2.1/sam2.1_hiera_l.yaml"    # Path to your SAM2 config
    
    import argparse
    parser = argparse.ArgumentParser(description="Run online Lego inference with dual cameras.")
    parser.add_argument("task", type=str, help="Task name for the assembly (e.g., 'cliff').")
    
    args = parser.parse_args()
    assembly_key = args.task

    # Initialize inferer
    inferer = OnlineLegoInferer(
        sim_data_root_dir=SIM_DATA_ROOT,
        sam2_checkpoint_path=SAM2_CHECKPOINT,
        sam2_model_config_path=SAM2_CONFIG,
        device="cuda" # Use "cuda" if available and desired
    )

    live_image_dir1 = Path(SIM_DATA_ROOT, "cam1", assembly_key)
    live_image_dir2 = Path(SIM_DATA_ROOT, "cam2", assembly_key)

    # open the live image folders and count the number of images
    live_images_cam1 = list(live_image_dir1.glob("*.jpg"))
    live_images_cam2 = list(live_image_dir2.glob("*.jpg"))
    length = min(len(live_images_cam1), len(live_images_cam2))  # Ensure both cameras have the same number of images
    print(f"Starting inference for assembly key: {assembly_key}")

    for i in range(length):
        img1_name = f'{i:06d}.jpg'
        img2_name = f'{i:06d}.jpg'
        live_image_cam1_np = cv2.imread(str(live_image_dir1 / img1_name), cv2.IMREAD_COLOR)
        live_image_cam2_np = cv2.imread(str(live_image_dir2 / img2_name), cv2.IMREAD_COLOR)
        

        if live_image_cam1_np is None or live_image_cam2_np is None:
            print(f"Error: Failed to read one of the live images for assembly key '{assembly_key}'.")
            continue
        live_image_cam1_np = cv2.cvtColor(live_image_cam1_np, cv2.COLOR_BGR2RGB)  # Convert to RGB
        live_image_cam2_np = cv2.cvtColor(live_image_cam2_np, cv2.COLOR_BGR2RGB)
        
        results = inferer.infer_dual_camera(
            live_image_cam1_np,
            live_image_cam2_np,
            assembly_key
        )

        print(f"\n--- Inference Results {i+1} ---")
        print(f"Best Matching Sim ID: {results.best_sim_id}")
        print(f"Best Combined IoU Score: {results.best_score:.4f}")
        print("\nDetails:")
        for sim_id, data in results.details.items():
            print(f"  Sim ID {sim_id}:")
            print(f"    Cam1 IoU: {data['cam1_iou']:.4f} ")
            print(f"    Cam2 IoU: {data['cam2_iou']:.4f} ")
            print(f"    Combined IoU: {data['combined_iou']:.4f}")

    # Clean up all temporary files and directories created by the inferer
    #inferer.cleanup_all_temp_dirs()