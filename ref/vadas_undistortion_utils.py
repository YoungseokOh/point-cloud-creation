import numpy as np
import cv2 as cv
import math
from typing import List, Tuple, Optional

# Import the VADASFisheyeCameraModel from ref_camera_lidar_projector
# This assumes ref_camera_lidar_projector is in the same directory or importable.
from ref.ref_camera_lidar_projector import VADASFisheyeCameraModel

def get_vadas_undistortion_maps(
    vadas_intrinsic: List[float],
    original_image_size: Tuple[int, int], # (width, height)
    rectified_size: Tuple[int, int],      # (width, height)
    rectified_K: Optional[np.ndarray] = None,
    depth_plane: float = 1.0, # Assumed depth of the plane in rectified camera coordinates
    rectified_fov_scale: float = 1.0 # Scale factor for rectified image FOV (<1.0 for wider FOV)
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Generates map1 and map2 for cv2.remap to undistort a VADAS fisheye image
    into a rectified (pinhole-like) view.

    This works by:
    1. Defining a virtual rectified (pinhole) camera.
    2. For each pixel in the *rectified* output image, unprojecting it to a 3D ray.
    3. Taking a point on this 3D ray (at `depth_plane` distance in the rectified camera's Z-axis).
    4. Projecting this 3D point through the VADAS fisheye model to find its corresponding
       (u, v) coordinates in the *original fisheye* image. These are our map values.

    Args:
        vadas_intrinsic: List of VADAS intrinsic parameters.
        original_image_size: (width, height) of the original fisheye image.
        rectified_size: (width, height) of the desired rectified output image.
        rectified_K: Optional. The intrinsic matrix for the virtual rectified camera.
                     If None, a default pinhole K is generated.
        depth_plane: The assumed depth (Z-coordinate) of the plane in the rectified
                     camera's coordinate system from which points are unprojected.
                     This affects the field of view of the rectified image.

    Returns:
        map1 (np.ndarray): X coordinates in the original image (float32).
        map2 (np.ndarray): Y coordinates in the original image (float32).
    """
    W_orig, H_orig = original_image_size
    W_rect, H_rect = rectified_size

    # Initialize VADAS camera model with original image size
    vadas_model = VADASFisheyeCameraModel(vadas_intrinsic, image_size=original_image_size)

    if rectified_K is None:
        # Generate a default pinhole K for the rectified camera.
        # This K should be chosen to capture a reasonable FOV from the fisheye.
        # A common heuristic is to set focal length to roughly half the image width/height.
        fx_rect = (W_rect / 2.0) * rectified_fov_scale
        fy_rect = (H_rect / 2.0) * rectified_fov_scale
        cx_rect = W_rect / 2.0
        cy_rect = H_rect / 2.0
        rectified_K = np.array([[fx_rect, 0, cx_rect],
                                [0, fy_rect, cy_rect],
                                [0, 0, 1]], dtype=np.float64)
    
    fx_rect, fy_rect = rectified_K[0, 0], rectified_K[1, 1]
    cx_rect, cy_rect = rectified_K[0, 2], rectified_K[1, 2]

    map_x = np.zeros((H_rect, W_rect), dtype=np.float32)
    map_y = np.zeros((H_rect, W_rect), dtype=np.float32)

    # Iterate over each pixel in the *output* (rectified) image
    for v_rect in range(H_rect):
        for u_rect in range(W_rect):
            # 1. Unproject (u_rect, v_rect) to a 3D point in the *rectified* camera's coordinate system.
            #    Assume a plane at `depth_plane` distance.
            #    X_cam_rect = (u_rect - cx_rect) / fx_rect * depth_plane
            #    Y_cam_rect = (v_rect - cy_rect) / fy_rect * depth_plane
            #    Z_cam_rect = depth_plane

            # 2. These (X_cam_rect, Y_cam_rect, Z_cam_rect) are the 3D coordinates in the *rectified* camera's frame.
            #    Now, we need to project these same 3D points through the VADAS fisheye model
            #    to find their corresponding (u, v) in the *original fisheye* image.
            #    The VADAS model's `project_point` expects (Xc, Yc, Zc) where Xc is forward depth.
            #    From `ref_camera_lidar_projector.py`'s `project_point`:
            #    `nx = -Yc`, `ny = -Zc`
            #    `theta = atan2(dist, Xc)`
            #    This implies VADAS X is forward, Y is right, Z is down.
            #    If our rectified camera is standard (X-right, Y-down, Z-forward):
            #    VADAS_Xc = Z_cam_rect (forward depth)
            #    VADAS_Yc = X_cam_rect (horizontal)
            #    VADAS_Zc = Y_cam_rect (vertical)

            Xc_vadas = depth_plane
            Yc_vadas = -((u_rect - cx_rect) / fx_rect * depth_plane)
            Zc_vadas = -(v_rect - cy_rect) / fy_rect * depth_plane

            u_fish, v_fish, valid_projection = vadas_model.project_point(Xc_vadas, Yc_vadas, Zc_vadas)

            if valid_projection and 0 <= u_fish < W_orig and 0 <= v_fish < H_orig:
                map_x[v_rect, u_rect] = u_fish
                map_y[v_rect, u_rect] = v_fish
            else:
                # Mark invalid pixels (e.g., outside original image bounds)
                map_x[v_rect, u_rect] = -1
                map_y[v_rect, u_rect] = -1
    
    return map_x, map_y
