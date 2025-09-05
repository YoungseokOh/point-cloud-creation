import os
import numpy as np
from typing import List, Optional, Tuple

# Open3D for PCD I/O
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except Exception as e:
    OPEN3D_AVAILABLE = False
    print(f"Warning: Open3D not available ({e}). This script requires Open3D.")

# Local analysis module to obtain curve coefficients from closest_line_points
try:
    import analyze_closest_line_points as aclp
    ANALYSIS_AVAILABLE = True
except Exception as e:
    ANALYSIS_AVAILABLE = False
    print(f"Warning: analyze_closest_line_points not importable ({e}). Ensure it exists and dependencies are installed.")

# Paths
BASE_SYNC_DIR = os.path.join("ncdb-cls-sample", "synced_data")
ORIGINAL_PCD_DIR = os.path.join(BASE_SYNC_DIR, "pcd")
OUTPUT_PCD_DIR = os.path.join(BASE_SYNC_DIR, "closest_line_pcd")
CLOSEST_LINE_POINTS_DIR = os.path.join(BASE_SYNC_DIR, "closest_line_points")

# Curve generation config (x-forward, y-left, z-up)
X_START = 0.0
X_END = 1.0
NUM_SAMPLES_PER_METER = 200  # number of points along 1m

# Cached global curve model
GLOBAL_POLY_COEFFS: Optional[Tuple[float, float, float]] = None  # a, b, c for y=ax^2+bx+c
GLOBAL_MEAN_Z: Optional[float] = None


def load_original_pcd_points(frame_id: str) -> Optional[np.ndarray]:
    pcd_path = os.path.join(ORIGINAL_PCD_DIR, f"{frame_id}.pcd")
    if not os.path.isfile(pcd_path):
        print(f"skip: original PCD not found for frame {frame_id}: {pcd_path}")
        return None
    if not OPEN3D_AVAILABLE:
        print("skip: Open3D is required to read original PCDs")
        return None
    try:
        pcd = o3d.io.read_point_cloud(pcd_path)
        if not pcd.has_points():
            print(f"skip: empty PCD for frame {frame_id}")
            return None
        return np.asarray(pcd.points)
    except Exception as e:
        print(f"Failed to read PCD {pcd_path}: {e}")
        return None


def get_global_curve_model() -> bool:
    """Compute/load global curve coefficients (y=ax^2+bx+c) and mean_z from closest_line_points.
    Uses analyze_closest_line_points.analyze_closest_line_points on the directory.
    """
    global GLOBAL_POLY_COEFFS, GLOBAL_MEAN_Z
    if GLOBAL_POLY_COEFFS is not None and GLOBAL_MEAN_Z is not None:
        return True
    if not ANALYSIS_AVAILABLE:
        print("skip: analysis module not available")
        return False
    if not os.path.isdir(CLOSEST_LINE_POINTS_DIR):
        print(f"skip: closest_line_points dir not found: {CLOSEST_LINE_POINTS_DIR}")
        return False
    try:
        inlier_pts, coeffs, mean_z = aclp.analyze_closest_line_points(CLOSEST_LINE_POINTS_DIR)
        if coeffs is None or mean_z is None:
            print("skip: analysis did not return coefficients/mean_z")
            return False
        a, b, c = coeffs
        GLOBAL_POLY_COEFFS = (float(a), float(b), float(c))
        GLOBAL_MEAN_Z = float(mean_z)
        print(f"[MODEL] y = {a:.6f}x^2 + {b:.6f}x + {c:.6f}, mean_z={GLOBAL_MEAN_Z:.4f}")
        return True
    except Exception as e:
        print(f"Analysis failed: {e}")
        return False


def generate_curve_line_points(a: float, b: float, c: float,
                               x_start: float, x_end: float, num_samples: int,
                               z_value: float) -> np.ndarray:
    """Generate 3D points along the fitted curve in XY with constant Z."""
    xs = np.linspace(x_start, x_end, num_samples, dtype=float)
    ys = a * xs**2 + b * xs + c
    zs = np.full_like(xs, z_value, dtype=float)
    return np.stack([xs, ys, zs], axis=1)


def save_combined_points_as_pcd(original_points: np.ndarray, gen_points: np.ndarray, out_path: str) -> bool:
    if not OPEN3D_AVAILABLE:
        print("Open3D not available for saving PCD")
        return False
    try:
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        pts = np.vstack([original_points.astype(np.float64), gen_points.astype(np.float64)])
        # Color original gray, generated red
        orig_colors = np.tile(np.array([[0.6, 0.6, 0.6]], dtype=np.float64), (original_points.shape[0], 1))
        gen_colors = np.tile(np.array([[1.0, 0.0, 0.0]], dtype=np.float64), (gen_points.shape[0], 1))
        colors = np.vstack([orig_colors, gen_colors])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        try:
            ok = o3d.io.write_point_cloud(out_path, pcd, write_ascii=True)
        except TypeError:
            ok = o3d.io.write_point_cloud(out_path, pcd)
        if not ok:
            print(f"Failed to write PCD to {out_path}")
        return bool(ok)
    except Exception as e:
        print(f"Error writing combined PCD {out_path}: {e}")
        return False


def generate_1m_line_pcd_for_frame(frame_id: str) -> Optional[str]:
    # Ensure model is available
    if not get_global_curve_model():
        return None
    assert GLOBAL_POLY_COEFFS is not None and GLOBAL_MEAN_Z is not None
    a, b, c = GLOBAL_POLY_COEFFS

    # Load original points
    orig = load_original_pcd_points(frame_id)
    if orig is None or orig.size == 0:
        return None

    # Generate 1m segment along x-forward
    gen_pts = generate_curve_line_points(a, b, c, X_START, X_END, NUM_SAMPLES_PER_METER, GLOBAL_MEAN_Z)

    # Save combined for visualization
    out_path = os.path.join(OUTPUT_PCD_DIR, f"{frame_id}.pcd")
    if save_combined_points_as_pcd(orig, gen_pts, out_path):
        return out_path
    return None


def main(frames: Optional[List[str]] = None) -> None:
    if frames is None:
        if not os.path.isdir(ORIGINAL_PCD_DIR):
            print(f"Original PCD dir not found: {ORIGINAL_PCD_DIR}")
            return
        frames = [os.path.splitext(f)[0] for f in os.listdir(ORIGINAL_PCD_DIR) if f.lower().endswith(".pcd")]
        frames.sort()

    os.makedirs(OUTPUT_PCD_DIR, exist_ok=True)

    print(f"Generating 1m curve-aligned PCDs into: {OUTPUT_PCD_DIR}")

    # Pre-compute model once
    if not get_global_curve_model():
        print("Aborting: could not obtain curve model from closest_line_points.")
        return

    done = 0
    for fid in frames:
        out = generate_1m_line_pcd_for_frame(fid)
        if out:
            done += 1
            print(f"ok: {fid} -> {out}")
        else:
            print(f"skip: {fid}")

    print(f"Completed. Generated {done} files.")


if __name__ == "__main__":
    # 1m test for a single frame
    sample_frame = "0000000931"
    main([sample_frame])
