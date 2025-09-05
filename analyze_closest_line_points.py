import os
import json
import numpy as np
from sklearn.linear_model import RANSACRegressor, LinearRegression
from sklearn.preprocessing import PolynomialFeatures # Added for polynomial features
from sklearn.pipeline import make_pipeline # Added for pipeline

def analyze_closest_line_points(data_dir):
    all_x = []
    all_y = []
    all_z = []
    
    json_files = [f for f in os.listdir(data_dir) if f.endswith('.json')]
    
    if not json_files:
        print(f"Error: No JSON files found in {data_dir}")
        return None, None

    print(f"Found {len(json_files)} JSON files in {data_dir}")

    for filename in json_files:
        filepath = os.path.join(data_dir, filename)
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
                all_x.extend(data.get('x', []))
                all_y.extend(data.get('y', []))
                all_z.extend(data.get('z', []))
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON from {filename}: {e}")
        except Exception as e:
            print(f"Error reading {filename}: {e}")

    if not all_x or not all_y or not all_z:
        print("No valid point data was loaded.")
        return None, None

    points_x = np.array(all_x)
    points_y = np.array(all_y)
    points_z = np.array(all_z)

    print("\n--- Initial Data Analysis Results ---")
    print(f"Total number of points loaded: {len(points_x)}")
    print("\n--- X-coordinates (Initial) ---")
    print(f"  Mean: {np.mean(points_x):.4f}")
    print(f"  Min:  {np.min(points_x):.4f}")
    print(f"  Max:  {np.max(points_x):.4f}")
    print(f"  Std Dev: {np.std(points_x):.4f}")

    print("\n--- Y-coordinates (Initial) ---")
    print(f"  Mean: {np.mean(points_y):.4f}")
    print(f"  Min:  {np.min(points_y):.4f}")
    print(f"  Max:  {np.max(points_y):.4f}")
    print(f"  Std Dev: {np.std(points_y):.4f}")

    print("\n--- Z-coordinates (Initial) ---")
    print(f"  Mean: {np.mean(points_z):.4f}")
    print(f"  Min:  {np.min(points_z):.4f}")
    print(f"  Max:  {np.max(points_z):.4f}")
    print(f"  Std Dev: {np.std(points_z):.4f}")

    # Fallback: Simple Z-score based outlier removal for X and Y coordinates
    print("\n--- Performing Z-score based Outlier Removal for X and Y coordinates ---")
    
    # Calculate Z-scores for X and Y
    mean_x, std_x = np.mean(points_x), np.std(points_x)
    mean_y, std_y = np.mean(points_y), np.std(points_y)

    z_scores_x = np.abs((points_x - mean_x) / std_x)
    z_scores_y = np.abs((points_y - mean_y) / std_y)

    # Define a Z-score threshold (e.g., 3 standard deviations)
    z_threshold = 3.0

    # Identify inliers based on Z-scores
    inlier_indices = np.where((z_scores_x < z_threshold) & (z_scores_y < z_threshold))[0]

    inlier_points_np = np.vstack((points_x[inlier_indices], points_y[inlier_indices], points_z[inlier_indices])).T
    outlier_points_np = np.delete(np.vstack((points_x, points_y, points_z)).T, inlier_indices, axis=0)

    if len(inlier_points_np) == 0:
        print("No inlier points remaining after outlier removal.")
        return None, None

    print(f"Total points after Z-score outlier removal: {len(inlier_points_np)}")
    print(f"Number of outliers removed by Z-score: {len(outlier_points_np)}")

    # Re-analyze statistics for inlier points
    inlier_x = inlier_points_np[:, 0]
    inlier_y = inlier_points_np[:, 1]
    inlier_z = inlier_points_np[:, 2]

    print("\n--- Data Analysis Results (After Z-score Outlier Removal) ---")
    print(f"Total number of inlier points: {len(inlier_x)}")
    print("\n--- X-coordinates (Inliers) ---")
    print(f"  Mean: {np.mean(inlier_x):.4f}")
    print(f"  Min:  {np.min(inlier_x):.4f}")
    print(f"  Max:  {np.max(inlier_x):.4f}")
    print(f"  Std Dev: {np.std(inlier_x):.4f}")

    print("\n--- Y-coordinates (Inliers) ---")
    print(f"  Mean: {np.mean(inlier_y):.4f}")
    print(f"  Min:  {np.min(inlier_y):.4f}")
    print(f"  Max:  {np.max(inlier_y):.4f}")
    print(f"  Std Dev: {np.std(inlier_y):.4f}")

    print("\n--- Z-coordinates (Inliers) ---")
    print(f"  Mean: {np.mean(inlier_z):.4f}")
    print(f"  Min:  {np.min(inlier_z):.4f}")
    print(f"  Max:  {np.max(inlier_z):.4f}")
    print(f"  Std Dev: {np.std(inlier_z):.4f}")

    print("\n--- Performing RANSAC 2nd Degree Polynomial Fitting (Y vs X) ---")
    # Reshape X data for scikit-learn
    X = inlier_x.reshape(-1, 1)
    y = inlier_y

    # Create a pipeline for 2nd degree polynomial features and linear regression
    polynomial_model = make_pipeline(PolynomialFeatures(degree=2), LinearRegression())

    # Create a RANSAC regressor with the polynomial model
    ransac = RANSACRegressor(polynomial_model,
                             min_samples=3, # Minimum 3 samples for 2nd degree polynomial
                             residual_threshold=0.1, # Maximum residual error for a data point to be considered an inlier
                             random_state=42)
    ransac.fit(X, y)

    # Get the inlier mask and the fitted polynomial model
    inlier_mask = ransac.inlier_mask_
    fitted_polynomial = ransac.estimator_

    # Extract coefficients from the LinearRegression step of the pipeline
    # PolynomialFeatures(degree=2) generates features [1, x, x^2]
    # So, the linear regression model is y = poly_intercept * 1 + poly_coefs[0] * x + poly_coefs[1] * x^2
    # Therefore, a = poly_coefs[1], b = poly_coefs[0], c = poly_intercept
    poly_coefs = fitted_polynomial.named_steps['linearregression'].coef_
    poly_intercept = fitted_polynomial.named_steps['linearregression'].intercept_

    a = poly_coefs[1]
    b = poly_coefs[0]
    c = poly_intercept

    print(f"RANSAC Fitted 2nd Degree Polynomial: y = {a:.4f}x^2 + {b:.4f}x + {c:.4f}")
    print(f"  Coefficients (a, b, c): [{a}, {b}, {c}]")
    print(f"  Number of RANSAC inliers: {np.sum(inlier_mask)}")

    # Calculate the mean Z-coordinate of the RANSAC inliers
    mean_inlier_z = np.mean(inlier_points_np[inlier_mask][:, 2])
    print(f"  Mean Z-coordinate of RANSAC inliers: {mean_inlier_z:.4f}")

    # Optionally, you could return the consolidated points for further processing
    return inlier_points_np[inlier_mask], [a, b, c], mean_inlier_z

if __name__ == "__main__":
    # Adjust this path to your specific data directory
    data_directory = r"C:\Users\seok436\Documents\VSCode\Projects\point-cloud-creation\point-cloud-creation\ncdb-cls-sample\synced_data\closest_line_points"
    
    # Ensure the directory exists
    if not os.path.isdir(data_directory):
        print(f"Error: Data directory not found at {data_directory}")
    else:
        ransac_inlier_points, fitted_coefficients, mean_z = analyze_closest_line_points(data_directory)
        if ransac_inlier_points is not None and fitted_coefficients is not None:
            print("\nData analysis, Z-score outlier removal, and RANSAC 2nd degree polynomial fitting complete.")
            print("'ransac_inlier_points' contains the (x,y,z) data after RANSAC inlier selection.")
            print("'fitted_coefficients' contains the [a, b, c] for y = ax^2 + bx + c.")
            print(f"'mean_z' is the average Z-coordinate of the RANSAC inliers: {mean_z:.4f}")
            # Example: print(ransac_inlier_points[:5]) # Print first 5 RANSAC inlier points
            # Example: print(fitted_coefficients)

