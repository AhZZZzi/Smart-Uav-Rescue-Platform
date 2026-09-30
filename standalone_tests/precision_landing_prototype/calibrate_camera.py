import cv2
import numpy as np
import glob

CHESSBOARD_SIZE = (9, 6)
SQUARE_SIZE_MM = 29.0

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE_MM

all_objpoints = []
all_imgpoints = []
valid_filenames = []

images = glob.glob("calib_images/*.png")
print(f"Found {len(images)} images")

img_shape = None

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_shape = gray.shape[::-1]

    ret, corners = cv2.findChessboardCornersSB(gray, CHESSBOARD_SIZE, None)

    if ret:
        all_objpoints.append(objp)
        all_imgpoints.append(corners)
        valid_filenames.append(fname)
        print(f"{fname}: chessboard found")
    else:
        print(f"{fname}: NOT found, skipped")

if len(all_objpoints) < 10:
    print("WARNING: fewer than 10 valid images.")

# --- First pass calibration ---
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    all_objpoints, all_imgpoints, img_shape, None, None
)
print(f"\nFirst pass reprojection error: {ret:.4f}")

# --- Compute per-image error, remove worst outliers ---
per_image_errors = []
for i in range(len(all_objpoints)):
    imgpoints2, _ = cv2.projectPoints(all_objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
    imgpoints1 = all_imgpoints[i].reshape(-1, 2).astype(np.float32)
    imgpoints2 = imgpoints2.reshape(-1, 2).astype(np.float32)
    error = cv2.norm(imgpoints1, imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    per_image_errors.append(error)
    print(f"  {valid_filenames[i]}: error={error:.4f}")

mean_error = np.mean(per_image_errors)
keep_objpoints = []
keep_imgpoints = []
removed = 0
for i, err in enumerate(per_image_errors):
    if err <= 1.5 * mean_error:
        keep_objpoints.append(all_objpoints[i])
        keep_imgpoints.append(all_imgpoints[i])
    else:
        print(f"  Removing outlier: {valid_filenames[i]} (error={err:.4f})")
        removed += 1

print(f"\nRemoved {removed} outlier images. Recalibrating with {len(keep_objpoints)} images...")

# --- Second pass calibration (cleaned data) ---
ret2, camera_matrix2, dist_coeffs2, rvecs2, tvecs2 = cv2.calibrateCamera(
    keep_objpoints, keep_imgpoints, img_shape, None, None
)

print(f"\n--- Final Calibration Result ---")
print(f"Reprojection error: {ret2:.4f}")
print("Camera matrix:\n", camera_matrix2)
print("Distortion coefficients:\n", dist_coeffs2)

np.savez("camera_calibration.npz",
         camera_matrix=camera_matrix2,
         dist_coeffs=dist_coeffs2)
print("\nSaved to camera_calibration.npz")