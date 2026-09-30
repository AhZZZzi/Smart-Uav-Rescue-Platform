import cv2
import numpy as np
import glob

CHESSBOARD_SIZE = (9, 6)
SQUARE_SIZE_MM = 25.0  # placeholder, không quan trọng cho bước test pipeline

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE_MM

objpoints = []
imgpoints = []

images = glob.glob("calib_test_images/*.jpg")
print(f"Found {len(images)} images")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        print(f"{fname}: chessboard found")
    else:
        print(f"{fname}: NOT found, skipped")

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("\n--- Test Calibration Result ---")
print("Reprojection error:", ret)
print("Camera matrix:\n", camera_matrix)
print("Distortion coefficients:\n", dist_coeffs)