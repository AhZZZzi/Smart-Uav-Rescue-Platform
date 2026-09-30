import cv2
import cv2.aruco as aruco
import numpy as np

# Load camera calibration (temporary for now)
calib_data = np.load("camera_calibration.npz")
camera_matrix = calib_data["camera_matrix"]
dist_coeffs = calib_data["dist_coeffs"]

MARKER_SIZE_M = 0.155  # <-- real marker edge length in meters, e.g. 10 cm. Adjust to your printed marker.

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector_params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, detector_params)

# 3D coordinates of marker corners in its own coordinate system
half_size = MARKER_SIZE_M / 2
marker_points_3d = np.array([
    [-half_size,  half_size, 0],
    [ half_size,  half_size, 0],
    [ half_size, -half_size, 0],
    [-half_size, -half_size, 0]
], dtype=np.float32)

cap = cv2.VideoCapture(0)
cv2.namedWindow("Pose Estimation", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Pose Estimation", 960, 720)

print("Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        for i, marker_id in enumerate(ids.flatten()):
            success, rvec, tvec = cv2.solvePnP(
                marker_points_3d, corners[i], camera_matrix, dist_coeffs
            )

            if success:
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, MARKER_SIZE_M * 0.5)

                x, y, z = tvec.flatten()
                distance = np.linalg.norm(tvec)

                text = f"ID {marker_id}: X={x:.2f} Y={y:.2f} Z={z:.2f} Dist={distance:.2f}m"
                cv2.putText(frame, text, (10, 30 + i * 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                print(text)

    cv2.imshow("Pose Estimation", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()