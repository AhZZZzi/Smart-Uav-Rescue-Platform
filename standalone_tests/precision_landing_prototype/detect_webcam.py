import cv2
import cv2.aruco as aruco

# Use the same dictionary as when generating the marker
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector_params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, detector_params)

# Open the default webcam (0 = first camera found)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("ERROR: Cannot open webcam")
    exit()

print("Webcam opened. Press 'q' to quit.")

cv2.namedWindow("ArUco Detection", cv2.WINDOW_NORMAL)
cv2.resizeWindow("ArUco Detection", 960, 720)

while True:
    ret, frame = cap.read()
    if not ret:
        print("ERROR: Failed to read frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        for marker_id in ids.flatten():
            print(f"Detected marker ID: {marker_id}")

    cv2.imshow("ArUco Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()