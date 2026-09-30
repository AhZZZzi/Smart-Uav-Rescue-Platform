import cv2
import os

output_dir = "calib_images"
os.makedirs(output_dir, exist_ok=True)

cap = cv2.VideoCapture(0)
cv2.namedWindow("Calibration Capture", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Calibration Capture", 960, 720)

print("Press SPACE to capture an image, 'q' to finish.")
img_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    display = frame.copy()
    cv2.putText(display, f"Captured: {img_count}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Calibration Capture", display)

    key = cv2.waitKey(1) & 0xFF
    if key == ord(' '):
        filename = os.path.join(output_dir, f"calib_{img_count:02d}.png")
        cv2.imwrite(filename, frame)
        print(f"Saved: {filename}")
        img_count += 1
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"Done. Total images captured: {img_count}")