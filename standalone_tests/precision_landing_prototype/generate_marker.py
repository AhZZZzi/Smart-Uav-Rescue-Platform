import cv2
import cv2.aruco as aruco
import numpy as np

# Choose the 4x4 dictionary, containing 50 markers (IDs from 0 to 49)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

marker_id = 0
marker_size = 400
border_size = 60  # white quiet zone thickness in pixels

marker_image = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

# Add a white border (quiet zone) around the marker
bordered = cv2.copyMakeBorder(
    marker_image,
    border_size, border_size, border_size, border_size,
    cv2.BORDER_CONSTANT,
    value=255  # white
)

filename = f"marker_id_{marker_id}.png"
cv2.imwrite(filename, bordered)
print(f"Generated marker ID {marker_id} with quiet zone, saved at: {filename}")
print(f"Final image size: {bordered.shape[0]}x{bordered.shape[1]} px")