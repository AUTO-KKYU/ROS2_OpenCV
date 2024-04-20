import cv2 as cv
from cv2 import aruco
import numpy as np

# dictionary to specify type of the marker
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)

# detect the marker
param_markers = aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(marker_dict, param_markers)

# utilizes default camera/webcam driver
cap = cv.VideoCapture(0)

# iterate through multiple frames, in a live video feed
while True:
    ret, frame = cap.read()
    if not ret:
        break
    # turning the frame to grayscale-only (for efficiency)
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Detect markers
    marker_corners, marker_IDs, _ = detector.detectMarkers(gray_frame)
    
    # Check if markers are detected
    if marker_corners:
        for ids, corners in zip(marker_IDs, marker_corners):
            # Draw marker outline
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )
            # Reshape corners
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            # Draw marker ID
            top_right = corners[0].ravel()
            cv.putText(
                frame,
                f"id: {ids[0]}",
                tuple(top_right),
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (200, 100, 0),
                2,
                cv.LINE_AA,
            )

    # Display frame
    cv.imshow("frame", frame)
    
    # Check for quit command
    key = cv.waitKey(1)
    if key == ord("q"):
        break

# Release video capture and close windows
cap.release()
cv.destroyAllWindows()
