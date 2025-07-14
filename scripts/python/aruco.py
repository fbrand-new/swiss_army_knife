import cv2
import cv2.aruco as aruco
import numpy as np

# Load the video
cap = cv2.VideoCapture('/home/fbrand/swiss_army_knife/data/static.mp4')

# Check if video opened successfully
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# Camera calibration parameters (replace with your camera's values)
# Camera calibration parameters are derived from calibration.json file produced by spectacular sdk
camera_matrix = np.array([[678.633, 0, 480],
                         [0, 678.633, 360],
                         [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4,1))  # Assuming no lens distortion


# Set up ArUco detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Marker length in meters
marker_length_id0 = 0.015  # 1.5cm
marker_length = 0.03 # 3cm
objPoints_id0 = np.array([[-marker_length_id0/2, marker_length_id0/2, 0],
                     [marker_length_id0/2, marker_length_id0/2, 0],
                     [marker_length_id0/2, -marker_length_id0/2, 0],
                     [-marker_length_id0/2, -marker_length_id0/2, 0]], dtype=np.float32)
objPoints = np.array([[-marker_length/2, marker_length/2, 0],
                     [marker_length/2, marker_length/2, 0],
                     [marker_length/2, -marker_length/2, 0],
                     [-marker_length/2, -marker_length/2, 0]], dtype=np.float32)

# Add these variables before the while loop
last_rvec = None
last_tvec = None
alpha = 0.5  # Smoothing factor

while cap.isOpened():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers using modern API
    corners, ids, rejected = detector.detectMarkers(gray)

    # If markers detected
    if ids is not None:
        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Prepare points for all markers at once
        all_corners = []
        all_obj_points = []
        
        for id, corner in zip(corners,ids):
            all_corners.extend(corner[0])

            if id == 0 :
                all_obj_points.extend([
                    [-marker_length_id0/2, marker_length_id0/2, 0],
                    [marker_length_id0/2, marker_length_id0/2, 0],
                    [marker_length_id0/2, -marker_length_id0/2, 0],
                    [-marker_length_id0/2, -marker_length_id0/2, 0]
                ])
            else:
                all_obj_points.extend([
                    [-marker_length_id0/2, marker_length_id0/2, 0],
                    [marker_length_id0/2, marker_length_id0/2, 0],
                    [marker_length_id0/2, -marker_length_id0/2, 0],
                    [-marker_length_id0/2, -marker_length_id0/2, 0]
                ])

        # Convert to numpy arrays
        all_corners = np.array(all_corners, dtype=np.float32)
        all_obj_points = np.array(all_obj_points, dtype=np.float32)

        # Solve PnP for all points at once
        retval, rvecs, tvecs, reprojectionError = cv2.solvePnPGeneric(
            all_obj_points, all_corners, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if retval:
            # Draw axes for each marker
            for i in range(len(ids)):
                rvec = rvecs[i]
                tvec = tvecs[i]
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, 
                                rvec, tvec, marker_length_id0)
                
                # Display pose information for marker ID 0
                if ids[i] == 0:
                    position = tvec
                    text = f"Pos (x,y,z): ({position[0][0]:.2f}, {position[1][0]:.2f}, {position[2][0]:.2f})"
                    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.7, (0, 255, 0), 2)

    # Display the resulting frame
    cv2.imshow('Frame', frame)

    # Press Q on keyboard to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the video capture object
cap.release()

# Closes all the frames
cv2.destroyAllWindows()

