import cv2
import numpy as np

def detect_chessboard(image, chessboard_dimension=(8, 6)):
    """
    return chessboard image with corners drawed and quality label,
    along with detection success flag and refined corner coordinates.
    """
    drawn_image = image.copy()
    gray = cv2.cvtColor(drawn_image, cv2.COLOR_BGR2GRAY)
    
    # Find the chess board corners with FAST_CHECK to avoid severe lag when not found
    ret, corners = cv2.findChessboardCorners(
        gray, 
        tuple(chessboard_dimension), 
        None, 
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    
    if ret:
        # Refine corner locations
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        # Draw the corners
        cv2.drawChessboardCorners(drawn_image, tuple(chessboard_dimension), corners_refined, ret)
        
        # Draw quality label
        cv2.putText(drawn_image, "Quality: GOOD", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return drawn_image, True, corners_refined
    else:
        # Draw quality label
        cv2.putText(drawn_image, "Quality: BAD (Not found)", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return drawn_image, False, None
