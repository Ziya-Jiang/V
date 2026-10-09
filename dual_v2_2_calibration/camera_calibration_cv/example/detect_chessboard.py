import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from scripts.utils import load_config, show_image
from scripts.camera_manager import CameraManager
from scripts.chessboard import detect_chessboard

def main():
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config.yaml')
    config = load_config(config_path)  
    camera_manager = CameraManager(config["camera_type"], camera_id=config.get("camera_id"))
    cb_dimension = tuple(config.get("chessboard_dimension", [8, 6]))
    
    try:
        while True:
            camera_manager.update()
            rgb_image = camera_manager.get_color_image()
            
            if rgb_image is None:
                continue
                
            processed_image, success, corners = detect_chessboard(rgb_image, cb_dimension)
            
            key = show_image(processed_image, wait_time_ms=1)
            if key == 'q':
                print("[INFO] Exiting camera preview...")
                break
    finally:
        camera_manager.release()

if __name__ == "__main__":
    main()
