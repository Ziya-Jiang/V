import sys
import os
import numpy as np
import cv2

class CameraManager:
    def __init__(self, camera_type: str, camera_id: int = None):
        self.camera_type = camera_type
        self.camera = None
        self.color_image = None
        self.depth_image = None
        self.camera_id = camera_id
        
        if self.camera_type == "realsense":
            sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../modules/depth_sensor/realsense/python/scripts')))
            from realsense import Realsense
            self.camera = Realsense()
        elif self.camera_type == "usb_cam":
            self.cap = None
            if camera_id is not None:
                cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        self.cap = cap
                        print(f"[Info] Successfully opened usb_cam at specified index: {camera_id}")
                    else:
                        cap.release()
                        print(f"[Warning] Camera at index {camera_id} did not return a valid frame. Searching for alternative...")
                else:
                    print(f"[Warning] Could not open camera at index {camera_id}. Searching for alternative...")
            
            if self.cap is None:
                print(f"[Info] Scanning for available USB cameras...")
                available_cameras = []
                for i in range(10):
                    cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
                    if cap.isOpened():
                        ret, frame = cap.read()
                        if ret and frame is not None:
                            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                            fps = cap.get(cv2.CAP_PROP_FPS)
                            available_cameras.append((i, width, height, fps))
                            print(f"  Found camera at index {i}: {width}x{height} @ {fps:.1f}fps")
                            self.cap = cap  
                            print(f"[Info] Using camera at index {i}")
                            break
                        cap.release()
                
                if self.cap is None:
                    raise RuntimeError("Failed to open any usable usb_cam device. Please check device connections.")
        elif self.camera_type == "zed":
            sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../modules/depth_sensor/zed/python/scripts')))
            from zed import Zed
            self.camera = Zed()
        elif self.camera_type == "kinect":
            sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../modules/depth_sensor/kinect/python/scripts')))
            from kinect import Kinect
            self.camera = Kinect()
        elif self.camera_type == "orbbec":
            depth_sensor_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../modules/depth_sensor'))
            orbbec_python = os.path.join(depth_sensor_root, 'orbbec/python')
            sys.path.append(depth_sensor_root)
            sys.path.append(orbbec_python)
            from scripts.orbbec import Orbbec
            self.camera = Orbbec()
            self.camera.__enter__()
        else:
            raise NotImplementedError(f"Camera type {self.camera_type} is not supported.")

    def update(self):
        # Update the camera frame and store the image in the class
        if self.camera_type in ["realsense", "zed", "kinect", "orbbec"]:
            self.camera.update()
            self.color_image = self.camera.get_color_image()
            self.depth_image = self.camera.get_depth_image()
        elif self.camera_type == "usb_cam":
            ret, frame = self.cap.read()
            if ret:
                self.color_image = frame
                self.depth_image = None

    def get_depth_image(self):
        # Get the depth image from the class
        return self.depth_image

    def get_color_image(self):
        # Get the color image from the class
        return self.color_image

    def release(self):
        if self.camera_type == "usb_cam":
            self.cap.release()
        elif self.camera_type == "orbbec" and self.camera is not None:
            self.camera.release()
        elif self.camera_type in ["realsense", "zed", "kinect"]:
            if hasattr(self.camera, 'release'):
                self.camera.release()
            elif hasattr(self.camera, 'pipeline'):  # fallback for realsense
                self.camera.pipeline.stop()
            elif hasattr(self.camera, 'cam'):  # fallback for zed
                self.camera.cam.close()
            elif hasattr(self.camera, 'device'):  # fallback for kinect
                self.camera.device.close()

