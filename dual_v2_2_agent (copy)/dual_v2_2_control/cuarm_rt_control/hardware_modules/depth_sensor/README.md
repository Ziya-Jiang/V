# depth_sensor
Api for various depth sensors. 
Tested on 
* arm64(Jetson Orin) Ubuntu20.04
* arm64(Jetson docker) Ubuntu20.04
* arm64(Jetson Orin) Ubuntu22.04 (require matplotlib==3.9.0)


## Setup for Orbbec
Follow instructions on [pyorbbecsdk](https://github.com/orbbec/pyorbbecsdk.git)
* If you encounter problem with `sudo udevadm control --reload-rules && sudo udevadm trigger`, make sure udev service is running.
  ```bash
  sudo service udev status #Check if udev service is running
  sudo service udev restart #Restart udev service if it is not running
  ```

## Setup for Kinect
Follow instructions on [install_kinect.md](https://github.com/CURI-Simulation-and-Software-Group/depth_sensor/edit/main/install_kinect.md)

## Setup for Realsense
Get python library with `pip install pyrealsense2`
