# kinect_installation
Tested Azure Kinect DK on 
* arm64(Jetson Orin) Ubuntu20.04 
* arm64(Jetson docker) Ubuntu20.04
* arm64(Jetson Orin) Ubuntu22.04
* arm64(Jetson docker) Ubuntu22.04

## Step 1. Install Azure Kinect SDK
The Azure Kinect Sensor SDK officially supports only Ubuntu 18.04. However, we can install it on Ubuntu 20.04 by following these steps. [Reference here.](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1853#issuecomment-1684553505)
1. Download the `.deb` package.
    ``` 
    curl -sSL -O https://packages.microsoft.com/config/ubuntu/18.04/packages-microsoft-prod.deb  
    sudo dpkg -i packages-microsoft-prod.deb  
    rm packages-microsoft-prod.deb  
    ```
    
  2. Edit `/etc/apt/sources.list.d/microsoft-prod.list`. Change "[arch=amd64]" into "[arch=arm64]" and replace the repostory path with https://packages.microsoft.com/ubuntu/18.04/multiarch/prod.
     ```
     deb [arch=arm64] https://packages.microsoft.com/ubuntu/18.04/multiarch/prod bionic main  
     ```
     
  3. **(for Docker only)** Set EULA acceptance in non-interactive environment for installing libk4abt in Docker. [Reference here.](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1190#issuecomment-618473882)  

      * Set the environmental variable DEBIAN_FRONTEND to noninteractive. If you are in a Dockerfile you can do this with the ARG command.  
        ```
        ARG DEBIAN_FRONTEND=noninteractive
        ```
        This tells dpkg and debconf that you are in a non-interactive environment.
      
      * Manually set the hash of the accepted EULA in the debconf database.  
        ```
        echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | sudo debconf-set-selections  
        ```
        NOTE: That hash may change if the EULA ever changes.  
      
      * Manually set that the user has accepted the EULA.  
        ```
        echo 'libk4a1.4 libk4a1.4/accept-eula boolean true' | sudo debconf-set-selections  
        ```
        
  4. Install package 
     ```
     sudo apt update
     sudo apt install libk4a1.4 libk4a1.4-dev k4a-tools
     ```
     **P.S.** k4a-tools require depedency libsoundio1, which is not available on Ubuntu22.04. Get and install Ubuntu20.04 version of libsoundio1 manually. 
     ```
     wget http://ports.ubuntu.com/pool/universe/libs/libsoundio/libsoundio1_1.1.0-1_arm64.deb
     sudo dpkg -i libsoundio1_1.1.0-1_arm64.deb
     ```


  5. To use Azure Kinect SDK without being ‘root’.
     ```
     git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git  
     cd Azure-Kinect-Sensor-SDK  
     sudo cp scripts/99-k4a.rules /etc/udev/rules.d/  
     ```
  
  6. **(for Docker only)** Change the Docker run option to access Azure Kinect in docker. [Reference here.](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1258#issuecomment-1954543086)
     ```
     --gpus 'all,"capabilities=compute,utility,graphics"' # instead of --gpus all
     ```
     
  7. Open the SDK on terminal.
     ```
     k4aviewer
     ```

## Step 2. Install pyKinectAzure
pyKinectAzure library lets us to use the Kinect Azure DK with Python.
1. Install pykinect_azure with pip
  ```
  pip install pykinect_azure
  ```
2. Run example from pyKinectAzure/examples to test it out.
  ```
  git clone https://github.com/ibaiGorordo/pyKinectAzure.git
  cd pyKinectAzure/examples
  python exampleDepthImage.py
  ```
