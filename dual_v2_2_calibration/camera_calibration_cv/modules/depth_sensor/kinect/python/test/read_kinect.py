import os
import sys
from pathlib import Path
project_path = Path(Path(__file__).parent.absolute()).parent.absolute()
sys.path.insert(0, str(project_path))

import cv2
cv2.namedWindow('Images',cv2.WINDOW_NORMAL) #Have to write before importing matplotlib

import numpy as np
import matplotlib.pyplot as plt

# Add the depth_sensor directory to the sys.path list
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
import scripts.utils as utils
from scripts.kinect import Kinect


kinect = Kinect()
extrinsics=np.array(
    [[0,0,1,-0.73],
    [-1,0,0, 0.18],
    [0,-1,0, 0.12],
    [0,0,0,1]])

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# ax.view_init(elev=-70, azim=-95) #Realsense coordinate view
# ax.view_init(elev=10, azim=180) #Robot coordiante view
ax.view_init(elev=0, azim=180) #front view
fig.show()

delay_ms = 500
while True:
    kinect.update()
    depth_np = kinect.get_depth_image()
    rgb_np = kinect.get_color_image()
    colored_depth_np = kinect.get_colored_depth_image()
    pts = kinect.get_point_cloud()
    if depth_np is None or rgb_np is None or pts is None:
        continue
    
    pts1 = pts/1000.0
    pts2 = utils.transform_point_cloud(pts1, extrinsics)
    pts2 = pts2[pts2[:,0] > -0.5]   
    pts2 = pts2[pts2[:,0] < 0.5]     
    pts2 = pts2[pts2[:,1] > -0.25]   
    pts2 = pts2[pts2[:,1] < 0.3]  
    pts2 = pts2[pts2[:,2] < 0.5]
    pts2 = pts2[pts2[:,2] > -0.01]
    
    #Plot point cloud
    plt.cla()
    ax.set_title("Kinect", y=1.0, pad=-30)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.scatter(*[0,0,0], c='red', marker="X")
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    fig.tight_layout()
    ax.scatter(pts2[:,0], pts2[:,1], pts2[:,2], c='pink', alpha=0.5)
    plt.draw()
    plt.pause(0.02)

    #Show Image
    small_color_image = cv2.resize(rgb_np, (400, 340), interpolation = cv2.INTER_LINEAR)
    small_depth_colormap = cv2.resize(colored_depth_np, (400, 340), interpolation = cv2.INTER_LINEAR)
    images = np.hstack((small_color_image, small_depth_colormap))
    cv2.imshow("Images", images)
    key = cv2.waitKey(delay_ms)
# utils.generate_gif(fig, ax, "Kinect", pts2)
