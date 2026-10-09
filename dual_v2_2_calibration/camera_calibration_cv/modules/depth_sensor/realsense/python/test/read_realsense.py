import sys
from pathlib import Path
project_path = Path(Path(__file__).parent.absolute()).parent.absolute()
sys.path.insert(0, str(project_path))

import cv2
import numpy as np
import matplotlib.pyplot as plt
import scripts.utils as utils
from scripts.realsense import Realsense

rs = Realsense(clipping_distance_mm=2000)
extrinsics=np.array(
    [[0,0,1,-0.73],
    [-1,0,0, 0.19],
    [0,-1,0, 0.12],
    [0,0,0,1]])
# intrinsics = rs.get_intrinsics_matrix()
intrinsics=np.array(
    [[409.2078,0,318.9116],
    [0,410.0721,235.1238],
    [0,0,1]])

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# ax.view_init(elev=-70, azim=-95) #Realsense coordinate view
# ax.view_init(elev=10, azim=180) #Robot coordiante view
ax.view_init(elev=0, azim=180) #front view
fig.show()

cv2.namedWindow("Images", cv2.WINDOW_NORMAL)
delay_ms = 500
while True:
    rs.update()
    depth_np = rs.get_depth_image()
    rgb_np   = rs.get_color_image()
    colored_depth_image = rs.get_colored_depth_image()
    if depth_np is None or colored_depth_image is None or rgb_np is None:
        continue
    pts = rs.get_point_cloud(intrinsics)

    pts1 = pts/1000.0
    pts2 = utils.transform_point_cloud(pts1, extrinsics)
    pts2 = pts2[pts2[:,0] > -0.7]   
    pts2 = pts2[pts2[:,0] < 0.3]     
    pts2 = pts2[pts2[:,1] > -0.5]   
    pts2 = pts2[pts2[:,1] < 0.43]  
    pts2 = pts2[pts2[:,2] > 0.01]
    
    #Plot point cloud
    plt.cla()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.scatter(*[0,0,0], c='red', marker="X")
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    fig.tight_layout()
    ax.scatter(pts2[:,0], pts2[:,1], pts2[:,2], c='pink', alpha=0.5)
    plt.draw()
#     plt.savefig(f"{n}.png", bbox_inches='tight', pad_inches=0.3)
    plt.pause(0.02)

    #Show Image
    small_color_image = cv2.resize(rgb_np, (0, 0), fx =0.6, fy =0.6)
    small_depth_colormap = cv2.resize(colored_depth_image, (0, 0), fx =0.6, fy =0.6)
    images = np.hstack((small_color_image, small_depth_colormap))
    cv2.imshow("Images", images)
    key = cv2.waitKey(delay_ms)
    if key == ord('q'):
        break
