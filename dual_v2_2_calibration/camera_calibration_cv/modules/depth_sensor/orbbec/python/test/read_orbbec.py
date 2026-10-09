import sys
from pathlib import Path
project_path = Path(Path(__file__).parent.absolute()).parent.absolute()
sys.path.insert(0, str(project_path))

import cv2
import numpy as np
import matplotlib.pyplot as plt

import scripts.utils as utils
from scripts.orbbec import Orbbec

MIN_DEPTH = 20  # 20mm
MAX_DEPTH = 10000  # 10000mm

extrinsics=np.array(
    [[0,0,1,-0.70],
    [-1,0,0, 0.165],
    [0,-1,0, 0.155],
    [0,0,0,1]])

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# ax.view_init(elev=-70, azim=-95) #Realsense coordinate view
# ax.view_init(elev=10, azim=180) #Robot coordiante view
ax.view_init(elev=0, azim=180) #front view
fig.show()

cv2.namedWindow('Images',cv2.WINDOW_NORMAL)
delay_ms = 500
with Orbbec() as orb:
    while True:
        try:
            orb.update()
            depth_image = orb.get_depth_image(min_depth=MIN_DEPTH, max_depth=MAX_DEPTH)
            colored_depth_image = orb.get_colored_depth_image(min_depth=MIN_DEPTH, max_depth=MAX_DEPTH)
            color_image = orb.get_color_image()
            pts = orb.get_point_cloud()
            if depth_image is None or colored_depth_image is None or color_image is None or pts is None:
                continue

            pts1 = pts.copy()/1000.0
            pts2 = utils.transform_point_cloud(pts1, extrinsics)
            pts2 = pts2[pts2[:,0] > -0.5]   
            pts2 = pts2[pts2[:,0] < 0.5]     
            pts2 = pts2[pts2[:,1] < 0.5]  

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
            plt.pause(0.02)

            #Show Image
            small_color_image = cv2.resize(color_image, (400, 340), interpolation = cv2.INTER_LINEAR)
            small_depth_colormap = cv2.resize(colored_depth_image, (400, 340), interpolation = cv2.INTER_LINEAR)
            images = np.hstack((small_color_image, small_depth_colormap))
            cv2.imshow("Images", images)
            key = cv2.waitKey(delay_ms)
            if key == ord('q'):
                break
        except Exception as e:
            if isinstance(e, KeyboardInterrupt):
                break
            print(e)
            raise e
        

