"""Native color profile selection and resize-consistent camera intrinsics."""
import numpy as np


def highest_color_profile(profiles,formats):
    choices=[profiles[i].as_video_stream_profile() for i in range(len(profiles))]
    maximum=max(p.get_width()*p.get_height() for p in choices)
    usable=[p for p in choices if p.get_width()*p.get_height()==maximum and p.get_format() in formats]
    if not usable:raise ValueError('Maximum head color resolution has no supported decoder')
    return max(usable,key=lambda p:(p.get_fps()<=30,p.get_fps() if p.get_fps()<=30 else -p.get_fps(),p.get_format()==formats[0]))


def scaled_intrinsics(k,source_wh,target_wh):
    sx,sy=np.array(target_wh,dtype=float)/source_wh
    # cv2.resize maps pixel centers, not the outer pixel corners.
    transform=np.array([[sx,0,(sx-1)/2],[0,sy,(sy-1)/2],[0,0,1]])
    return transform@np.asarray(k,float)
