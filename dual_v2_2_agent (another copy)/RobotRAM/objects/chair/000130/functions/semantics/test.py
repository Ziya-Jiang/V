import os
import numpy as np
import _pickle as cPickle
import open3d as o3d

with open('/research/dept8/kaichen/ObjectRAM/objects/motorcycle/000127/functions/semantics/000127_semantic.pkl', 'rb') as f:
    semantics = cPickle.load(f)
    f.close()

for k in semantics.keys():
    # print(k)
    # print(semantics[k].shape)
    pts = o3d.geometry.PointCloud()
    pts.points = o3d.utility.Vector3dVector(semantics[k])
    o3d.io.write_point_cloud(k + '.ply', pts)