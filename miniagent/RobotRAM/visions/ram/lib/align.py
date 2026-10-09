"""
    RANSAC for Similarity Transformation Estimation
    Modified from https://github.com/hughw19/NOCS_CVPR2019
    Originally Written by Srinath Sridhar
"""
import time
import numpy as np
import cv2
import _pickle as cPickle
from scipy.spatial import cKDTree
from sklearn.neighbors import NearestNeighbors
import open3d as o3d

import torch
import torch.nn as nn
from torch.autograd import Variable
# from lib.loss import NOCSLoss

class DistanceRatio(nn.Module):
    def __init__(self, corr_wt):
        super(DistanceRatio, self).__init__()
        self.threshold = 0.1
        self.corr_wt = corr_wt
    
    def forward(self, pts1, pts2):
        pairwise_dis1 = pts1.unsqueeze(-1) - pts1.unsqueeze(-1).permute(2,1,0)
        pairwise_dis1 = torch.sum(torch.norm(pairwise_dis1, dim=1), dim=-1)

        pairwise_dis2 = pts2.unsqueeze(-1) - pts2.unsqueeze(-1).permute(2,1,0)
        pairwise_dis2 = torch.sum(torch.norm(pairwise_dis2, dim=1), dim=-1)

        idx = (pairwise_dis2 < torch.median(pairwise_dis2)).nonzero().flatten()

        valid_pairwise_dis1 = torch.gather(pairwise_dis1, dim=0, index=idx)
        valid_pairwise_dis2 = torch.gather(pairwise_dis2, dim=0, index=idx)

        # perturm_idx = torch.randperm(pairwise_dis1.shape[0])
        # pairwise_dis1_2 = pairwise_dis1[perturm_idx]
        # pairwise_dis2_2 = pairwise_dis2[perturm_idx]

        # ratio1 = pairwise_dis1 / pairwise_dis1_2
        # ratio2 = pairwise_dis2 / pairwise_dis2_2

        # ratio_loss = nn.L1Loss()(ratio1, ratio2) * self.corr_wt

        perturm_idx = torch.randperm(valid_pairwise_dis1.shape[0])
        valid_pairwise_dis1_2 = valid_pairwise_dis1[perturm_idx]
        valid_pairwise_dis2_2 = valid_pairwise_dis2[perturm_idx]

        ratio1 = valid_pairwise_dis1 / valid_pairwise_dis1_2
        ratio2 = valid_pairwise_dis2 / valid_pairwise_dis2_2

        ratio_loss = nn.L1Loss()(ratio1, ratio2) * self.corr_wt

        return ratio_loss

class optimizeDeformation(nn.Module):
    def __init__(self):
        super(optimizeDeformation, self).__init__()
        self.criterion = DistanceRatio(0.1)  
        self.iter_num = 10      
    
    def forward(self, source, target):
        deform = Variable(torch.zeros(source.shape).to(source.device), requires_grad = True)
        optimizer = torch.optim.Adam([deform], lr=0.002)

        loss_list = []
        for i in range(self.iter_num):
            deformed_nocs = target + deform
            pts_loss = self.criterion(deformed_nocs, source)

            loss_list.append(pts_loss.item())

            optimizer.zero_grad()
            pts_loss.backward(retain_graph=True)
            optimizer.step()
        
        return deform, loss_list

def estimateSimilarityUmeyama(SourceHom, TargetHom):
    # Copy of original paper is at: http://web.stanford.edu/class/cs273/refs/umeyama.pdf
    SourceCentroid = np.mean(SourceHom[:3, :], axis=1)
    TargetCentroid = np.mean(TargetHom[:3, :], axis=1)
    nPoints = SourceHom.shape[1]
    CenteredSource = SourceHom[:3, :] - np.tile(SourceCentroid, (nPoints, 1)).transpose()
    CenteredTarget = TargetHom[:3, :] - np.tile(TargetCentroid, (nPoints, 1)).transpose()
    CovMatrix = np.matmul(CenteredTarget, np.transpose(CenteredSource)) / nPoints
    if np.isnan(CovMatrix).any():
        print('nPoints:', nPoints)
        print(SourceHom.shape)
        print(TargetHom.shape)
        raise RuntimeError('There are NANs in the input.')

    U, D, Vh = np.linalg.svd(CovMatrix, full_matrices=True)
    d = (np.linalg.det(U) * np.linalg.det(Vh)) < 0.0
    if d:
        D[-1] = -D[-1]
        U[:, -1] = -U[:, -1]
    # rotation
    Rotation = np.matmul(U, Vh)
    # scale
    varP = np.var(SourceHom[:3, :], axis=1).sum()
    Scale = 1 / varP * np.sum(D)
    # translation
    Translation = TargetHom[:3, :].mean(axis=1) - SourceHom[:3, :].mean(axis=1).dot(Scale*Rotation.T)
    # transformation matrix
    OutTransform = np.identity(4)
    OutTransform[:3, :3] = Scale * Rotation
    OutTransform[:3, 3] = Translation

    return Scale, Rotation, Translation, OutTransform

# def estimateSimilarityUmeyama(SourceHom, TargetHom, est_scale=True):
#     SourceCentroid = np.mean(SourceHom[:3, :], axis=1)
#     TargetCentroid = np.mean(TargetHom[:3, :], axis=1)
#     nPoints = SourceHom.shape[1]

#     CenteredSource = SourceHom[:3, :] - np.tile(SourceCentroid, (nPoints, 1)).transpose()  #(3,N)
#     CenteredTarget = TargetHom[:3, :] - np.tile(TargetCentroid, (nPoints, 1)).transpose()

#     CovMatrix = np.matmul(CenteredTarget, np.transpose(CenteredSource)) / nPoints

#     if np.isnan(CovMatrix).any():
#         raise RuntimeError('There are NANs in the input. nPoints={}'.format(nPoints))

#     U, D, Vh = np.linalg.svd(CovMatrix, full_matrices=True)
#     d = (np.linalg.det(U) * np.linalg.det(Vh)) < 0.0
#     if d:
#         D[-1] = -D[-1]
#         U[:, -1] = -U[:, -1]

#     Rotation = np.matmul(U, Vh).T # Transpose is the one that works

#     ScaleFact = np.eye(3)
#     if est_scale:
#         varP = np.var(SourceHom[:3, :], axis=1).sum()
#         ScaleFact = 1/varP * np.sum(D) # scale factor
#         ScaleFact = np.diag([ScaleFact,ScaleFact,ScaleFact])

#     Translation = TargetHom[:3, :].mean(axis=1) - SourceHom[:3, :].mean(axis=1).dot(ScaleFact@Rotation)

#     Rotation = Rotation.T
#     OutTransform = np.identity(4)
#     OutTransform[:3, :3] = ScaleFact@Rotation
#     OutTransform[:3, 3] = Translation
#     return ScaleFact, Rotation, Translation, OutTransform

def estimateSimilarityTransform(source: np.array, target: np.array, verbose=False):
    """ Add RANSAC algorithm to account for outliers.

    """
    assert source.shape[0] == target.shape[0], 'Source and Target must have same number of points.'
    SourceHom = np.transpose(np.hstack([source, np.ones([source.shape[0], 1])]))
    TargetHom = np.transpose(np.hstack([target, np.ones([target.shape[0], 1])]))
    # Auto-parameter selection based on source heuristics
    # Assume source is object model or gt nocs map, which is of high quality
    SourceCentroid = np.mean(SourceHom[:3, :], axis=1)
    nPoints = SourceHom.shape[1]
    CenteredSource = SourceHom[:3, :] - np.tile(SourceCentroid, (nPoints, 1)).transpose()
    SourceDiameter = 2 * np.amax(np.linalg.norm(CenteredSource, axis=0))
    InlierT = SourceDiameter / 10.0  # 0.1 of source diameter
    maxIter = 128
    # maxIter = 1000
    confidence = 0.99

    if verbose:
        print('Inlier threshold: ', InlierT)
        print('Max number of iterations: ', maxIter)

    BestInlierRatio = 0
    BestInlierIdx = np.arange(nPoints)
    for i in range(0, maxIter):
        # Pick 5 random (but corresponding) points from source and target
        RandIdx = np.random.randint(nPoints, size=5)
        Scale, _, _, OutTransform = estimateSimilarityUmeyama(SourceHom[:, RandIdx], TargetHom[:, RandIdx])
        PassThreshold = Scale * InlierT    # propagate inlier threshold to target scale
        Diff = TargetHom - np.matmul(OutTransform, SourceHom)
        ResidualVec = np.linalg.norm(Diff[:3, :], axis=0)
        InlierIdx = np.where(ResidualVec < PassThreshold)[0]
        nInliers = InlierIdx.shape[0]
        InlierRatio = nInliers / nPoints
        # update best hypothesis
        if InlierRatio > BestInlierRatio:
            BestInlierRatio = InlierRatio
            BestInlierIdx = InlierIdx
        if verbose:
            print('Iteration: ', i)
            print('Inlier ratio: ', BestInlierRatio)
        # early break
        if (1 - (1 - BestInlierRatio ** 5) ** i) > confidence:
            break

    if(BestInlierRatio < 0.1):
        print('[ WARN ] - Something is wrong. Small BestInlierRatio: ', BestInlierRatio)
        return None, None, None, None, None, None

    SourceInliersHom = SourceHom[:, BestInlierIdx]
    TargetInliersHom = TargetHom[:, BestInlierIdx]
    Scale, Rotation, Translation, OutTransform = estimateSimilarityUmeyama(SourceInliersHom, TargetInliersHom)

    if verbose:
        print('BestInlierRatio:', BestInlierRatio)
        print('Rotation:\n', Rotation)
        print('Translation:\n', Translation)
        print('Scale:', Scale)

    return Scale, Rotation, Translation, OutTransform, BestInlierRatio, BestInlierIdx


def backproject(depth, intrinsics, instance_mask):
    """ Back-projection, use opencv camera coordinate frame.

    """
    cam_fx = intrinsics[0, 0]
    cam_fy = intrinsics[1, 1]
    cam_cx = intrinsics[0, 2]
    cam_cy = intrinsics[1, 2]

    non_zero_mask = (depth > 0)
    final_instance_mask = np.logical_and(instance_mask, non_zero_mask)
    idxs = np.where(final_instance_mask)

    z = depth[idxs[0], idxs[1]]
    x = (idxs[1] - cam_cx) * z / cam_fx
    y = (idxs[0] - cam_cy) * z / cam_fy
    pts = np.stack((x, y, z), axis=1)

    return pts, idxs


def align_nocs_to_depth(masks, coords, depth, intrinsics, instance_ids, img_path, verbose=False):
    num_instances = len(instance_ids)
    error_messages = ''
    elapses = []
    scales = np.zeros(num_instances)
    rotations = np.zeros((num_instances, 3, 3))
    translations = np.zeros((num_instances, 3))

    for i in range(num_instances):
        mask = masks[:, :, i]
        coord = coords[:, :, i, :]
        pts, idxs = backproject(depth, intrinsics, mask)
        coord_pts = coord[idxs[0], idxs[1], :] - 0.5
        try:
            start = time.time()
            s, R, T, outtransform = estimateSimilarityTransform(coord_pts, pts, False)
            elapsed = time.time() - start
            if verbose:
                print('elapsed: ', elapsed)
            elapses.append(elapsed)
        except Exception as e:
            message = '[ Error ] aligning instance {} in {} fails. Message: {}.'.format(instance_ids[i], img_path, str(e))
            print(message)
            error_messages += message + '\n'
            s = 1.0
            R = np.eye(3)
            T = np.zeros(3)
            outtransform = np.identity(4, dtype=np.float32)

        scales[i] = s / 1000.0
        rotations[i, :, :] = R
        translations[i, :] = T / 1000.0

    return scales, rotations, translations, error_messages, elapses

# estimate 9D transformation based on nunocs coordinate

def toOpen3dCloud(points,colors=None,normals=None):
    
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    if colors is not None:
      if colors.max()>1:
        colors = colors/255.0
      cloud.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))
    if normals is not None:
      cloud.normals = o3d.utility.Vector3dVector(normals.astype(np.float64))
    return cloud

def to_homo(pts):
  '''
  @pts: (N,3 or 2) will homogeneliaze the last dimension
  '''
  assert len(pts.shape)==2, f'pts.shape: {pts.shape}'
  homo = np.concatenate((pts, np.ones((pts.shape[0],1))),axis=-1)
  return homo

def estimateAffine3D(source,target,PassThreshold):
    '''
    @source: (N,3)
    '''
    ret,transform,inliers = cv2.estimateAffine3D(source, target,confidence=0.999,ransacThreshold=PassThreshold)
    tmp = np.eye(4)
    tmp[:3] = transform
    transform = tmp
    inliers = np.where(inliers>0)[0]
    return transform, inliers

def estimate9DTransform_worker(cur_src,cur_dst,source,target,PassThreshold,use_kdtree_for_eval=False,kdtree_eval_resolution=None,max_scale=np.array([99,99,99]),min_scale=np.array([0,0,0]),max_dimensions=None):
    bad_return = None,None,None
    transform,inliers = estimateAffine3D(source=cur_src,target=cur_dst,PassThreshold=PassThreshold)
    new_transform = transform.copy()
    scales = np.linalg.norm(transform[:3,:3],axis=0)
    if (scales>max_scale).any() or (scales<min_scale).any():
        return bad_return
    
    R = transform[:3,:3]/scales.reshape(1,3)
    u,s,vh = np.linalg.svd(R)

    if s.min()<0.8 or s.max()>1.2:
        return bad_return

    R = u@vh
    if np.linalg.det(R)<0:
        return bad_return
    
    new_transform[:3,:3] = R@np.diag(scales)
    transform = new_transform.copy()

    if max_dimensions is not None:
        cloud_at_canonical = (np.linalg.inv(transform)@to_homo(target).T).T[:,:3]
        dimensions = cloud_at_canonical.max(axis=0)-cloud_at_canonical.min(axis=0)
        if (dimensions>max_dimensions).any():
            return bad_return

    src_transformed = (transform@to_homo(source).T).T[:,:3]

    if not use_kdtree_for_eval:
        errs = np.linalg.norm(src_transformed-target,axis=-1)
        ratio = np.sum(errs<=PassThreshold)/len(errs)
        inliers = np.where(errs<=PassThreshold)[0]
    else:
        pcd = toOpen3dCloud(target)
        pcd = pcd.voxel_down_sample(voxel_size=kdtree_eval_resolution)
        kdtree = cKDTree(np.asarray(pcd.points).copy())
        dists1,indices1 = kdtree.query(src_transformed)
        pcd = toOpen3dCloud(src_transformed)
        pcd = pcd.voxel_down_sample(voxel_size=kdtree_eval_resolution)
        kdtree = cKDTree(np.asarray(pcd.points).copy())
        dists2,indices2 = kdtree.query(target)
        errs = np.concatenate((dists1,dists2),axis=0).reshape(-1)
        ratio = np.sum(errs<=PassThreshold)/len(errs)
        inliers = np.where(dists1<=PassThreshold)[0]

    return ratio,transform,inliers

def estimate9DTransform(source,target,PassThreshold,max_iter=10000,use_kdtree_for_eval=False,kdtree_eval_resolution=None,max_scale=np.array([99,99,99]),min_scale=np.array([0,0,0]),max_dimensions=None):
    best_transform = None
    best_ratio = 0
    inliers = None

    n_iter = 0
    srcs = []
    dsts = []
    for i in range(max_iter):
        ids = np.random.choice(len(source),size=4,replace=False)
        cur_src = source[ids]
        cur_dst = target[ids]
        srcs.append(cur_src)
        dsts.append(cur_dst)
    
    outs = []
    for i in range(len(srcs)):
        out = estimate9DTransform_worker(srcs[i],dsts[i],source,target,PassThreshold,use_kdtree_for_eval,kdtree_eval_resolution=kdtree_eval_resolution,max_scale=max_scale,min_scale=min_scale,max_dimensions=max_dimensions)
        if out[0] is None:
            continue
        outs.append((out))
    if len(outs)==0:
        return None,None

    ratios = []
    transforms = []
    inlierss = []
    for out in outs:
        ratio,transform,inliers = out
        ratios.append(ratio)
        transforms.append(transform)
        inlierss.append(inliers)

    best_id = np.array(ratios).argmax()
    best_transform = transforms[best_id]
    inliers = inlierss[best_id]

    return best_transform, inliers

    # # fitting the final transformation using the best inlier
    # best_source = source[inliers]
    # best_target = target[inliers]

    # final_transform, _ = estimateAffine3D(source=best_source,target=best_target,PassThreshold=PassThreshold)
    # new_final_transform = final_transform.copy()
    # scales = np.linalg.norm(final_transform[:3,:3],axis=0)
    
    # R = final_transform[:3,:3]/scales.reshape(1,3)
    # u,s,vh = np.linalg.svd(R)
    # R = u@vh
    
    # new_final_transform[:3,:3] = R@np.diag(scales)
    # final_transform = new_final_transform.copy()

    # return final_transform, inliers

def getRANSACInliers(source, target, n_sample=3, MaxIterations=100, PassThreshold=None, est_scale=True):
    '''
    @source: (N,3)
    '''
    SourceHom = np.transpose(np.hstack([source, np.ones([source.shape[0], 1])]))  #(4,N)
    TargetHom = np.transpose(np.hstack([target, np.ones([source.shape[0], 1])]))

    BestInlierRatio = 0
    BestInlierIdx = np.arange(3)
    best_res_vec = None
    for i in range(0, MaxIterations):
        RandIdx = np.random.choice(np.arange(SourceHom.shape[1]), size=n_sample,replace=False)
        Scales, Rotation, Translation, OutTransform = estimateSimilarityUmeyama(SourceHom[:, RandIdx], TargetHom[:, RandIdx], est_scale=est_scale)
        if not np.isfinite(OutTransform).all():
            continue
        ResidualVec, InlierRatio, InlierIdx = evaluateModel(OutTransform, SourceHom, TargetHom, PassThreshold)
        if InlierRatio>BestInlierRatio:
            BestInlierRatio = InlierRatio
            BestInlierIdx = InlierIdx
            best_res_vec = ResidualVec
    return SourceHom[:, BestInlierIdx], TargetHom[:, BestInlierIdx], BestInlierRatio, BestInlierIdx

def evaluateModel(OutTransform, SourceHom, TargetHom, PassThreshold):
    Diff = TargetHom - np.matmul(OutTransform, SourceHom)
    ResidualVec = np.linalg.norm(Diff[:3, :], axis=0)
    InlierIdx = np.where(ResidualVec < PassThreshold)[0]
    if len(InlierIdx)<5:
        return ResidualVec,0,np.arange(3)

    nInliers = np.sum(ResidualVec < PassThreshold)
    InlierRatio = nInliers / float(SourceHom.shape[1])
    return ResidualVec, InlierRatio, InlierIdx

def evaluateModelNoThresh(OutTransform, SourceHom, TargetHom):
    Diff = TargetHom - np.matmul(OutTransform, SourceHom)
    ResidualVec = np.linalg.norm(Diff[:3, :], axis=0)
    Residual = np.linalg.norm(ResidualVec)
    return Residual

def evaluateModelNonHom(source, target, Scales, Rotation, Translation):
    RepTrans = np.tile(Translation, (source.shape[0], 1))
    TransSource = (np.diag(Scales) @ Rotation @ source.transpose() + RepTrans.transpose()).transpose()
    Diff = target - TransSource
    ResidualVec = np.linalg.norm(Diff, axis=0)
    Residual = np.linalg.norm(ResidualVec)
    return Residual

# icp to refine rotation
def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # # translate points to their centroids
    # centroid_A = np.mean(A, axis=0)
    # centroid_B = np.mean(B, axis=0)
    # AA = A - centroid_A
    # BB = B - centroid_B

    # rotation matrix
    H = np.dot(A.T, B)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m-1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # # translation
    # t = centroid_B.T - np.dot(R,centroid_A.T)

    # # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    # T[:m, m] = t

    return T, R

def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    assert src.shape == dst.shape

    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()

def icp(A, B, max_iterations=20, tolerance=0.001):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of source mD points
        B: Nxm numpy array of destination mD point
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # make points homogeneous, copy them to maintain the originals
    src = np.ones((m+1,A.shape[0]))
    dst = np.ones((m+1,B.shape[0]))
    src[:m,:] = np.copy(A.T)
    dst[:m,:] = np.copy(B.T)

    prev_error = 0
    for i in range(max_iterations):
        # find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src[:m,:].T, dst[:m,:].T)

        # compute the transformation between the current source and nearest destination points
        T, _ = best_fit_transform(src[:m,:].T, dst[:m,indices].T)

        # update the current source
        src = np.dot(T, src)

        # check error
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    # calculate final transformation
    T, _ = best_fit_transform(A, src[:m,:].T)

    return T, distances, i


if __name__ == '__main__':
    support_path = '../cad_models/bottle/support.pkl'
    with open(support_path, 'rb') as f:
        data = cPickle.load(f)
        f.close()

    mesh = o3d.io.read_triangle_mesh('../cad_models/bottle/model.obj')
    pcd = mesh.sample_points_uniformly(number_of_points=1024)
    pts = np.array(pcd.points)
    obj_min, obj_max = np.min(pts, axis=0), np.max(pts, axis=0)

    print('object size:', obj_max - obj_min)
    
    for v in range(len(data)):
        img = data[v]['rgb']
        mask = data[v]['mask']
        depth = data[v]['depth']
        pose = data[v]['object_pose']
        k = data[v]['K']
        nunocs = data[v]['nunocs_map']
        depth_scale = 1000.0

        xmap = np.array([[i for i in range(img.shape[1])] for j in range(img.shape[0])])
        ymap = np.array([[j for i in range(img.shape[1])] for j in range(img.shape[0])])

        mask = (depth > 0)
        choose = mask.flatten().nonzero()[0]

        depth_masked = depth.flatten()[choose][:, np.newaxis]
        xmap_masked = xmap.flatten()[choose][:, np.newaxis]
        ymap_masked = ymap.flatten()[choose][:, np.newaxis]
        pt2 = depth_masked / depth_scale
        pt0 = (xmap_masked - k[0,2]) * pt2 / k[0,0]
        pt1 = (ymap_masked - k[1,2]) * pt2 / k[1,1]
        points = np.concatenate((pt0, pt1, pt2), axis=1)

        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(points)
        # o3d.io.write_point_cloud("pts.ply", pcd)

        nunocs_pts = nunocs.reshape(-1, 3)[choose][:, (2,1,0)].astype(np.float32) / 255.0 - 0.5

        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(nunocs_pts)
        # o3d.io.write_point_cloud("pts.ply", pcd)

        # cv2.imwrite('t.png', img)
        # cv2.imwrite('nunocs.png', nunocs)

        rt, inliers = estimate9DTransform(nunocs_pts, points, 0.003, 10000, False, 0.003, [1.5,1.5,1.5], [0.005,0.005,0.005], np.array([1.2,1.2,1.2]))
        scales = np.linalg.norm(rt[:3,:3],axis=0)

        print('pred size:', scales)

        x = rt[:3, 0] / scales[0]
        y = rt[:3, 1] / scales[1]
        z = rt[:3, 2] / scales[2]

        pred_r = np.eye(3)
        pred_r[:3, 0] = x
        pred_r[:3, 1] = y
        pred_r[:3, 2] = z

        R12 = pred_r @ pose[:3,:3].transpose()

        R_error = np.arccos(np.clip((np.trace(R12)-1)/2, -1.0, 1.0)) * 180 / np.pi
        T_error = np.linalg.norm(rt[:3,3] - pose[:3,3])

        print('view_{}:'.format(v), R_error, T_error)