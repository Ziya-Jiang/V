import os
import cv2
import math
import random
import copy
import json
import numpy as np
import _pickle as cPickle
import matplotlib.pyplot as plt
from skimage.segmentation import slic
from PIL import Image
import torch
import torch.nn.functional as F
import torch.utils.data as data
import torchvision.transforms as transforms
import scipy.io as sio
from tqdm import tqdm

import sys
sys.path.append('.')
from ram_utils import load_depth, aug_get_bbox, get_bbox, transform_coordinates_3d

import matplotlib.pyplot as plt
import open3d as o3d

def reshape_dino_feats(feats, layers, feat_type, patch_num, with_register = False):
    select_feats_with_class_token = []
    select_feats_no_class_token = []
    for l in layers:
        if feat_type != 't':
            b, h, n, d = feats[l][feat_type].shape
            class_token = feats[l][feat_type].transpose(0, 1, 3, 2)[:, :, :, 0].reshape(b, h*d, 1).astype(np.float32)

            if with_register: # 1 class token + 4 register tokens
                patch_token = feats[l][feat_type].transpose(0, 1, 3, 2)[:, :, :, 5:].reshape(b, h*d, -1).astype(np.float32)
            else:
                patch_token = feats[l][feat_type].transpose(0, 1, 3, 2)[:, :, :, 1:].reshape(b, h*d, -1).astype(np.float32)
            class_token = np.tile(class_token, (1,1,patch_token.shape[-1]))

            # feat_with_class_token = np.concatenate([patch_token, class_token], axis=1).reshape(b, -1, patch_num, patch_num)
            feat_with_class_token = (patch_token + class_token).reshape(b, -1, patch_num, patch_num)
            feat_no_class_token = patch_token.reshape(b, -1, patch_num, patch_num)

            select_feats_with_class_token.append(feat_with_class_token)
            select_feats_no_class_token.append(feat_no_class_token)

            # select_feats.append((patch_token + class_token).reshape(b, -1, patch_num, patch_num))
            # select_feats.append(feats[l][feat_type].transpose((0, 1, 3, 2))[:, :, :, 1:].reshape(b, h*d, -1).reshape(b, -1, patch_num, patch_num))
        else:
            b, n, d = feats[l][feat_type].shape
            class_token = feats[l][feat_type].transpose(0, 2, 1)[:, :, 0][:, :, None].astype(np.float32)

            if with_register:
                patch_token = feats[l][feat_type].transpose(0, 2, 1)[:, :, 5:].astype(np.float32)
            else:
                patch_token = feats[l][feat_type].transpose(0, 2, 1)[:, :, 1:].astype(np.float32)

            class_token = np.tile(class_token, (1,1,patch_token.shape[-1]))

            # feat_with_class_token = np.concatenate([patch_token, class_token], axis=1).reshape(b, -1, patch_num, patch_num)
            feat_with_class_token = (patch_token + class_token).reshape(b, -1, patch_num, patch_num)
            feat_no_class_token = patch_token.reshape(b, -1, patch_num, patch_num)

            select_feats_with_class_token.append(feat_with_class_token)
            select_feats_no_class_token.append(feat_no_class_token)

            # select_feats.append((patch_token + class_token).reshape(b, -1, patch_num, patch_num))
            # select_feats.append(feats[l][feat_type].transpose((0, 2, 1))[:, :, 1:].reshape(b, -1, patch_num, patch_num))
    
    select_feats_with_class_token = np.concatenate(select_feats_with_class_token, axis=1)
    select_feats_no_class_token = np.concatenate(select_feats_no_class_token, axis=1)

    # # for network v4, simplely average multi-level features
    # select_feats_with_class_token = np.mean(np.stack(select_feats_with_class_token, axis=0), axis=0)
    # select_feats_no_class_token = np.mean(np.stack(select_feats_no_class_token, axis=0), axis=0)

    return select_feats_with_class_token, select_feats_no_class_token

def normalize_shape_pts(pts):
    if len(pts.shape) == 3:
        center = np.mean(pts, axis=1)
        pts = pts - center[:, np.newaxis, :]
        s = np.max(np.sqrt(np.sum(pts ** 2, axis=2)), axis=1)
        pts = pts / s[:, np.newaxis, np.newaxis]
    
    if len(pts.shape) == 2:
        center = np.mean(pts, axis=0)
        pts = pts - center[np.newaxis, :]
        s = np.max(np.sqrt(np.sum(pts ** 2, axis=1)))
        pts = pts / s

    return pts

def sample_support_information_v0521(mask, feat_map_with_class_token, feat_map_np_class_token, \
    nocs_map, depth, K, depth_scale = 0.1, n_pts = 1024):
    h, w = mask.shape
    xmap = np.array([[i for i in range(w)] for j in range(h)])
    ymap = np.array([[j for i in range(w)] for j in range(h)])

    # sample 1024 positions from the mask image
    choose = mask.flatten().nonzero()[0]
    init_n_pts = int(n_pts * 1.25)
    if len(choose) > init_n_pts:
        c_mask = np.zeros(len(choose), dtype=int)
        c_mask[:init_n_pts] = 1
        np.random.shuffle(c_mask)
        choose = choose[c_mask.nonzero()]
    else:
        choose = np.pad(choose, (0, init_n_pts-len(choose)), 'wrap')
    
    depth_masked = depth.flatten()[choose][:, np.newaxis]
    xmap_masked = xmap.flatten()[choose][:, np.newaxis]
    ymap_masked = ymap.flatten()[choose][:, np.newaxis]
    pt2 = depth_masked * depth_scale
    pt0 = (xmap_masked - K[0,2]) * pt2 / K[0,0]
    pt1 = (ymap_masked - K[1,2]) * pt2 / K[1,1]
    points = np.concatenate((pt0, pt1, pt2), axis=1)

    points = normalize_shape_pts(points)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    labels = point_cloud.cluster_dbscan(eps=0.05, min_points=5)
    unique_labels, counts = np.unique(labels, return_counts=True)

    valid_labels = unique_labels[unique_labels != -1]
    valid_counts = counts[unique_labels != -1]
    sorted_indices = np.argsort(valid_counts)[::-1]
    sorted_labels = valid_labels[sorted_indices]
    sorted_counts = valid_counts[sorted_indices]

    retain_threshold = int(points.shape[0] * 0.8)
    inlier_indices = []
    current_count = 0
    for label in sorted_labels:
        label_indices = np.where(labels == label)[0]
        inlier_indices.extend(label_indices)
        current_count += len(label_indices)     

        if current_count >= retain_threshold:
            break
    if inlier_indices:
        inlier_cloud = point_cloud.select_by_index(inlier_indices)
    else:
        print(f"cannot find valid point cloud\n")
    
    choose = choose[inlier_indices]
    if len(choose) > n_pts:
        c_mask = np.zeros(len(choose), dtype=int)
        c_mask[:n_pts] = 1
        np.random.shuffle(c_mask)
        choose = choose[c_mask.nonzero()]
    else:
        choose = np.pad(choose, (0, n_pts-len(choose)), 'wrap')

    feat_map_with_class_token = F.interpolate(torch.from_numpy(feat_map_with_class_token), \
        size = (h, w), mode = 'bilinear', align_corners=False)
    
    _, d, _, _ = feat_map_with_class_token.shape
    feat_map_with_class_token = feat_map_with_class_token.squeeze().permute(1,2,0).numpy()
    feat_with_class_token = feat_map_with_class_token.reshape((-1, d))[choose, :]

    feat_map_np_class_token = F.interpolate(torch.from_numpy(feat_map_np_class_token), \
        size = (h, w), mode = 'bilinear', align_corners=False)
    
    _, d, _, _ = feat_map_np_class_token.shape
    feat_map_np_class_token = feat_map_np_class_token.squeeze().permute(1,2,0).numpy()
    feat_no_class_token = feat_map_np_class_token.reshape((-1, d))[choose, :]

    # sample point-wise nocs coordinates
    coord = nocs_map.astype(np.float32) / 255
    nocs = coord.reshape((-1, 3))[choose, :] - 0.5 # (1024, 3)

    # sample pts from depth
    xmap = np.array([[i for i in range(w)] for j in range(h)])
    ymap = np.array([[j for i in range(w)] for j in range(h)])
    depth_masked = depth.flatten()[choose][:, np.newaxis]
    xmap_masked = xmap.flatten()[choose][:, np.newaxis]
    ymap_masked = ymap.flatten()[choose][:, np.newaxis]
    pt2 = depth_masked * depth_scale
    pt0 = (xmap_masked - K[0,2]) * pt2 / K[0,0]
    pt1 = (ymap_masked - K[1,2]) * pt2 / K[1,1]
    points = np.concatenate((pt0, pt1, pt2), axis=1)
    pixels = np.concatenate((xmap_masked, ymap_masked), axis=1)

    return feat_with_class_token, feat_no_class_token, nocs, points, choose, pixels

def sample_support_information(mask, feat_map_with_class_token, feat_map_np_class_token, \
    nocs_map, depth, K, depth_scale = 0.1, n_pts = 1024):
    h, w = mask.shape
    # sample 1024 positions from the mask image
    choose = mask.flatten().nonzero()[0]
    if len(choose) > n_pts:
        c_mask = np.zeros(len(choose), dtype=int)
        c_mask[:n_pts] = 1
        np.random.shuffle(c_mask)
        choose = choose[c_mask.nonzero()]
    else:
        choose = np.pad(choose, (0, n_pts-len(choose)), 'wrap')

    feat_map_with_class_token = F.interpolate(torch.from_numpy(feat_map_with_class_token), \
        size = (h, w), mode = 'bilinear', align_corners=False)
    
    _, d, _, _ = feat_map_with_class_token.shape
    feat_map_with_class_token = feat_map_with_class_token.squeeze().permute(1,2,0).numpy()
    feat_with_class_token = feat_map_with_class_token.reshape((-1, d))[choose, :]

    feat_map_np_class_token = F.interpolate(torch.from_numpy(feat_map_np_class_token), \
        size = (h, w), mode = 'bilinear', align_corners=False)
    
    _, d, _, _ = feat_map_np_class_token.shape
    feat_map_np_class_token = feat_map_np_class_token.squeeze().permute(1,2,0).numpy()
    feat_no_class_token = feat_map_np_class_token.reshape((-1, d))[choose, :]

    # sample point-wise nocs coordinates
    coord = nocs_map.astype(np.float32) / 255
    nocs = coord.reshape((-1, 3))[choose, :] - 0.5 # (1024, 3)

    # sample pts from depth
    xmap = np.array([[i for i in range(w)] for j in range(h)])
    ymap = np.array([[j for i in range(w)] for j in range(h)])
    depth_masked = depth.flatten()[choose][:, np.newaxis]
    xmap_masked = xmap.flatten()[choose][:, np.newaxis]
    ymap_masked = ymap.flatten()[choose][:, np.newaxis]
    pt2 = depth_masked * depth_scale
    pt0 = (xmap_masked - K[0,2]) * pt2 / K[0,0]
    pt1 = (ymap_masked - K[1,2]) * pt2 / K[1,1]
    points = np.concatenate((pt0, pt1, pt2), axis=1)
    pixels = np.concatenate((xmap_masked, ymap_masked), axis=1)

    return feat_with_class_token, feat_no_class_token, nocs, points, choose, pixels

def position_encoding(emb_dim, max_length=60000):
    position = np.arange(max_length)[:, np.newaxis]
    positional_encoding = np.zeros((1, max_length, emb_dim)).astype(np.float32)

    _2i = np.arange(0, emb_dim, step=2).astype(np.float32)
    positional_encoding[0, :, 0::2] = np.sin(position / (10000 ** (_2i / emb_dim)))
    positional_encoding[0, :, 1::2] = np.cos(position / (10000 ** (_2i / emb_dim)))

    return positional_encoding[0]

def load_template_information(template_file, pe_table, feat_layers, feat_type, patch_num, n_pts, foundation_model_name):
    nocs_maps = []
    dino_feat_maps = []
    dino_feat_vector_with_class_token = []
    dino_feat_vector_no_class_token = []
    nocs_vector = []
    template_pts = []
    template_pts2d = []
    poses = []
    rgbs = []
    masks = []
    position_encodings = []
    chooses = []

    for i in range(len(template_file)):
        rgbs.append(template_file[i]['rgb'])
        masks.append(template_file[i]['mask'])
        poses.append(template_file[i]['object_pose'])

        # (1, 768, 28, 28)
        if 'clip' in foundation_model_name:
            dino_feat_with_class_token, dino_feat_no_class_token = \
                reshape_dino_feats(template_file[i]['clip_feat'], feat_layers, feat_type, patch_num)
        elif 'mvp' in foundation_model_name:
            dino_feat_with_class_token, dino_feat_no_class_token = \
                reshape_dino_feats(template_file[i]['mvp_feat'], feat_layers, feat_type, patch_num)
        elif 'dinov2' in foundation_model_name:
            if 'register' in foundation_model_name:
                dino_feat_with_class_token, dino_feat_no_class_token = \
                    reshape_dino_feats(template_file[i]['dinov2_register_s14_feat'], feat_layers, feat_type, patch_num, True)
            else:
                dino_feat_with_class_token, dino_feat_no_class_token = \
                    reshape_dino_feats(template_file[i]['dinov2_s14_feat'], feat_layers, feat_type, patch_num)
                # dino_feat_with_class_token, dino_feat_no_class_token = \
                #     reshape_dino_feats(support_data[i]['dinov2_b14_feat'], layers, feat_type, patch_num)
                # dino_feat_with_class_token, dino_feat_no_class_token = \
                #     reshape_dino_feats(support_data[i]['dinov2_l14_feat'], layers, feat_type, patch_num)
        else:    
            dino_feat_with_class_token, dino_feat_no_class_token = \
                reshape_dino_feats(template_file[i]['dinov1_s14_feat'], feat_layers, feat_type, patch_num)

        dino_vec_with_class_token, dino_vec_no_class_token, nocs_vec, pts, choose, pts2d = \
            sample_support_information(template_file[i]['mask'], dino_feat_with_class_token, \
            dino_feat_no_class_token, template_file[i]['nocs_map'][:, :, (2, 1, 0)], \
            template_file[i]['depth'], template_file[i]['K'], 0.1, n_pts)
        
        chooses.append(choose)
        
        # dino_feat_maps.append(dino_feat_with_class_token)
            
        dino_feat_vector_with_class_token.append(dino_vec_with_class_token[np.newaxis, :, :])
        dino_feat_vector_no_class_token.append(dino_vec_no_class_token[np.newaxis, :, :])
        nocs_vector.append(nocs_vec[np.newaxis, :, :])
        template_pts.append(pts[np.newaxis, :, :])
        template_pts2d.append(pts2d[np.newaxis, :, :])
        position_encodings.append(pe_table[choose, :][np.newaxis, :, :])

        nocs_map = template_file[i]['nocs_map'][:, :, (2, 1, 0)]
        nocs_maps.append(nocs_map[np.newaxis, :, :, :])

    nocs_maps = np.concatenate(nocs_maps, axis=0) # (128, 224, 224, 3)
    dino_vecs_with_class_token = np.concatenate(dino_feat_vector_with_class_token, axis=0) # (128, 1024, 768)
    dino_vecs_no_class_token = np.concatenate(dino_feat_vector_no_class_token, axis=0) # (128, 1024, 768)
    nocs_vecs = np.concatenate(nocs_vector, axis=0) # (128, 1024, 3)
    template_pts = np.concatenate(template_pts, axis=0) #(128, 1024, 3)
    template_pts2d = np.concatenate(template_pts2d, axis=0) #(128, 1024, 2)
    position_encodings = np.concatenate(position_encodings, axis=0) # (128, 1024, 768)

    # dino_feat_maps = np.concatenate(dino_feat_maps, axis=0) # (128, 768, 16, 16)

    template = {}
    template['nocs'] = nocs_maps
    template['feat_vec_with_class_token'] = dino_vecs_with_class_token.astype(np.float16)
    template['feat_vec_no_class_token'] = dino_vecs_no_class_token.astype(np.float16)
    template['nocs_vec'] = nocs_vecs
    template['pts'] = template_pts
    template['pts2d'] = template_pts2d
    template['rgb'] = rgbs
    template['mask'] = masks
    template['reference_poses'] = poses
    template['position_encoding'] = position_encodings
    template['sample_choose'] = chooses

    # template['dino_feat_maps'] = dino_feat_maps.astype(np.float16)

    return template

def transform_coordinates_3d(coordinates, sRT):
    """
    Args:
        coordinates: [3, N]
        sRT: [4, 4]

    Returns:
        new_coordinates: [3, N]

    """
    assert coordinates.shape[0] == 3
    coordinates = np.vstack([coordinates, np.ones((1, coordinates.shape[1]), dtype=np.float32)])
    new_coordinates = sRT @ coordinates
    new_coordinates = new_coordinates[:3, :] / new_coordinates[3, :]
    return new_coordinates

class DINOPoseDataset(data.Dataset):
    def __init__(self, source, mode, data_dir, n_pts, img_size, feat_layers = [7, 9, 11], \
        feat_type = 'k', foundation_model = 'dinov2-s14', template_views = 128, use_nunocs = False):
        self.source = source
        self.mode = mode
        self.data_dir = data_dir
        self.n_pts = n_pts
        self.img_size = img_size

        self.layers = feat_layers
        self.feat_type = feat_type
        self.use_nunocs = use_nunocs
        self.template_views = template_views

        self.foundation_model = foundation_model

        if 'clip' in self.foundation_model or 'mvp' in self.foundation_model:
            self.pe_table = position_encoding(768)
        else:
            self.pe_table = position_encoding(len(self.layers) * 384)

        if 'clip' in self.foundation_model or 'mvp' in self.foundation_model:
            self.patch_num = int(224 // 16)
        elif 'dinov2' in self.foundation_model:
            self.patch_num = int( 224 // 14)
        elif 'dinov1' in self.foundation_model:
            self.patch_num = int(224 // 8)
        
        assert source in ['bop', 'real']
        assert mode in ['train', 'test']

        img_list_path = self.source + '_extend_' + self.mode + '_list.txt'
        # img_list_path = self.source + '_' + 'test' + '_list.txt'
        cad_model_file_path = self.source + '_cad_model.pkl'

        img_list = []
        img_list += [line.rstrip('\n') for line in open(os.path.join(data_dir, img_list_path))]

        self.img_list = img_list
        self.length = len(self.img_list)

        models = {}
        with open(os.path.join(self.data_dir, cad_model_file_path), 'rb') as f:
            models.update(cPickle.load(f))
        self.models = models

        # meta info for 120 object models in bop synthetic dataset
        obj_meta_path = os.path.join(self.data_dir, 'cad_models', 'model_meta.json')
        with open(obj_meta_path) as f:
            self.objs_meta = json.load(f)
            f.close()
        
        self.intrinsics = [572.411363389757, 573.5704328585578, 325.2611083984375, 242.04899588216654]    # [fx, fy, cx, cy]
        self.depth_scale = 0.1
        self.colorjitter = transforms.ColorJitter(0.2, 0.2, 0.2, 0.05)
        self.transform = transforms.Compose([transforms.ToTensor(),
                                             transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                                                  std=[0.229, 0.224, 0.225])])
        
        # load template information
        # self.template_dir = os.path.join(self.data_dir, 'template')
        if 'dinov1' in self.foundation_model:
            self.template_dir = os.path.join(self.data_dir, 'dinov1_template')

        if 'dinov2' in self.foundation_model:
            if 'register' in self.foundation_model:
                self.template_dir = os.path.join(self.data_dir, 'dinov2_reg_template')
            else:
                if self.foundation_model == 'dinov2-s14':
                    self.template_dir = os.path.join(self.data_dir, 'dinov2_template')
                
                if self.foundation_model == 'dinov2-b14':
                    self.template_dir = os.path.join(self.data_dir, 'dinov2_b14_template')
                
                if self.foundation_model == 'dinov2-l14':
                    self.template_dir = os.path.join(self.data_dir, 'dinov2_l14_template')
        
        if 'mvp' in self.foundation_model:
            self.template_dir = os.path.join(self.data_dir, 'mvp_template')
            self.feat_type = 't'
            self.layers = [11]
        
        if 'clip' in self.foundation_model:
            self.template_dir = os.path.join(self.data_dir, 'clip_template')
            self.feat_type = 't'
            self.layers = [11]

        self.training_category_config = { 'basket': list(range(13, 19)), 'bench': list(range(97, 103)), \
                    'cap': list(range(67, 73)), 'earphone': list(range(79, 85)), 'faucet': list(range(55, 61)), \
                    'guitar': list(range(49, 55)), 'knife': list(range(19, 25)), 'lamp': list(range(103, 109)), \
                    'loudspeaker': list(range(73, 79)), 'piano': list(range(31, 37)), 'pistol': list(range(85, 91)), \
                    'printer': list(range(61, 67)), 'rifle': list(range(109, 115)), 'skateboard': list(range(25, 31)), \
                    'sofa': list(range(1, 7)), 'table': list(range(7, 13)), 'telephone': list(range(37, 43)), \
                    'watercraft': list(range(115, 121)), 'kettle': list(range(501, 507)), 'wateringcan': list(range(507, 513)), \
                    'teapot': list(range(513, 519)), 'pan': list(range(519, 525)), 'camera': list(range(525, 531)), \
                    'shoe': list(range(531, 537)), 'stapler': list(range(537, 543)), 'spatula': list(range(549, 555)), \
                    'whisk': list(range(555, 561))}
        
        # self.training_category_config = {'bed': list(range(91, 97)), 'stove': list(range(43, 49))}

        self.testing_category_config = {'bed': list(range(91, 97)), 'stove': list(range(43, 49)), 'ladle': list(range(543, 549))}

        self.template = {}
        self.template_id = {}
        self.__preload__template__()

    def __len__(self):
        return self.length
    
    def change_template(self):
        self.__preload__template__()
    
    def __preload__template__(self):
        if self.mode == 'train':
            cate_config = self.training_category_config
        else:
            cate_config = self.testing_category_config

        for cate in tqdm(cate_config.keys()):
            template_obj_id = random.choice(cate_config[cate])
            with open(os.path.join(self.template_dir, str(template_obj_id).zfill(6) + '.pkl'), 'rb') as f:
                template_pkl = cPickle.load(f)
                f.close()

            self.template_id[cate] = str(template_obj_id)
            self.template[cate] = load_template_information(template_pkl, self.pe_table, self.layers, \
                self.feat_type, self.patch_num, self.n_pts, self.foundation_model)
    
    def __compute_orientation_error__(self, template_r, query_r, is_symmetry, symmetry_axis = 'z'):
        if is_symmetry:
            z = np.array([0, 0, 1])
            z1 = template_r @ z
            z2 = query_r @ z
            cos_theta = z1.dot(z2) / (np.linalg.norm(z1) * np.linalg.norm(z2))
        else:
            R = template_r @ query_r.transpose()
            cos_theta = (np.trace(R) - 1) / 2

        delt_theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))

        return delt_theta
    
    def __normalize_shape_pts__(self, pts):
        if len(pts.shape) == 3:
            center = np.mean(pts, axis=1)
            pts = pts - center[:, np.newaxis, :]
            s = np.max(np.sqrt(np.sum(pts ** 2, axis=2)), axis=1)
            pts = pts / s[:, np.newaxis, np.newaxis]
        
        if len(pts.shape) == 2:
            center = np.mean(pts, axis=0)
            pts = pts - center[np.newaxis, :]
            s = np.max(np.sqrt(np.sum(pts ** 2, axis=1)))
            pts = pts / s

        return pts

    def __getitem__(self, index):
        img_path = self.img_list[index]

        root_dir, obj_cate, obj_id, img_idx = img_path.split(',')

        # load gt pose .json file
        gt_path = os.path.join(root_dir, 'scene_gt.json')
        with open(gt_path) as f:
            gts = json.load(f)
            f.close()
        
        gt_r = np.array(gts[str(int(img_idx))][0]['cam_R_m2c']).reshape(3, 3)
        gt_t = np.array(gts[str(int(img_idx))][0]['cam_t_m2c']).flatten()
        assert (str(gts[str(int(img_idx))][0]['obj_id']) == obj_id)

        # load rgb, mask, depth, and nocs map
        rgb_path = os.path.join(root_dir, 'rgb', img_idx + '.jpg')
        mask_path = os.path.join(root_dir, 'mask', img_idx + '_000000.png')
        depth_path = os.path.join(root_dir, 'depth', img_idx + '.png')
        nocs_path = os.path.join(root_dir, 'nocs', img_idx + '.png')

        rgb = cv2.imread(rgb_path)[:, :, :3]
        rgb = rgb[:, :, ::-1]
        h, w, _ = rgb.shape

        cam_fx, cam_fy, cam_cx, cam_cy = self.intrinsics

        xmap = np.array([[i for i in range(w)] for j in range(h)])
        ymap = np.array([[j for i in range(w)] for j in range(h)])

        depth = cv2.imread(depth_path, -1)
        
        mask = cv2.imread(mask_path, -1)

        # # mask augmentation for training
        # if self.mode == 'train':
        #     num_slic = 200
        #     slic_segs = slic(rgb, n_segments=num_slic, compactness=20, mask=mask)
        #     mask_ratio = np.random.uniform(0.15)
        #     slic_idx = np.arange(num_slic) + 1
        #     random.shuffle(slic_idx)
        #     masked_slic = slic_idx[:int(num_slic*mask_ratio)]
        #     valid_mask = np.in1d(slic_segs.flatten(), masked_slic, invert=True).reshape(slic_segs.shape)
        #     mask = np.logical_and(mask, valid_mask)
        #     # cv2.imwrite('slic.png', mask.astype(np.uint8)*255)

        # if len(np.nonzero(mask)[0]) == 0:
        #     mask = cv2.imread(mask_path, -1)

        nocs = cv2.imread(nocs_path, -1)[:, :, ::-1].astype(np.float32) / 255

        ys, xs = np.nonzero(mask)
        y1 = np.min(ys)
        y2 = np.max(ys)
        x1 = np.min(xs)
        x2 = np.max(xs)

        # # random translation for data augmentation
        # if self.mode == 'train':
        #     rmin, rmax, cmin, cmax = aug_get_bbox([y1, x1, y2, x2], img_w = w, img_h = h)
        # else:
        #     rmin, rmax, cmin, cmax = get_bbox([y1, x1, y2, x2], img_w = w, img_h = h)
        rmin, rmax, cmin, cmax = get_bbox([y1, x1, y2, x2], img_w = w, img_h = h)

        choose = mask[rmin:rmax, cmin:cmax].flatten().nonzero()[0]
        # sample pts from depth map based on 'choose'
        if len(choose) > self.n_pts:
            c_mask = np.zeros(len(choose), dtype=int)
            c_mask[:self.n_pts] = 1
            np.random.shuffle(c_mask)
            choose = choose[c_mask.nonzero()]
        else:
            choose = np.pad(choose, (0, self.n_pts-len(choose)), 'wrap')
        
        depth_masked = depth[rmin:rmax, cmin:cmax].flatten()[choose][:, np.newaxis]
        xmap_masked = xmap[rmin:rmax, cmin:cmax].flatten()[choose][:, np.newaxis]
        ymap_masked = ymap[rmin:rmax, cmin:cmax].flatten()[choose][:, np.newaxis]
        pt2 = depth_masked * self.depth_scale
        pt0 = (xmap_masked - cam_cx) * pt2 / cam_fx
        pt1 = (ymap_masked - cam_cy) * pt2 / cam_fy
        points = np.concatenate((pt0, pt1, pt2), axis=1)
        query_shape_pts = self.__normalize_shape_pts__(copy.deepcopy(points))

        nocs = nocs[rmin:rmax, cmin:cmax, :].reshape((-1, 3))[choose, :] - 0.5
        rgb = rgb[rmin:rmax, cmin:cmax, :]
        rgb = cv2.resize(rgb, (self.img_size, self.img_size), interpolation=cv2.INTER_LINEAR)

        # adjust choose to align to the resized rgb image
        crop_w = rmax - rmin
        ratio = self.img_size / crop_w
        col_idx = choose % crop_w
        row_idx = choose // crop_w
        choose = (np.floor(row_idx * ratio) * self.img_size + np.floor(col_idx * ratio)).astype(np.int64)

        # data augmentation
        if self.mode == 'train':
            # color jitter
            rgb = self.colorjitter(Image.fromarray(np.uint8(rgb)))
            rgb = np.array(rgb)
        
        rgb = self.transform(rgb)

        # load meta info
        obj_meta = self.objs_meta[obj_id]
        obj_scale = obj_meta['object_scale']
        is_symmetry = (obj_meta['is_symmetric'] == 'True')

        template_complete_nocs = self.models[self.template_id[obj_cate]] / \
            self.objs_meta[self.template_id[obj_cate]]['object_scale']
        query_complete_nocs = self.models[obj_id] / obj_scale

        # # check query_pts, query_nocs, gt_r, and gt_t
        # sRT = np.eye(4).astype(np.float32)
        # sRT[:3, :3] = gt_r * (obj_scale / 1000.0)
        # sRT[:3, 3] = gt_t / 1000.0

        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(points / 1000.0)
        # o3d.io.write_point_cloud('depth_pts.ply', pcd)

        # transformed_nocs = transform_coordinates_3d(nocs.T, sRT).T
        # pcd2 = o3d.geometry.PointCloud()
        # pcd2.points = o3d.utility.Vector3dVector(transformed_nocs)
        # o3d.io.write_point_cloud('nocs_pts.ply', pcd2)

        # # check query_nocs and query shape
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(nocs)
        # o3d.io.write_point_cloud('nocs1.ply', pcd)

        # query_shape = self.models[obj_id] / obj_scale
        # pcd2 = o3d.geometry.PointCloud()
        # pcd2.points = o3d.utility.Vector3dVector(query_shape)
        # o3d.io.write_point_cloud('nocs2.ply', pcd2)
        
        # compute the orientation distance between query image and all template views
        template_orientation_errors = []
        for i in range(self.template_views):
            template_orientation_errors.append(self.__compute_orientation_error__(\
                self.template[obj_cate]['reference_poses'][i][:3, :3], \
                gt_r, is_symmetry))
        
        # orientation error for all template images
        sorted_view_id = np.array(template_orientation_errors).argsort()
        top_k_closese_view_id = sorted_view_id[:5]
        closest_view = top_k_closese_view_id[0]
        select_template_view = random.choice(list(top_k_closese_view_id))

        select_template_r = self.template[obj_cate]['reference_poses'][select_template_view][:3, :3]

        template_orientation_errors = -10.0 * np.array(template_orientation_errors)
        softmax_template_orientation_errors = np.exp(template_orientation_errors - np.max(template_orientation_errors))
        softmax_template_orientation_errors = softmax_template_orientation_errors / softmax_template_orientation_errors.sum(axis=0)
        softmax_template_orientation_errors = softmax_template_orientation_errors.astype(np.float32)

        # sample 8 views from all 128 views
        sv = 8
        step = self.template_views // sv
        sampled_template_views = []
        for i in range(sv):
            a = int(i * step)
            b = int((i + 1) * step)
            current_view = random.choice(list(sorted_view_id[a:b]))
            if i == 0:
                sampled_template_views.append(closest_view)
            else:
                sampled_template_views.append(current_view)
        
        sampled_orientation_errors = np.array(template_orientation_errors[sampled_template_views])
        sampled_softmax_template_orientation_errors = np.exp(sampled_orientation_errors - np.max(sampled_orientation_errors))
        sampled_softmax_template_orientation_errors = sampled_softmax_template_orientation_errors / sampled_softmax_template_orientation_errors.sum(axis=0)
        sampled_softmax_template_orientation_errors = sampled_softmax_template_orientation_errors.astype(np.float32)

        # the most 8 hardest views from all 128 views
        sampled_hard_template_views = sorted_view_id[:sv]
        sampled_hard_orientation_errors = np.array(template_orientation_errors[sampled_hard_template_views])
        sampled_softmax_hard_template_orientation_errors = np.exp(sampled_hard_orientation_errors - np.max(sampled_hard_orientation_errors))
        sampled_softmax_hard_template_orientation_errors = sampled_softmax_hard_template_orientation_errors / sampled_softmax_hard_template_orientation_errors.sum(axis=0)
        sampled_softmax_hard_template_orientation_errors = sampled_softmax_hard_template_orientation_errors.astype(np.float32)

        # # check the selected closest template view
        # plt.figure(figsize=(10, 5))
        # plt.subplot(1, 2, 1)
        # plt.imshow(rgb)

        # plt.subplot(1, 2, 2)
        # plt.imshow(self.template[obj_cate]['rgb'][closest_view])

        # plt.savefig('debug.png')

        # 
        template_meta = self.objs_meta[self.template_id[obj_cate]]
        template_obj_scale = template_meta['object_scale']

        template_shape_pts = self.__normalize_shape_pts__(self.template[obj_cate]['pts'])
        # template_shape_pts = self.__normalize_shape_pts__(self.template[obj_cate]['pts'][select_template_view])

        template_feat_with_class_token = self.template[obj_cate]['feat_vec_with_class_token'].transpose(0,2,1)
        template_choose = self.template[obj_cate]['position_encoding'].transpose(0,2,1)
        template_feat_no_class_token = self.template[obj_cate]['feat_vec_no_class_token'].transpose(0,2,1)
        template_nocs_vec = self.template[obj_cate]['nocs_vec']
        template_obj_pose = self.template[obj_cate]['reference_poses']

        # # check template_pts, template_nocs, template_pose
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(self.template[obj_cate]['pts'][select_template_view])
        # o3d.io.write_point_cloud('depth_pts.ply', pcd)

        # template_nocs = self.template[obj_cate]['nocs_vec'][select_template_view]
        # template_sRT = self.template[obj_cate]['reference_poses'][select_template_view]
        # template_sRT[:3, :3] = template_obj_scale * template_sRT[:3, :3]

        # print(template_nocs.shape, template_sRT.shape)

        # transformed_nocs = transform_coordinates_3d(template_nocs.T, template_sRT).T
        # pcd2 = o3d.geometry.PointCloud()
        # pcd2.points = o3d.utility.Vector3dVector(transformed_nocs)
        # o3d.io.write_point_cloud('nocs_pts.ply', pcd2)

        query_complete_nocs = query_complete_nocs.astype(np.float32)
        template_complete_nocs = template_complete_nocs.astype(np.float32)
        nocs = nocs.astype(np.float32)
        points = points.astype(np.float32)
        query_shape_pts = query_shape_pts.astype(np.float32)

        template_shape_pts = template_shape_pts.astype(np.float32)
        select_template_nocs = template_nocs_vec[select_template_view].astype(np.float32)

        select_template_rt = np.eye(4).astype(np.float32)
        select_template_rt[:3, :3] = select_template_r

        # transform select template shape pts into object frame
        select_template_partial_shape_in_obj_frame = \
            transform_coordinates_3d(template_shape_pts[select_template_view].T, \
            np.linalg.inv(select_template_rt)).T

        # transform query pts into object frame based on the pose of the select template view
        query_partial_shape_in_template_view = \
            transform_coordinates_3d(query_shape_pts.T, \
            np.linalg.inv(select_template_rt)).T

        return {
            'query_pts': points, 'query_rgb': rgb, 'query_choose': choose, \
            'query_nocs': nocs, 'query_complete_nocs': query_complete_nocs, \
            'query_partial_shape': query_shape_pts, \
            'template_view_score': softmax_template_orientation_errors, \
            'all_template_feat': template_feat_with_class_token, \
            'position_encoding': template_choose, \
            'select_template_feat': template_feat_no_class_token[select_template_view], \
            'all_template_partial_shape': template_shape_pts, \
            'select_template_partial_shape': template_shape_pts[select_template_view], \
            'select_template_nocs': select_template_nocs, \
            'template_complete_nocs': template_complete_nocs, \
            'r': gt_r, 't': gt_t, 's': obj_scale, \
            'template_r': template_obj_pose[select_template_view][:3, :3], \
            'template_t': template_obj_pose[select_template_view][:3, 3].flatten(), \
            'template_scale': template_obj_scale, \
            'is_symmetry': float(is_symmetry), \
            'sampled_template_view_score': sampled_softmax_template_orientation_errors, \
            'sampled_template_feat': template_feat_with_class_token[sampled_template_views], \
            'sampled_template_partial_shape': template_shape_pts[sampled_template_views], \
            'sampled_position_encoding': template_choose[sampled_template_views], \
            'hard_sampled_template_view_score': sampled_softmax_hard_template_orientation_errors, \
            'hard_sampled_template_feat': template_feat_with_class_token[sampled_hard_template_views], \
            'hard_sampled_template_partial_shape': template_shape_pts[sampled_hard_template_views], \
            'hard_sampled_position_encoding': template_choose[sampled_hard_template_views], \
            'top_1_view_choose': np.array([0]).astype(np.int64), \
            'view_error': np.array(-sampled_orientation_errors / 10.0).copy(), \
            'hard_view_error': np.array(-sampled_hard_orientation_errors / 10.0).copy(), \
            'select_template_shape_in_obj_frame': select_template_partial_shape_in_obj_frame, \
            'query_shape_in_template_view': query_partial_shape_in_template_view
        }

if __name__ == '__main__':
    dataset = DINOPoseDataset('bop', 'train', 'data/data/BOP', 1024, 224)
    
    # data = dataset.__getitem__(15000)
    # print(data['sampled_template_partial_shape'].shape)

    for i in range(176200, dataset.__len__()):
        dataset.__getitem__(i)

    # # check query_pts, query_nocs, query_pose
    # sRT = np.eye(4).astype(np.float32)
    # sRT[:3, :3] = data['r'] * (data['s'] / 1000.0)
    # sRT[:3, 3] = data['t'] / 1000.0

    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(data['query_pts'] / 1000.0)
    # o3d.io.write_point_cloud('depth_pts.ply', pcd)

    # transformed_nocs = transform_coordinates_3d(data['query_nocs'].T, sRT).T
    # pcd2 = o3d.geometry.PointCloud()
    # pcd2.points = o3d.utility.Vector3dVector(transformed_nocs)
    # o3d.io.write_point_cloud('nocs_pts.ply', pcd2)