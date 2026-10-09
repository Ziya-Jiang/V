#!/usr/bin/env python3
"""
简化版VGGT推理服务
"""

import os
import sys
import time
import logging
import threading
import numpy as np
import torch
import cv2
from flask import Flask, request, jsonify
import requests

# 添加项目路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from visions.vggt.vggt.models.vggt import VGGT
from visions.vggt.vggt.utils.load_fn import load_and_preprocess_images
from visions.vggt.vggt.utils.pose_enc import pose_encoding_to_extri_intri
from utils.image_utils import save_depth_npy, save_rgb_png, turbo_cmap

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class VGGTService:
    """简化版VGGT推理服务"""
    
    def __init__(self, device="cuda:1"):
        self.device = torch.device(device)
        self.model = None
        self.dtype = torch.bfloat16 if torch.cuda.get_device_capability()[0] >= 8 else torch.float16
        self.is_loaded = False
        self.lock = threading.Lock()
        
    def load_model(self):
        """加载模型"""
        with self.lock:
            if self.is_loaded:
                return True
            
            logger.info("加载VGGT模型...")
            start_time = time.time()
            self.model = VGGT.from_pretrained("facebook/VGGT-1B").to(self.device)
            load_time = time.time() - start_time
            self.is_loaded = True
            logger.info(f"模型加载完成，耗时: {load_time:.2f}秒")
            return True
    
    def inference(self, image_paths, output_dir):
        """执行推理"""
        if not self.is_loaded:
            raise RuntimeError("模型未加载")
        
        with self.lock:
            # 加载图像
            images = load_and_preprocess_images(image_paths).to(self.device)
            
            # 推理
            with torch.no_grad():
                with torch.cuda.amp.autocast(dtype=self.dtype):
                    start_time = time.time()
                    predictions = self.model(images)
                    inference_time = time.time() - start_time
            
            logger.info(f"推理完成，耗时: {inference_time:.2f}秒")
            
            # 处理结果
            return self._process_results(predictions, images, output_dir)
    
    def _process_results(self, predictions, images, output_dir):
        """处理推理结果"""
        # 提取相机参数
        extrinsic, intrinsic = pose_encoding_to_extri_intri(predictions["pose_enc"], images.shape[-2:])
        extrinsic = extrinsic.squeeze(0).cpu().numpy()
        vggt_intrinsic = intrinsic.squeeze(0).cpu().numpy()[0]
        
        # 计算深度
        vggt_baseline = np.linalg.norm(extrinsic[0, :, 3] - extrinsic[1, :, 3])
        vggt_depth_rescale = 0.120 / vggt_baseline
        
        depth_map = predictions["depth"].detach().cpu().numpy().squeeze(0)[0, ..., 0] * vggt_depth_rescale
        depth_confidence_map = predictions["depth_conf"].detach().cpu().numpy().squeeze(0)[0]
        
        # 调整尺寸
        target_width, target_height = 1280, 720
        depth_confidence_map = cv2.resize(depth_confidence_map, (target_width, target_height), interpolation=cv2.INTER_NEAREST)
        vggt_depth_map_ext = cv2.resize(depth_map, (target_width, target_height), interpolation=cv2.INTER_NEAREST)
        
        # 保存文件
        os.makedirs(output_dir, exist_ok=True)
        save_depth_npy(vggt_depth_map_ext, os.path.join(output_dir, "vggt_depth_ext.npy"))
        save_rgb_png(turbo_cmap(vggt_depth_map_ext), os.path.join(output_dir, "vggt_depth_ext.png"))
        
        # 计算内参矩阵
        K = np.array([
            [vggt_intrinsic[0,0] * target_width / 518, 0, vggt_intrinsic[0,2] * target_width / 518],
            [0, vggt_intrinsic[1,1] * target_width / 518, vggt_intrinsic[1,2] * target_width / 518],
            [0, 0, 1]
        ])
        np.save(os.path.join(output_dir, "vggt_K.npy"), K)
        
        # 计算深度掩码
        confidence_threshold = np.percentile(depth_confidence_map.flatten(), 10)
        depth_mask = (depth_confidence_map >= confidence_threshold) & (depth_confidence_map > 0.1)
        
        return {
            'depth_map': vggt_depth_map_ext.tolist(),
            'depth_confidence_map': depth_confidence_map.tolist(),
            'depth_mask': depth_mask.tolist(),
            'intrinsic_matrix': K.tolist(),
            'vggt_intrinsic': vggt_intrinsic.tolist()
        }

# 全局服务实例
vggt_service = None
app = Flask(__name__)

@app.route('/health')
def health():
    """健康检查"""
    return jsonify({
        'status': 'ok',
        'model_loaded': vggt_service.is_loaded if vggt_service else False
    })

@app.route('/inference', methods=['POST'])
def inference():
    """推理接口"""
    try:
        data = request.get_json()
        image_paths = data['image_paths']
        output_dir = data.get('output_dir', './output')
        
        # 检查文件存在
        for path in image_paths:
            if not os.path.exists(path):
                return jsonify({'success': False, 'error': f'文件不存在: {path}'}), 400
        
        # 执行推理
        results = vggt_service.inference(image_paths, output_dir)
        return jsonify({'success': True, 'results': results})
        
    except Exception as e:
        logger.error(f"推理失败: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

class VGGTClient:
    """简化版客户端"""
    
    def __init__(self, url="http://localhost:5001"):
        self.url = url.rstrip('/')
    
    def health_check(self):
        """健康检查"""
        try:
            response = requests.get(f"{self.url}/health", timeout=5)
            return response.json()
        except:
            return {'status': 'error'}
    
    def inference(self, image_paths, output_dir):
        """推理请求"""
        data = {'image_paths': image_paths, 'output_dir': output_dir}
        response = requests.post(f"{self.url}/inference", json=data, timeout=120)
        return response.json()

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=['server', 'client'], default='server')
    parser.add_argument("--device", default="cuda:1")
    parser.add_argument("--port", type=int, default=5001)
    args = parser.parse_args()
    
    if args.mode == 'server':
        global vggt_service
        vggt_service = VGGTService(device=args.device)
        
        # 自动加载模型
        logger.info("正在加载模型...")
        vggt_service.load_model()
        
        # 启动服务
        logger.info(f"启动服务，端口: {args.port}")
        app.run(host='0.0.0.0', port=args.port, threaded=True)
    
    elif args.mode == 'client':
        # 测试客户端
        client = VGGTClient()
        health = client.health_check()
        print(f"服务状态: {health}")

if __name__ == "__main__":
    main()
