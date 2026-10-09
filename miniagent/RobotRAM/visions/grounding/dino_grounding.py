import sys
import os
from pathlib import Path

cur_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(cur_path)
robotram_root = Path(__file__).resolve().parents[2]
sam2_source_root = robotram_root / "3rdparty" / "sam2"
sys.path.insert(0, str(sam2_source_root))

import time
import math
import functools
import numpy as np
import json
import cv2
from PIL import Image
import logging

import torch
from torchvision.ops import box_convert
import supervision as sv

from .base_grounding import BaseGrounding
from grounding_dino.groundingdino.util.inference import load_model, load_image, predict, load_data
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor


_SAM2_CHECKPOINT = "./Grounded-SAM-2/checkpoints/sam2.1_hiera_large.pt"
_SAM2_MODEL_CONFIG = "../../3rdparty/sam2/sam2/configs/sam2.1/sam2.1_hiera_l.yaml"
_GROUNDING_DINO_CONFIG = "./grounding_dino/groundingdino/config/GroundingDINO_SwinT_OGC.py"
_GROUNDING_DINO_CHECKPOINT = "./Grounded-SAM-2/gdino_checkpoints/groundingdino_swint_ogc.pth"

BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25


class DinoGrounding(BaseGrounding):
    def __init__(self, config_file=None, device=None):
        cur_path = os.path.dirname(os.path.abspath(__file__))

        SAM2_CHECKPOINT = os.path.join(cur_path, _SAM2_CHECKPOINT)
        SAM2_MODEL_CONFIG = os.path.join(cur_path, _SAM2_MODEL_CONFIG)
        GROUNDING_DINO_CONFIG = os.path.join(cur_path, _GROUNDING_DINO_CONFIG)
        GROUNDING_DINO_CHECKPOINT = os.path.join(cur_path, _GROUNDING_DINO_CHECKPOINT)

        # get abs path
        self.sam2_checkpoint = os.path.abspath(SAM2_CHECKPOINT)
        self.model_cfg = os.path.abspath(SAM2_MODEL_CONFIG)
        self.grounding_dino_config = os.path.abspath(GROUNDING_DINO_CONFIG)
        self.grounding_dino_checkpoint = os.path.abspath(GROUNDING_DINO_CHECKPOINT)


        if device is not None:
            self.device = device
        else:
            self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.box_threshold = BOX_THRESHOLD
        self.text_threshold = TEXT_THRESHOLD

        if config_file is not None:
            self.read_config(config_file)
            logging.info("==> Grounding configuration loaded from %s", config_file)

        # build SAM2 image predictor
        sam2_checkpoint = SAM2_CHECKPOINT
        model_cfg = SAM2_MODEL_CONFIG
        self.sam2_model = build_sam2('/' + self.model_cfg, self.sam2_checkpoint, device=self.device)
        self.sam2_predictor = SAM2ImagePredictor(self.sam2_model)
        self._prepared_image_source = None
        self._prepared_dino_image = None

        # build grounding dino model
        self.grounding_model = load_model(
            model_config_path=self.grounding_dino_config, 
            model_checkpoint_path=self.grounding_dino_checkpoint,
            device=self.device
        )

    def restore_config(self, config_file):
        """
        Restore the configuration file
        :param config_file: path to the configuration file
        :return: None
        """
        config_paras = {
            "sam2_checkpoint": self.sam2_checkpoint,
            "model_cfg": self.model_cfg,
            "grounding_dino_config": self.grounding_dino_config,
            "grounding_dino_checkpoint": self.grounding_dino_checkpoint,
            "box_threshold": self.box_threshold,
            "text_threshold": self.text_threshold,
        }
        # write config_paras to a json file
        with open(config_file, 'w') as f:
            json.dump(config_paras, f, indent=4)
        logging.info("Configuration saved to %s", config_file)

    def read_config(self, config_file):
        with open(config_file, 'r') as f:
            config_paras = json.load(f)
        self.sam2_checkpoint = config_paras["sam2_checkpoint"]
        self.model_cfg = config_paras["model_cfg"]
        self.grounding_dino_config = config_paras["grounding_dino_config"]
        self.grounding_dino_checkpoint = config_paras["grounding_dino_checkpoint"]
        self.box_threshold = config_paras["box_threshold"]
        self.text_threshold = config_paras["text_threshold"]
        return 0
    
    def reset(self):
        """
        Reset the grounding model
        :return: None
        """
        self.sam2_model = build_sam2('/' + self.model_cfg, self.sam2_checkpoint, device=self.device)
        self.sam2_predictor = SAM2ImagePredictor(self.sam2_model)
        self._prepared_image_source = None
        self._prepared_dino_image = None

        # build grounding dino model
        self.grounding_model = load_model(
            model_config_path=self.grounding_dino_config, 
            model_checkpoint_path=self.grounding_dino_checkpoint,
            device=self.device
        )
        logging.info("==> Grounding model reset successfully")
        return 0

    def prepare_image(self, image_rgb):
        """Encode one RGB frame once for all subsequent SAM2 box prompts."""
        image_source, image = load_data(image_rgb)
        self.sam2_predictor.set_image(image_source)
        self._prepared_image_source = image_source
        self._prepared_dino_image = image

    def grounding(self, image_rgb, text_prompt):
        """
        Grounding the object in the image
        :param image_rgb: image in RGB format
        :param text_prompt: text prompt for grounding
        :return: bounding boxes and labels
            detections.xyxy: (n, 4)
            detections.mask: (n, h, w)
            detections.class_id: (n, )
        """
        self.prepare_image(image_rgb)
        return self.grounding_prepared(text_prompt)

    def grounding_prepared(self, text_prompt):
        """Ground one text prompt using the currently prepared RGB frame."""
        if self._prepared_image_source is None or self._prepared_dino_image is None:
            raise RuntimeError("prepare_image() must run before grounding_prepared()")

        # setup the input image and text prompt for SAM 2 and Grounding DINO
        # VERY important: text queries need to be lowercased + end with a dot

        # initial
        text = text_prompt

        if text[-1] != ".":
            text += "."
        
        image_source = self._prepared_image_source
        image = self._prepared_dino_image

        # use bfloat16 for dino
        with torch.autocast(device_type=self.device, dtype=torch.bfloat16):
            if torch.cuda.get_device_properties(0).major >= 8:
                torch.backends.cuda.matmul.allow_tf32 = True
                torch.backends.cudnn.allow_tf32 = True
        
        print('sam text prompt: ', text)
        # dino predict boxes and labels
        boxes, confidences, labels = predict(
            model=self.grounding_model,
            image=image,
            caption=text,
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold,
        )
        if confidences.numel() > 1:
            order = torch.argsort(confidences, descending=True)
            boxes = boxes[order]
            confidences = confidences[order]
            labels = [labels[index] for index in order.cpu().tolist()]

        # process the box prompt for SAM 2
        h, w, _ = image_source.shape
        boxes = boxes * torch.Tensor([w, h, w, h])
        input_boxes = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy()

        masks, scores, logits = self.sam2_predictor.predict(
            point_coords=None,
            point_labels=None,
            box=input_boxes,
            multimask_output=False,
        )

        """
        Post-process the output of the model to get the masks, scores, and logits for visualization
        """
        # convert the shape to (n, H, W)
        if masks.ndim == 4:
            masks = masks.squeeze(1)


        confidence_values = (
            confidences.detach().cpu().numpy().astype(np.float32)
        )
        confidences = confidence_values.tolist()
        class_names = labels

        class_ids = np.array(list(range(len(class_names))))

        labels = [
            f"{class_name} {confidence:.2f}"
            for class_name, confidence
            in zip(class_names, confidences)
        ]
        print('LABLES', labels)

        detections = sv.Detections(
            xyxy=input_boxes,  # (n, 4)
            mask=masks.astype(bool),  # (n, h, w)
            confidence=confidence_values,  # (n,)
            class_id=class_ids,  # (n,)
        )

        return detections

    
    
