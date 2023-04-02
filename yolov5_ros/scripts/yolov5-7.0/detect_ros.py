#! /home/sj/anaconda3/envs/yolov5_strongsort/bin/python
# coding=utf-8

import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np

from yolov5_ros.msg import Detection2D, Detection2DArray

# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
Run inference on images, videos, directories, streams, etc.

Usage - sources:
    $ python path/to/detect.py --weights yolov5s.pt --source 0              # webcam
                                                             img.jpg        # image
                                                             vid.mp4        # video
                                                             path/          # directory
                                                             path/*.jpg     # glob
                                                             'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                                             'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python path/to/detect.py --weights yolov5s.pt                 # PyTorch
                                         yolov5s.torchscript        # TorchScript
                                         yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                         yolov5s.xml                # OpenVINO
                                         yolov5s.engine             # TensorRT
                                         yolov5s.mlmodel            # CoreML (macOS-only)
                                         yolov5s_saved_model        # TensorFlow SavedModel
                                         yolov5s.pb                 # TensorFlow GraphDef
                                         yolov5s.tflite             # TensorFlow Lite
                                         yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
"""

import argparse
import os
import platform
import sys
from pathlib import Path

import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args,  scale_boxes, scale_segments, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.segment.general import masks2segments, process_mask
from utils.torch_utils import select_device, smart_inference_mode, time_sync
from utils.augmentations import letterbox

# detect class
class detector:
    def __init__(self):
        self.weights = ROOT / 'checkpoints/yolov5l-seg.pt'  # model.pt path(s)
        self.data=ROOT / 'data/coco128.yaml'  # dataset.yaml path
        self.imgsz=(640, 640)  # inference size (height, width)
        self.conf_thres=0.25  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=1000  # maximum detections per image
        self.device=''  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.view_img=False  # show results
        self.save_txt=False  # save results to *.txt
        self.save_conf=False  # save confidences in --save-txt labels
        self.save_crop=False  # save cropped prediction boxes

        self.save_img = False
        self.classes= (0,1,2,3,5,7)   # filter by class: --class 0, or --class 0 2 3
        self.agnostic_nms=False  # class-agnostic NMS
        self.augment=False  # augmented inference
        self.visualize=False  # visualize features
        self.update=False  # update all models
        self.project=ROOT / 'runs/detect'  # save results to project/name
        self.name='exp'  # save results to project/name
        self.exist_ok=False  # existing project/name ok, do not increment
        self.line_thickness=3  # bounding box thickness (pixels)
        self.hide_labels=False  # hide labels
        self.hide_conf=False  # hide confidences
        self.half=False  # use FP16 half-precision inference
        self.dnn=False  # use OpenCV DNN for ONNX inference

        self.retina_masks = False

        # Load model
        self.device = select_device(self.device)
        self.model = DetectMultiBackend(self.weights, device=self.device, dnn=self.dnn, data=self.data, fp16=self.half)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size(self.imgsz, s=self.stride)  # check image size

        # call back
        rospy.Subscriber("/camera/image_color/compressed", CompressedImage, self.compressedimage_callback, queue_size=1)
        # rospy.Subscriber("/gige_cam/image_raw/compressed", CompressedImage, self.compressedimage_callback, queue_size=1)
        self.object_pub = rospy.Publisher("~objects", Detection2DArray, queue_size=1)

        self.bs = 1  # batch_size
        # Run inference
        self.model.warmup(imgsz=(1 if self.pt else self.bs, 3, *self.imgsz))  # warmup
        # seen, windows, dt = 0, [], [0.0, 0.0, 0.0]

    @smart_inference_mode()
    def run(self, im, im0s, raw_img):
        seen, windows, dt = 0, [], [0.0, 0.0, 0.0]
        # for path, im, im0s, vid_cap, s in dataset:
        t1 = time_sync()
        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        t2 = time_sync()
        dt[0] += t2 - t1

        # Inference
        pred, proto = self.model(im, augment=self.augment, visualize=self.visualize)[:2]
        t3 = time_sync()
        dt[1] += t3 - t2

        # NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det, nm=32)
        dt[2] += time_sync() - t3

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        # object detection result
        objArray = Detection2DArray()
        objArray.detections = []
        objArray.header = self.header
        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1
            im0 = im0s.copy()
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            imc = im0.copy() if self.save_crop else im0  # for save_crop
            annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                masks = process_mask(proto[i], det[:, 6:], det[:, :4], im.shape[2:], upsample=True)  # HWC
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                segments = reversed(masks2segments(masks))
                segments = [scale_segments(im.shape[2:], x, im0.shape, normalize=True) for x in segments]
                
                # Mask plotting
                annotator.masks(masks,
                                colors=[colors(x, True) for x in det[:, 5]],
                                im_gpu=None if self.retina_masks else im[i])

                # Write results
                # for *xyxy, conf, cls in reversed(det):

                #     # if self.save_img or self.save_crop or self.view_img:  # Add bbox to image
                #     #     c = int(cls)  # integer class
                #     #     label = None if self.hide_labels else (self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                #     #     annotator.box_label(xyxy, label, color=colors(c, True))  

                #     objArray.detections.append(self.generate_obj(int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]), conf, self.names[int(cls)]))             
                for j, (*xyxy, conf, cls) in enumerate(reversed(det[:, :6])):
                    if self.save_img or self.save_crop or self.view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        label = None if self.hide_labels else (self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                        annotator.box_label(xyxy, label, color=colors(c, True))  
                    segj = segments[j].reshape(-1)  # (n,2) to (n*2)
                    # print(segj)
                    objArray.detections.append(self.generate_obj(int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]), conf, self.names[int(cls)], segj))
            # Stream results
            im0 = annotator.result()
            if self.view_img:
                duration = (dt[0] + dt[1] + dt[2]) * 1000 # ms

                duration_str = '%f' % duration
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(im0, 'duration ' + duration_str + ' ms',(10,50), font, 1,(0,0,255),2,cv2.LINE_AA)
                cv2.namedWindow('detected image',0)
                cv2.imshow("detected image", im0)
                cv2.waitKey(1)

        # objArray.compressed_source_img = raw_img

        self.object_pub.publish(objArray)
        # Print results
        # t = tuple(x / seen * 1E3 for x in dt)  # speeds per image
        # LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *self.imgsz)}' % t)
       
        if self.update:
            strip_optimizer(self.weights[0])  # update model (to fix SourceChangeWarning)

    def compressedimage_callback(self, image):

        np_arr = np.frombuffer(image.data, np.uint8)
        img0 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.header = image.header
        # Padded resize
        img = letterbox(img0, self.imgsz, stride=self.stride, auto=self.pt)[0]

        # Convert
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)

        self.run(img, img0, image)

    def generate_obj(self, x1, y1, x2, y2, score, name, seg):
        obj = Detection2D()
        obj.header = self.header
        # if self._is_compressed_img:
        # obj.compressed_source_img = msg
        # else:
        #     obj.source_img = msg

        obj.bbox.center.x = (x1 + x2) / 2
        obj.bbox.center.y = (y1 + y2) / 2
        obj.bbox.size_x = x2 - x1
        obj.bbox.size_y = y2 - y1

        obj.segments = seg

        # obj_hypothesis = ObjectHypothesisWithPose()
        obj.name = str(name)
        obj.score = float(score) 
        # obj.results.append(obj_hypothesis)

        return obj


def main():
    check_requirements(exclude=('tensorboard', 'thop'))
    det = detector()
    print("success!!!!!")
    rospy.spin() # 回调函数


if __name__ == "__main__":
    # opt = parse_opt()
    rospy.init_node('yolov5_ros')
    main()
