#!/usr/bin/env python3
# coding: UTF-8 
# author: Cmoon
# date: 2024/1/26 下午2:59

import argparse
from pathlib import Path
import sys
from typing import Any, Dict, Generator, List

import torch
from ultralytics import YOLO
from ultralytics.engine.results import Results

CWD = Path(__file__).resolve().parent
sys.path.append(str(CWD))
from cmoon_utils import YoloResult


class Yolo:
    def __init__(self, weights: Path):
        """
        检测器
        :param weights:.pt文件路径
        """
        self.weights: Path = weights
        self.device: torch.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model: YOLO = YOLO(weights).to(self.device)
        self.names: Dict = self.model.names
        self.results: YoloResult | Generator | None = None
        self.stream: bool = False
        print(f"Model {weights.name} running on {self.model.device}\nnames:{self.names}")

    def process_result(self, results: List[Results]) -> YoloResult | Generator:
        """
        处理检测结果,并返回封装的YoloResult
        :param results: List[Results]
        :return: YoloResult | Generator[YoloResult, None, YoloResult]
        """

        if self.stream:
            return (YoloResult(result.cpu()) for result in results if len(result))
        else:
            for result in results:
                if len(result):
                    self.results = YoloResult(result.cpu())
            return self.results

    def predict(self, source: Any, conf: float = 0.25, iou: float = 0.7, imgsz: int = 640,
                classes: List[str] | List[int] = None,
                stream: bool = False, show: bool = False, save: bool = False, **kwargs) -> YoloResult | Generator[
        YoloResult, None, YoloResult]:
        """
        检测函数
        :param source: 检测源
        :param conf: 置信度阈值
        :param iou: iou阈值
        :param imgsz: 图片大小
        :param classes: 检测类别
        :param stream: 是否实时检测
        :param show: 是否显示检测结果
        :param save: 是否保存检测结果
        :return: YoloResult
        """
        self.stream = stream
        if classes is not None and isinstance(classes[0], str):
            classes = [list(self.names.values()).index(name) for name in classes]
        results = self.model(source=source, stream=self.stream, show=show, save=save, conf=conf, iou=iou, imgsz=imgsz,
                             classes=classes, **kwargs)
        return self.process_result(results)

    def __call__(self, *args, **kwargs):
        return self.predict(*args, **kwargs)


def main():
    parse = argparse.ArgumentParser()
    parse.add_argument('--weights', type=str, default=CWD / "weights" / "yolov8n.pt", help='weights path')
    parse.add_argument('--source', type=str, default='0', help='source')
    parse.add_argument('--conf', type=float, default=0.25, help='confidence threshold')
    parse.add_argument('--iou', type=float, default=0.7, help='iou threshold')
    parse.add_argument('--imgsz', type=int, default=640, help='image size')
    parse.add_argument('--device', default=torch.device('cuda' if torch.cuda.is_available() else 'cpu'),
                       help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parse.add_argument('--stream', action='store_true', help='stream results')
    parse.add_argument('--noshow', action='store_true', help='do not show results')
    parse.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parse.add_argument('--classes', nargs='+', type=int, help='filter by class')
    opt = parse.parse_args()

    detector = Yolo(opt.weights)
    source = opt.source
    # source = CWD / "images" / "bus.jpg"
    results = detector.predict(source=source, conf=opt.conf, iou=opt.iou, imgsz=opt.imgsz, device=opt.device,
                               show=not opt.noshow, save=not opt.nosave, stream=opt.stream, classes=opt.classes)
    print(results)


if __name__ == '__main__':
    main()
