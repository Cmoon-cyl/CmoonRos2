#!/usr/bin/env python
# coding: UTF-8
# author: Cmoon
# date: 2024/1/27$ 下午8:32$

from pathlib import Path
import sys
from typing import Any, Dict, Generator, List, Optional, Union, Sequence

import rclpy
from rclpy.node import Node

CWD = Path(__file__).resolve().parent
sys.path.append(str(CWD))
from CmoonVision import Yolo


class Detector(Node):
    def __init__(self, name):
        super().__init__(name)
        self.yolo = Yolo(CWD / "weights" / "yolov8n.pt")

    def detect(self):
        source = CWD / "images" / "bus.jpg"
        results = self.yolo.predict(source, show=True, stream=False, conf=0.25, iou=0.45, imgsz=640, classes=None)
        print(results.objects[0].name)
        return results


def main():
    rclpy.init()
    node = Detector("detector")
    try:
        node.detect()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
