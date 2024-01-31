#!/usr/bin/env python
# coding: UTF-8 
# author: Cmoon
# date: 2023/3/11 下午2:20

from dataclasses import dataclass, field
import json
import time
from typing import List, Tuple, Iterator, Sequence

import cv2
import numpy as np
from ultralytics.engine.results import Results


class Utils:
    @staticmethod
    def timer(f):
        """计时函数运行时间"""

        def timeit(*args, **kwargs):
            print("---------开始计时---------")
            start = time.time()
            ret = f(*args, **kwargs)
            print(f"----运行时间：{(time.time() - start):.5f}秒----")
            return ret

        return timeit

    @staticmethod
    def xyxy2cnt(xyxy: List[int]) -> List[int]:
        """
        xyxy坐标转中心点坐标

        :param xyxy: [x,y,x,y]
        :return: [x,y]
        """
        center = [int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)]
        return center

    @staticmethod
    def show_image(name, img):
        try:
            cv2.imshow(name, img)
            if cv2.waitKey(0) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
        except KeyboardInterrupt:
            cv2.destroyAllWindows()

    @staticmethod
    def show_stream(name, img):
        try:
            cv2.imshow(name, img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
        except KeyboardInterrupt:
            cv2.destroyAllWindows()

    @staticmethod
    def plot_masks(img0, mask):
        img = img0.copy()
        if type(mask) is list:
            for result in mask:
                msk1 = result.mask.astype(np.bool_)
                color = np.array([0, 0, 255], dtype=np.uint8)
                img[msk1] = img[msk1] * 0.7 + color * 0.3

        else:
            msk1 = mask.astype(np.bool_)
            color = np.array([0, 0, 255], dtype=np.uint8)
            img[msk1] = img[msk1] * 0.7 + color * 0.3

        return img

    @staticmethod
    def calc_roi(size, roi):
        size = [pix for pix in size for _ in range(2)]
        roi_range = [pixel * propotion for pixel, propotion in zip(size, roi)]
        roi_point = [[int(roi_range[0]), int(roi_range[2])], [int(roi_range[1]), int(roi_range[3])]]
        return roi_range, roi_point

    @staticmethod
    def in_roi(point: List[int], size: List[int], roi: Sequence[float]) -> bool:
        """
        判断物品中心点是否在设定的画面范围内

        :param xs: 一张图像上检测到的物品中心坐标
        :param range: 画面中心多大范围内的检测结果被采用
        :return:bool
        """
        roi_range, _ = Utils.calc_roi(size, roi)
        return roi_range[0] <= point[0] <= roi_range[1] and roi_range[2] <= point[1] <= roi_range[3]


@dataclass
class YoloObject:
    """
    单个物体检测结果数据类型

    :param _name: 标签名称
    :param _box: 矩形框左上右下xyxy坐标
    :param _center: 矩形框中心点坐标
    :param _conf: 置信度
    :param _id: 第几个物体（track时启用）
    """

    raw_data: Results = field(init=True)
    _json_data: str = field(init=False, repr=False, default=None)
    _dict_data: dict = field(init=False, repr=False, default=None)
    _name: str = field(init=False, repr=False, default=None)
    _box: List[float] = field(init=False, repr=False, default=None)
    _center: List[int] = field(init=False, repr=False, default=None)
    _conf: float = field(init=False, repr=False, default=None)
    _id: int = field(init=False, repr=False, default=None)
    _img0: np.ndarray = field(init=False, repr=False, default=None)
    _img: np.ndarray = field(init=False, repr=False, default=None)
    _mask: np.ndarray = field(init=False, repr=False, default=None)
    _points: List[Tuple[float, float]] = field(init=False, repr=False, default=None)

    @property
    def json_data(self) -> str:
        if self._json_data is None:
            self._json_data = self.raw_data.tojson()
        return self._json_data

    @property
    def dict_data(self) -> dict:
        if self._dict_data is None:
            self._dict_data = json.loads(self.json_data)[0]
        return self._dict_data

    @property
    def name(self) -> str:
        if self._name is None:
            self._name = self.dict_data["name"]
        return self._name

    @property
    def box(self) -> List[float]:
        if self._box is None:
            self._box = [self.dict_data["box"].get(key) for key in ["x1", "y1", "x2", "y2"]]
        return self._box

    @property
    def center(self) -> List[int]:
        if self._center is None:
            self._center = [int((self.box[0] + self.box[2]) / 2), int((self.box[1] + self.box[3]) / 2)]
        return self._center

    @property
    def conf(self) -> float:
        if self._conf is None:
            self._conf = self.dict_data["confidence"]
        return self._conf

    @property
    def id(self) -> int:
        if self._id is None:
            self._id = self.dict_data["tracker_id"] if "tracker_id" in self.dict_data.keys() else 1
        return self._id

    @property
    def img0(self) -> np.ndarray:
        if self._img0 is None:
            self._img0 = self.raw_data.orig_img
        return self._img0

    @property
    def img(self) -> np.ndarray:
        if self._img is None:
            self._img = self.raw_data.plot()
        return self._img

    @property
    def mask(self) -> np.ndarray:
        if self._mask is None:
            segment = self.dict_data["segments"] if "segments" in self.dict_data.keys() else None
            if segment is None:
                raise ValueError("No segment data. Ensure using a segment model.")
            points = np.array(list(zip(segment['x'], segment['y'])), dtype=np.int32)
            self._mask = np.zeros((480, 640), dtype=np.uint8)
            cv2.fillPoly(self._mask, [points.reshape((-1, 1, 2))], color=(255, 255, 255))
        return self._mask

    @property
    def points(self) -> List[Tuple[float, float]]:
        if self._points is None:
            keypoints = self.dict_data["keypoints"] if "keypoints" in self.dict_data.keys() else None
            if keypoints is None:
                raise ValueError("No pose data. Ensure using a pose model.")
            self._points = list(zip(keypoints['x'], keypoints['y']))
        return self._points

    def plot(self, stream=False):
        if stream:
            Utils.show_stream("result", self.img)
        else:
            Utils.show_image("result", self.img)

    def plot_mask(self, stream=False):
        if stream:
            Utils.show_stream("result", self.mask)
        else:
            Utils.show_image("result", self.mask)

    def __sub__(self, other):
        if not isinstance(other, YoloObject):
            raise TypeError("Subtraction can only be performed between two YoloObject instances.")
        return np.sqrt((self.center[0] - other.center[0]) ** 2 + (self.center[1] - other.center[1]) ** 2)

    def __lt__(self, other):
        if not isinstance(other, YoloObject):
            raise TypeError("'<' not supported between instances of 'YoloObject' and other types.")
        return self.center[0] < other.center[0]

    def __eq__(self, other):
        if not isinstance(other, YoloObject):
            return False
        return self.name == other.name

    def __str__(self):
        return f"YoloResult({self.json_data})"

    def __repr__(self):
        return self.__str__()


@dataclass
class YoloResult:
    """
    单帧检测结果数据类型
    :param raw_result: 原始检测结果
    :param timestamp: 时间戳
    :param _json_result: json格式的检测结果
    :param _img0: 原图
    :param _img: 检测结果图
    :param _img_shape: 原图大小
    :param _objects: YoloObject列表
    """
    raw_result: Results = field(init=True, repr=False)
    timestamp: float = field(init=False)
    _json_result: str = field(init=False, default=None)
    _img0: np.ndarray = field(init=False, repr=False, default=None)
    _img: np.ndarray = field(init=False, repr=False, default=None)
    _img_shape: Tuple[int, int] = field(init=False, repr=False, default=None)
    _objects: List[YoloObject] = field(init=False, repr=False, default=None)

    @property
    def json_result(self) -> str:
        if self._json_result is None:
            self._json_result = self.raw_result.tojson()
        return self._json_result

    @property
    def img0(self) -> np.ndarray:
        if self._img0 is None:
            self._img0 = self.raw_result.orig_img
        return self._img0

    @property
    def img(self) -> np.ndarray:
        if self._img is None:
            self._img = self.raw_result.plot()
        return self._img

    @property
    def img_shape(self) -> Tuple[int, int]:
        if self._img_shape is None:
            self._img_shape = self.raw_result.orig_shape[::-1]
        return self._img_shape

    @property
    def objects(self) -> List[YoloObject]:
        if self._objects is None:
            self._objects = [YoloObject(result) for result in self.raw_result]
        return self._objects

    def plot(self, stream=False):
        if stream:
            Utils.show_stream("result", self.img)
        else:
            Utils.show_image("result", self.img)

    def __post_init__(self):
        self.timestamp = time.time()

    def __iter__(self) -> Iterator[YoloObject]:
        return iter(self.objects)

    def __str__(self):
        return f"YoloResult({self.json_result}, {self.timestamp})"

    def __repr__(self):
        return self.__str__()
