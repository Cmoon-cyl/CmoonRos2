#!/usr/bin/env python
# coding: UTF-8 
# author: Cmoon
# date: 2024/4/27 下午11:54

from cmoonvision import Yolo
from pathlib import Path


def main():
    # 初始化
    weights = Path(__file__).resolve().parent.parent / "resource" / "weights" / "yolov8n-seg.pt"
    print(weights)
    detector = Yolo(weights)

    # 摄像头检测
    results = detector(source=0, conf=0.25, iou=0.7, show=False, save=False, stream=True)  # stream设置为True，检测视频流
    # 图片检测
    # img_path = Path(__file__).resolve().parent.parent / "cmoonvision" / "images" / "bus.jpg"
    # results = detector(source=img_path, conf=0.25, iou=0.7, show=True, save=False, stream=False)
    # 筛选类别检测
    # results = detector(source=0, conf=0.25, iou=0.7, show=False, save=False, stream=True,
    #                    classes=["person", "chair"])  # 设置classes,只检测人和椅子

    # 结果处理示例
    # for result in results:
    #     根据检测结果进行其他操作
    # print(result)

    # 结果显示示例
    for result in results:
        # 显示所有检测结果
        result.show_img0(stream=True)  # 显示原图
        result.show_img(stream=True)  # 显示带框图
        result.show_mask(stream=True)  # 显示mask

        # 分别显示每个物体
        for obj in result:
            obj.show_img0(stream=True)  # 显示原图
            obj.show_img(stream=True)  # 显示带框图
            obj.show_mask(stream=True)  # 显示mask

    # # 复杂处理示例
    # for result in results:
    #     # 判断某个物体是否在检测结果中
    #     if "person" in result:
    #         print("检测到了person")
    #
    #     # 判断这一帧图像是否检测到物体
    #     if len(result):
    #         print("检测到了物体")
    #
    #     # 获取两个检测结果中心点的像素距离
    #     distance = result[0] - result[1]
    #     print(distance)
    #
    #     # 快速判断物体位置关系(x坐标)
    #     if result[0] < result[1]:
    #         print("第一个物体在第二个物体左边")
    #     elif result[0] > result[1]:
    #         print("第一个物体在第二个物体右边")
    #
    #     # 判断两个物体名称是否相同
    #     if result[0] == result[1]:
    #         print("两个物体名称相同")
    #
    #     # 对每个物体进行操作
    #     for obj in result:
    #         # 判断物体中心是否在指定区域
    #         if obj.center[0] > 100 and obj.center[0] < 200 and obj.center[1] > 100 and obj.center[1] < 200:
    #             print("物体在指定区域内")
    #         # 获取物体name, box, mask等信息
    #         print(obj.name, obj.box, obj.mask)


if __name__ == '__main__':
    main()
