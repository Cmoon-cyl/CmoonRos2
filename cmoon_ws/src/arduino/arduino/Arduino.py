#!/usr/bin/env python
# coding: UTF-8 
# author: Cmoon
# date: 2023/10/29$ 下午2:11$
import os
import sys

sys.path.append(os.path.dirname(__file__))
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import json


class Arduino(Node):
    def __init__(self, com, name="arduino"):
        super().__init__(name)
        self.ser, self.ret = self.init_port(com, 9600)
        self._onreceive = self.create_subscription(String, '/arduino', self.onreceive, 10)
        self._order = {}
        self._last_order = {}
        self.get_logger().info("Arduino Ready")

    def init_port(self, com, bps):
        ret = True
        try:
            # 端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 com 等
            portx = com
            ser = serial.Serial(portx, bps)
            return ser, ret
        except Exception as e:
            print("---异常---：", e)

    def onreceive(self, msg):
        data = msg.data
        jsondata = json.loads(data)
        self._last_order = self._order
        self._order = jsondata
        self.send_flag = True
        self.get_logger().info(f"jsondata:{jsondata} received")
        self.send2arduino()

    @property
    def order(self):
        return self._order

    @order.setter
    def order(self, order):
        if type(order) is not dict:
            raise TypeError("order must be dict")
        self._order = order
        print(f'order set:{order}')

    @order.deleter
    def order(self):
        self._order = {}
        print('order deleted')

    def write_json(self, jsondata):
        self.write(json.dumps(jsondata).encode())

    def write(self, data):
        self.ser.write(data)
        print(f'write:{data} success')

    def send2arduino(self):
        if self._order and self._last_order != self._order:
            self.write_json(self._order)


def main():
    rclpy.init()
    node = Arduino("/dev/ttyUSB0")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
