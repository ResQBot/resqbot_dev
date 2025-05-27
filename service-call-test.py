import sys
import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import  Node


class TestCaller(Node):

    def __init__(self):
        super().__init__('test_caller')
        self.cli = self.create_client()