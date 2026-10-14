import time
import unittest
import rclpy


class RosTest(unittest.TestCase):
    def wait(self, predicate, timeout=3):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.02)
        assert predicate(), 'ROS operation did not finish within the test deadline'

    def future(self, future, timeout=3):
        self.wait(future.done,timeout)
        return future.result()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.01)

