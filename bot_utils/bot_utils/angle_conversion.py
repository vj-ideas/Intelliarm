#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from bot_msgs.srv import QuaternionToEuler, EulerToQuaternion
from tf_transformations import quaternion_from_euler,euler_from_quaternion

class AnglesConverter(Node):
    def __init__(self):
        super().__init__("angle_conversion_service_server")

        self.euler_to_quaternion = self.create_service(EulerToQuaternion, "euler_to_quaternion", self.euler_to_quaternionCallback)
        self.quaternion_to_euler = self.create_service(QuaternionToEuler, "quaternion_to_euler", self.quaternion_to_eulerCallback)

        self.get_logger().info("angle conversion Ready")

    def quaternion_to_eulerCallback(self, req, res):
        self.get_logger().info("Requested to convert quaternion: x=%f, y=%f, z=%f, w=%f" % (req.x, req.y, req.z, req.w))
        (res.roll, res.pitch, res.yaw) = euler_from_quaternion([req.x, req.y, req.z, req.w])
        self.get_logger().info("Converted to Euler angles: roll=%f, pitch=%f, yaw=%f" % (res.roll, res.pitch, res.yaw))
        return res

    def euler_to_quaternionCallback(self, req, res):
        self.get_logger().info("Requested to convert Euler angles: roll=%f, pitch=%f, yaw=%f" % (req.roll, req.pitch, req.yaw))
        (res.x, res.y, res.z, res.w) = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        self.get_logger().info("Converted to quaternion: x=%f, y=%f, z=%f, w=%f" % (res.x, res.y, res.z, res.w))
        return res



def main():
    rclpy.init()
    angle_conversion = AnglesConverter()
    rclpy.spin(angle_conversion)
    angle_conversion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
