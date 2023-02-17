from parody_msgs.srv import MotorTrigger

import rclpy

import argparse


def zero_joints(joints_to_zero: list[int]) -> None:

    rclpy.init()
    node = rclpy.create_node("joint_zeroer")
    cli = node.create_client(MotorTrigger, "zero_joint")

    node.get_logger().info(f"attempting to zero joints with ids {joints_to_zero}")

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("service not available, waiting again...")

    for joint_to_zero in joint_to_zero:
        req = MotorTrigger.Request()
        req.joint_number = joint_to_zero
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future)

        try:
            result = future.result()
        except Exception as e:
            node.get_logger().info(
                f"Service call failed for joint {joint_to_zero}, error {e}"
            )
        else:
            node.get_logger().info(
                f"Result of zeroing joint {joint_to_zero}: {result.success}, {result.message}"
            )

    node.destroy_node()
    rclpy.shutdown()


def zero_left_leg() -> None:
    joints_to_zero = [0, 1, 2, 3]
    zero_joints(joints_to_zero)


def zero_right_leg() -> None:
    joints_to_zero = [4, 5, 6, 7]
    zero_joints(joints_to_zero)


def zero_neck() -> None:
    joints_to_zero = [8, 9, 10, 11, 12]
    zero_joints(joints_to_zero)


def zero_tail() -> None:
    joints_to_zero = [13]
    zero_joints(joints_to_zero)
