#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
import math


class DrawPath(Node):
    def __init__(self):
        super().__init__('draw_path')
        self.path_publisher = self.create_publisher(Path, 'path', 10)
        self.timer = self.create_timer(0.1, self.publish_path)  # 10Hz 주기로 실행

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        # 시작점 설정
        num_points = 100  # 점의 개수
        for i in range(1, num_points):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = i * 10.0 / num_points
            pose.pose.position.y = math.sin(i * 0.1)
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)


        # 경로 메시지 발행
        self.path_publisher.publish(path_msg)
        self.get_logger().info('Publishing path from (0,0) to (10,0)')


def main(args=None):
    rclpy.init(args=args)
    draw_path_node = DrawPath()
    rclpy.spin(draw_path_node)
    draw_path_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
