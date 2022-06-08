import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

class service_taskAssigner(Node):
    def __init__(self, n_SycaBot):
        super().__init__('task_assignment', namespace='central')
        self.srv = self.create_service(Point, 'task', self.assign_task)

    def assign_task(self, request, response):
        return