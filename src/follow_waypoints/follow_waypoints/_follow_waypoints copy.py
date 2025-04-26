#!/usr/bin/env python

import signal
import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from rclpy.task import Future

class DogFollowWaypoints(Node):
    def __init__(self):
        super().__init__('dog_follow_waypoints_node')
        self.get_logger().info('Dogzilla Follow Waypoints node started')
        self.srv = self.create_service(Empty, 'dog_follow_waypoints', self.service_callback)
        
        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        
    def service_callback(self, request, response):
        self.get_logger().info('Follow Waypoints service called')
        self.send_goal()
        return response
      
    def send_goal(self):
      goal_msg = FollowWaypoints.Goal()
      self._action_client.wait_for_server()

      return self._action_client.send_goal_async(goal_msg)
  
    def cancel_goal(self):
        """Cancel the goal when Ctrl+C is pressed."""
        if hasattr(self, 'goal_handle') and self.goal_handle is not None:
            self.get_logger().info('Cancelling goal...')
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
        rclpy.shutdown()  
  
  
def main(args=None):
    rclpy.init(args=args)
    
    dog_follow_waypoints_node = DogFollowWaypoints()
    
    # Handle Ctrl+C to cancel goal
    def signal_handler(sig, frame):
        dog_follow_waypoints_node._action_client.get_logger().info('Ctrl+C detected, cancelling goal...')
        dog_follow_waypoints_node._action_client.cancel_goal()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    
    rclpy.spin(dog_follow_waypoints_node)
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()