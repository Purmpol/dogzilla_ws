#!/usr/bin/env python

import sys, os
import copy
import rclpy
from rclpy.node import Node
import std_msgs

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from nav2_msgs.action._follow_waypoints import FollowWaypoints_FeedbackMessage, FollowWaypoints_Result
from nav_msgs.msg import Path

from custom_interfaces.msg import DogWaypointState  # States for the control buttons on Foxglove
from custom_interfaces.srv import CustomWayPoints # Waypoints service

NAMES = {0: "A", 1: "B", 2: "C"}

class DogFollowWaypoints(Node):
    def __init__(self):
        super().__init__('dog_follow_waypoints_node')
        self.get_logger().info('Dogzilla Follow Waypoints node started')
        self.srv = self.create_service(CustomWayPoints, 'dog_follow_waypoints', self.service_callback)
           
        # For single goal
        self.single_goal_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')       
        # For multi goals 
        self.muti_goal_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.action_goal_handle = None        
        
        # States for Foxglove extension, periodically publishing the state        
        self.pub_dog_wp_state = self.create_publisher(DogWaypointState, '/dog_waypoint_state', 1)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.dog_wp_state = DogWaypointState()
        
        # Rewrite the frame_id of the Path topic for display in Foxglove
        # On Dogzilla's VM (PC), we in the file
        # ~/yahboomcar_ws/install/yahboom_dog_nav/share/yahboom_dog_nav/launch/navigation_launch.py
        # we must add this line:
        # remappings = [('/tf', 'tf'),
        #       ('/tf_static', 'tf_static'),
        #      ('/plan', '/plan_remap')]
        self.sub_plan = self.create_subscription(Path, '/plan_remap', self.repub_callback, 10)
        self.pub_plan = self.create_publisher(Path, '/plan', 10)
     
    def repub_callback(self, msg: Path):
        """ Modify the frame_id from "" to "map" """
        # For details, study nav_msgs/Path.msg 
        msg.header.frame_id = 'map'         
        for pose in msg.poses:
            pose.header.frame_id = 'map'
        self.pub_plan.publish(msg)   
        
    def timer_callback(self):
        """ Periodically publish the states to for Foxglove """
        self.pub_dog_wp_state.publish(self.dog_wp_state)     
                             
    def service_callback(self, request, response):
        self.get_logger().info('Dog Follow Waypoints service was called')
        # Check if already called send_goal (robot is still moving)
        if self.action_goal_handle: 
            # To cancel goal, Foxglove will send empty list of waypoints
            if len(request.poses) == 0:
                self.get_logger().info('Canceling Goal ...')
                self.dog_wp_state.is_wait_cancel = True
                self.pub_dog_wp_state.publish(self.dog_wp_state) 
                
                cancel_future = self.action_goal_handle.result().cancel_goal_async()
                cancel_future.add_done_callback(self.goal_canceled_callback)
                self.action_goal_handle = None
                self.goal_pose_msg = None                
                return response
            
        else:
            # To navigate to a single goal, Foxglove will send a list of waypoints with length 1
            if len(request.poses) == 1:
                # Single goal
                positionX = request.poses[0].pose.position.x
                positionY = request.poses[0].pose.position.y
                orientationZ = request.poses[0].pose.orientation.z
                self.send_goal(positionX, positionY, orientationZ)
                
                self.dog_wp_state.current_point_index = 1  # Start from 1
                # frame_id from Foxglove is the name of the waypoint "A", "B", "C"
                self.dog_wp_state.all_points = [request.poses[0].header.frame_id]
                return response
            # To follow multiple goals, Foxglove will send a list of waypoints with length = 3
            elif len(request.poses) == 3:   
                # Muti goals                            
                goal_msg = FollowWaypoints.Goal()
                goal_msg.poses = copy.deepcopy(request.poses)
                # Must change the frame_id to "map" to operate Follow Waypoints correctly
                for pose in goal_msg.poses:
                    pose.header.frame_id = "map"
                    
                self.muti_goal_client.wait_for_server()
                self.get_logger().info(f"Sending multi goals with {len(request.poses)} poses")
                self.action_goal_handle = self.muti_goal_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
                self.action_goal_handle.add_done_callback(self.goal_response_callback)
                
                self.dog_wp_state.current_point_index = 1  # Start from 1
                self.dog_wp_state.all_points = [request.poses[i].header.frame_id for i in range(3)]    
                return response
            
            # To restart the node, Foxglove will send a list of nulls with length 5
            elif len(request.poses) == 5:
                # Restart this node
                self.restart_node()                
                return response
                     
    def reset_wp_state(self, state):
        """ Reset the waypoint state for Foxglove """
        self.get_logger().info('Resetting waypoint state ...')
        self.action_goal_handle = None
        self.dog_wp_state.current_point_index = 0
        self.dog_wp_state.all_points = []
        self.dog_wp_state.is_wait_cancel = False
        self.pub_dog_wp_state.publish(self.dog_wp_state)  
        
    def goal_canceled_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.reset_wp_state("cancel")
        else:
            self.get_logger().warning('Goal failed to cancel')
      
    def send_goal(self, x, y, yaw):
        """ Send a goal pose to Nav2 """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = yaw  # Simplified orientation (not full quaternion)
        goal_msg.pose.pose.orientation.w = 1.0  # Assume no rotation

        self.single_goal_client.wait_for_server()
        self.get_logger().info(f"Sending goal: x={x}, y={y}")
        self.action_goal_handle = self.single_goal_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.action_goal_handle.add_done_callback(self.goal_response_callback)
  
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected!")
            return
        self.get_logger().info("Goal accepted!")
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Log feedback during navigation."""
        # Check if the feedback message is from NavigateToPose or FollowWaypoints
        if isinstance(feedback_msg, NavigateToPose_FeedbackMessage):
            feedback = feedback_msg.feedback
            self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} m")
        elif isinstance(feedback_msg, FollowWaypoints_FeedbackMessage):
            feedback = feedback_msg.feedback
            self.get_logger().info(f"Current Waypoint: {NAMES[feedback.current_waypoint]}")
            self.dog_wp_state.current_point_index = feedback.current_waypoint + 1  # Start from 1

    def result_callback(self, future):
        """Handle goal result."""
        if (isinstance(future.result().result, FollowWaypoints_Result)):
            result = future.result()
            missed_waypoints = result.result.missed_waypoints
            self.get_logger().info(f"Number of the Missed WayPoints: {len(missed_waypoints)}")
            if len(missed_waypoints) == 0:
                self.get_logger().info("Multi goals: Goal reached successfully!")
                self.reset_wp_state("success")
            else:   
                self.get_logger().warn("Multi goals: Failed to reach goal.")   
                self.reset_wp_state() 
                self.dog_wp_state.failed_point_index  = missed_waypoints[-1]        
        else:
            result = future.result().result
            if result.result == std_msgs.msg.Empty():
                self.get_logger().info("SinglePoint: Goal reached successfully!") 
                self.reset_wp_state("success")
            else:
                self.get_logger().warn("Failed to reach goal.")
                
    def restart_node(self):
        """ Restart the node """
        self.get_logger().info('Restarting node...')
        python = sys.executable
        os.execl(python, python, *sys.argv)  # Replaces current process with a new instance
        
  
def main():
    rclpy.init()
    dog_follow_waypoints_node = DogFollowWaypoints()
    rclpy.spin(dog_follow_waypoints_node)
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
