# Based on: https://github.com/ros-planning/navigation2/blob/humble/nav2_simple_commander/nav2_simple_commander/example_nav_to_pose.py

import sys
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import QoSPresetProfiles

from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf_transformations import euler_from_quaternion, quaternion_from_euler
import angles

from assessment_interfaces.msg import Item, ItemList, Zone, ZoneList, Robot, RobotList, ItemHolder, ItemHolders, ItemLog
from auro_interfaces.srv import ItemRequest

from enum import Enum

class State(Enum):
    SET_GOAL = 0
    NAVIGATING = 1
    SPINNING = 2
    BACKUP = 3
    SEARCHING = 4

class AutonomousNavigation(Node):

    def __init__(self):
        super().__init__('autonomous_navigation_multithreaded')

        self.state = State.SET_GOAL

        subscriber_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()
        client_callback_group = MutuallyExclusiveCallbackGroup()

        self.setup_subscribers(subscriber_callback_group)
    
        self.pick_up_service = self.create_client(ItemRequest, '/pick_up_item', callback_group=client_callback_group)
        self.offload_service = self.create_client(ItemRequest, '/offload_item', callback_group=client_callback_group)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.distance = 0.0
        self.angle = self.get_parameter('yaw').value
        
        self.get_logger().info(f"Initial pose: ({self.x:.2f}, {self.y:.2f}), {self.angle:.2f} degrees")

        self.current_goal = 0
        self.potential_goals = []

        self.potential_goals.append(Point(x = 0.0, y = 0.0))

        self.potential_goals.append(Point(x = 2.57, y = 2.5))
        self.potential_goals.append(Point(x = 2.57, y = -2.46))
        self.potential_goals.append(Point(x = -3.42, y =  2.5))
        self.potential_goals.append(Point(x = -3.42, y =  -2.46))

        self.navigator = BasicNavigator()
        
        self.robot_id = self.get_namespace().replace('/', '')
        self.get_logger().info(f"Robot ID: {self.get_namespace()}")

        initial_pose = PoseStamped()
        initial_pose.pose.position.x = self.x
        initial_pose.pose.position.y = self.y
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = self.angle
        self.get_logger().info(f"Initial Pose: {initial_pose}")
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        (initial_pose.pose.orientation.x,
         initial_pose.pose.orientation.y,
         initial_pose.pose.orientation.z,
         initial_pose.pose.orientation.w) = quaternion_from_euler(0, 0, math.radians(0), axes='sxyz')

        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()

        
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop, callback_group=timer_callback_group)

        self.previous_time = self.get_clock().now()


    def odom_callback(self, msg):
        self.get_logger().info(f"odom: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")


    def item_callback(self, msg):
        self.items = msg
    
    def zone_callback(self, msg):
        self.zones = msg
    
    def robot_callback(self, msg):
        self.robots = msg

    def item_holders_callback(self, msg):
        self.item_holders = msg
    
    def scan_callback(self, msg):
        pass
    
    def control_loop(self):

        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time())
            
            self.x = t.transform.translation.x
            self.y = t.transform.translation.y

            (roll, pitch, yaw) = euler_from_quaternion([t.transform.rotation.x,
                                                        t.transform.rotation.y,
                                                        t.transform.rotation.z,
                                                        t.transform.rotation.w])

            self.distance = math.sqrt(self.x ** 2 + self.y ** 2)
            self.angle = math.atan2(self.y, self.x)

            # self.get_logger().info(f"self.x: {self.x:.2f}")
            # self.get_logger().info(f"self.y: {self.y:.2f}")
            # self.get_logger().info(f"yaw (degrees): {math.degrees(yaw):.2f}")
            # self.get_logger().info(f"distance: {self.distance:.2f}")
            # self.get_logger().info(f"angle (degrees): {math.degrees(self.angle):.2f}")

        except TransformException as e:
            self.get_logger().info(f"{e}")

        time_difference = self.get_clock().now() - self.previous_time

        if time_difference > Duration(seconds = 300):
            self.navigator.cancelTask()
            self.previous_time = self.get_clock().now()
            self.get_logger().info(f"SPINNING...")
            self.state = State.SPINNING

        self.get_logger().info(f"State: {self.state}")

        match self.state:

            case State.SET_GOAL:

                if len(self.potential_goals) == 0:
                    self.state = State.SPINNING
                    return

                self.current_goal = random.randint(0, len(self.potential_goals) - 1)
                angle = random.uniform(-180, 180)

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()                
                goal_pose.pose.position = self.potential_goals[self.current_goal]

                del self.potential_goals[self.current_goal]

                (goal_pose.pose.orientation.x,
                 goal_pose.pose.orientation.y,
                 goal_pose.pose.orientation.z,
                 goal_pose.pose.orientation.w) = quaternion_from_euler(0, 0, math.radians(angle), axes='sxyz')
                
                self.get_logger().info(f"Remaining goals:")

                for goal in self.potential_goals:
                    self.get_logger().info(f"{goal}")
                
                self.get_logger().info(f"Navigating to: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}), {angle:.2f} degrees")

                self.navigator.goToPose(goal_pose)
                self.state = State.NAVIGATING

            case State.NAVIGATING:

                if not self.navigator.isTaskComplete():

                    feedback = self.navigator.getFeedback()
                    self.get_logger().info(f"Estimated time of arrival: {(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9):.0f} seconds")

                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds = 30):
                        self.get_logger().info(f"Navigation took too long... cancelling")
                        self.navigator.cancelTask()

                else:

                    result = self.navigator.getResult()

                    match result:

                        case TaskResult.SUCCEEDED:
                            self.get_logger().info(f"Goal succeeded!")

                            self.get_logger().info(f"Spinning")

                            self.navigator.spin(spin_dist=math.radians(180), time_allowance=10)
                            self.state = State.SPINNING

                        case TaskResult.CANCELED:
                            self.get_logger().info(f"Goal was canceled!")
                            
                            self.state = State.SET_GOAL

                        case TaskResult.FAILED:
                            self.get_logger().info(f"Goal failed!")

                            self.state = State.SET_GOAL

                        case _:
                            self.get_logger().info(f"Goal has an invalid return status!")

            case State.SPINNING:

                if not self.navigator.isTaskComplete():

                    feedback = self.navigator.getFeedback()
                    self.get_logger().info(f"Turned: {math.degrees(feedback.angular_distance_traveled):.2f} degrees")

                else:

                    result = self.navigator.getResult()

                    match result:

                        case TaskResult.SUCCEEDED:
                            self.get_logger().info(f"Goal succeeded!")

                            self.get_logger().info(f"Backing up")

                            self.navigator.backup(backup_dist=0.15, backup_speed=0.025, time_allowance=10)
                            self.state = State.BACKUP

                        case TaskResult.CANCELED:
                            self.get_logger().info(f"Goal was canceled!")

                            self.state = State.SET_GOAL

                        case TaskResult.FAILED:
                            self.get_logger().info(f"Goal failed!")

                            self.state = State.SET_GOAL

                        case _:
                            self.get_logger().info(f"Goal has an invalid return status!")

            case State.BACKUP:

                if not self.navigator.isTaskComplete():

                    feedback = self.navigator.getFeedback()
                    self.get_logger().info(f"Distance travelled: {feedback.distance_traveled:.2f} metres")

                else:

                    result = self.navigator.getResult()

                    match result:

                        case TaskResult.SUCCEEDED:
                            self.get_logger().info(f"Goal succeeded!")

                            self.state = State.SET_GOAL

                        case TaskResult.CANCELED:
                            self.get_logger().info(f"Goal was canceled!")

                            self.state = State.SET_GOAL

                        case TaskResult.FAILED:
                            self.get_logger().info(f"Goal failed!")

                            self.state = State.SET_GOAL

                        case _:
                            self.get_logger().info(f"Goal has an invalid return status!")


            case _:
                pass
    
    def setup_subscribers(self, subscriber_callback_group):
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10,
            callback_group=subscriber_callback_group)
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value, callback_group=subscriber_callback_group)
        
        self.item_subscriber = self.create_subscription(
            ItemList,
            'items',
            self.item_callback,
            10, callback_group=subscriber_callback_group
        )

        self.zone_subscriber = self.create_subscription(
            ZoneList,
            'zone',
            self.zone_callback,
            10, callback_group=subscriber_callback_group
        )

        self.robot_subscriber = self.create_subscription(
            RobotList,
            'robots',
            self.robot_callback,
            10, callback_group=subscriber_callback_group
        )

        self.item_holders_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.item_holders_callback,
            10, callback_group=subscriber_callback_group
        )
    def destroy_node(self):
        self.get_logger().info(f"Shutting down")
        self.navigator.lifecycleShutdown()
        self.navigator.destroyNode()
        super().destroy_node()
        

def main(args=None):

    rclpy.init(args = args)

    node = AutonomousNavigation()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()