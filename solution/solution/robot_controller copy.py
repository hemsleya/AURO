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
    OFFLOADING = 1
    SPINNING = 2
    BACKUP = 3
    COLLECTING = 4
    FORWARD = 5


LINEAR_VELOCITY  = 0.3 # Metres per second
ANGULAR_VELOCITY = 0.5 # Radians per second

TURN_LEFT = 1 # Postive angular velocity turns left
TURN_RIGHT = -1 # Negative angular velocity turns right

SCAN_THRESHOLD = 0.5 # Metres per second
 # Array indexes for sensor sectors
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3


class AutonomousNavigation(Node):

    def __init__(self):
        super().__init__('CONTROLLER')

        self.state = State.SET_GOAL

        subscriber_callback_group = MutuallyExclusiveCallbackGroup()
        publisher_callback_group = MutuallyExclusiveCallbackGroup()
        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.setup_subscribers(subscriber_callback_group)

        self.pick_up_service = self.create_client(ItemRequest, '/pick_up_item', callback_group=client_callback_group)
        self.offload_service = self.create_client(ItemRequest, '/offload_item', callback_group=client_callback_group)

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.distance = 0.0
        self.yaw = self.get_parameter('yaw').value
        
        self.get_logger().info(f"Initial pose: ({self.x:.2f}, {self.y:.2f}), {self.yaw:.2f} degrees")

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
        initial_pose.pose.orientation.w = self.yaw
        self.get_logger().info(f"Initial Pose: {initial_pose}")
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        (initial_pose.pose.orientation.x,
         initial_pose.pose.orientation.y,
         initial_pose.pose.orientation.z,
         initial_pose.pose.orientation.w) = quaternion_from_euler(0, 0, math.radians(0), axes='sxyz')

        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()


        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10, callback_group=publisher_callback_group)
        
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop, callback_group=timer_callback_group)

        self.previous_time = self.get_clock().now()


    def odom_callback(self, msg):
        self.get_logger().info(f"odom: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")
        self.pose = msg.pose.pose

        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                    self.pose.orientation.y,
                                                    self.pose.orientation.z,
                                                    self.pose.orientation.w])
        
        self.yaw = yaw

    def item_callback(self, msg):
        self.items = msg
    
    def zone_callback(self, msg):
        self.zones = msg
    
    def robot_callback(self, msg):
        self.robots = msg

    def item_holders_callback(self, msg):
        self.item_holders = msg

    def scan_callback(self, msg):
        # Group scan ranges into 4 segments
        # Front, left, and right segments are each 60 degrees
        # Back segment is 180 degrees
        front_ranges = msg.ranges[331:359] + msg.ranges[0:30] # 30 to 331 degrees (30 to -30 degrees)
        left_ranges  = msg.ranges[31:90] # 31 to 90 degrees (31 to 90 degrees)
        back_ranges  = msg.ranges[91:270] # 91 to 270 degrees (91 to -90 degrees)
        right_ranges = msg.ranges[271:330] # 271 to 330 degrees (-30 to -91 degrees)

        # Store True/False values for each sensor segment, based on whether the nearest detected obstacle is closer than SCAN_THRESHOLD
        self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
        self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SCAN_THRESHOLD

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
            self.yaw = math.atan2(self.y, self.x)

            # self.get_logger().info(f"self.x: {self.x:.2f}")
            # self.get_logger().info(f"self.y: {self.y:.2f}")
            # self.get_logger().info(f"yaw (degrees): {math.degrees(yaw):.2f}")
            # self.get_logger().info(f"distance: {self.distance:.2f}")
            # self.get_logger().info(f"angle (degrees): {math.degrees(self.yaw):.2f}")

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
                self.state = State.OFFLOADING

            case State.OFFLOADING:

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

                            rqt = ItemRequest.Request()
                            rqt.robot_id = self.robot_id
                            try:
                                future = self.offload_service.call_async(rqt)
                                self.executor.spin_until_future_complete(future)
                                response = future.result()
                                if response.success:
                                    self.get_logger().info('Item picked up.')
                                    self.state = State.OFFLOADING
                                    self.items.data = []
                                else:
                                    self.get_logger().info('Unable to pick up item: ' + response.message)
                            except Exception as e:
                                self.get_logger().info('Exception ' + e)

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

            case State.FORWARD:

                if self.scan_triggered[SCAN_FRONT]:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = random.uniform(150, 170)
                    self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                    self.get_logger().info("Detected obstacle in front, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                    return
                
                if self.scan_triggered[SCAN_LEFT] or self.scan_triggered[SCAN_RIGHT]:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = 45

                    if self.scan_triggered[SCAN_LEFT] and self.scan_triggered[SCAN_RIGHT]:
                        self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                        self.get_logger().info("Detected obstacle to both the left and right, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                    elif self.scan_triggered[SCAN_LEFT]:
                        self.turn_direction = TURN_RIGHT
                        self.get_logger().info(f"Detected obstacle to the left, turning right by {self.turn_angle} degrees")
                    else: # self.scan_triggered[SCAN_RIGHT]
                        self.turn_direction = TURN_LEFT
                        self.get_logger().info(f"Detected obstacle to the right, turning left by {self.turn_angle} degrees")
                    return
                
                if len(self.items.data) > 0 and not self.holding_item():
                    self.state = State.COLLECTING
                    return
                
                if len(self.zones.data) > 0 and self.holding_item():
                    self.state = State.SET_GOAL
                    return

                msg = Twist()
                msg.linear.x = LINEAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)

                difference_x = self.pose.position.x - self.previous_pose.position.x
                difference_y = self.pose.position.y - self.previous_pose.position.y
                distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)

                # self.get_logger().info(f"Driven {distance_travelled:.2f} out of {self.goal_distance:.2f} metres")

                if distance_travelled >= self.goal_distance:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = random.uniform(30, 150)
                    self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                    self.get_logger().info("Goal reached, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")

            case State.TURNING:

                if len(self.items.data) > 0 and not self.holding_item():
                    self.state = State.COLLECTING
                    return
                
                if len(self.zones.data) > 0 and self.holding_item():
                    self.state = State.SET_GOAL
                    return

                msg = Twist()
                msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)

                # self.get_logger().info(f"Turned {math.degrees(math.fabs(yaw_difference)):.2f} out of {self.turn_angle:.2f} degrees")

                yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)                

                if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
                    self.previous_pose = self.pose
                    self.goal_distance = random.uniform(1.0, 2.0)
                    self.state = State.FORWARD
                    self.get_logger().info(f"Finished turning, driving forward by {self.goal_distance:.2f} metres")

            case State.COLLECTING:

                if len(self.items.data) == 0:
                    self.previous_pose = self.pose
                    self.state = State.FORWARD
                    return
                
                item = self.items.data[0]

                # Obtained by curve fitting from experimental runs.
                estimated_distance = 32.4 * float(item.diameter) ** -0.75 #69.0 * float(item.diameter) ** -0.89

                self.get_logger().info(f'Estimated distance {estimated_distance}')

                if estimated_distance <= 0.35:
                    rqt = ItemRequest.Request()
                    rqt.robot_id = self.robot_id
                    try:
                        future = self.pick_up_service.call_async(rqt)
                        self.executor.spin_until_future_complete(future)
                        response = future.result()
                        if response.success:
                            self.get_logger().info('Item picked up.')
                            self.state = State.OFFLOADING
                            self.items.data = []
                        else:
                            self.get_logger().info('Unable to pick up item: ' + response.message)
                    except Exception as e:
                        self.get_logger().info('Exception ' + e)   

                msg = Twist()
                msg.linear.x = 0.25 * estimated_distance
                msg.angular.z = item.x / 320.0
                self.cmd_vel_publisher.publish(msg)

            case _:
                pass

    def destroy_node(self):
        self.get_logger().info(f"Shutting down")
        self.navigator.lifecycleShutdown()
        self.navigator.destroyNode()
        super().destroy_node()
        
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