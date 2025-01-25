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

from geometry_msgs.msg import PoseStamped, Point, Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import asyncio

from tf_transformations import euler_from_quaternion, quaternion_from_euler
import angles

from assessment_interfaces.msg import Item, ItemList, Zone, ZoneList, Robot, RobotList, ItemHolder, ItemHolders, ItemLog
from auro_interfaces.srv import ItemRequest
from solution_interfaces.msg import ItemZone, ItemZones

from enum import Enum

class State(Enum):
    SET_GOAL = 0
    NAVIGATING = 1
    SPINNING = 2
    ITEM_HANDLER = 3
    RECOVERING = 4


    

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.min_x = -3.5
        self.max_x = 2.6
        self.min_y = -2.5
        self.max_y = 2.5

        self.state = State.SPINNING
        self.items = ItemList()
        self.zones = ZoneList()
        self.robots = RobotList()
        self.item_holders = ItemHolders()
        self.OccupancyGrid = OccupancyGrid()
        self.navigating_to_item = False
        self.item_holder = ItemHolder()
        self.first_run = True
        self.item_retry_count = 0
        self.zone = Zone()
        self.item_zones = ItemZones()

        self.zone_goal_index = 0
        self.zone_goals = []
        self.current_goal = Point()

        self.zone_goals.append(Point(x = 2.57, y = 2.5))
        self.zone_goals.append(Point(x = 2.57, y = -2.46))
        self.zone_goals.append(Point(x = -3.42, y =  2.5))
        self.zone_goals.append(Point(x = -3.42, y =  -2.46))

        self.navigator = BasicNavigator()

        self.robot_id = self.get_namespace().replace('/', '')
        self.get_logger().info(f"Robot ID: {self.get_namespace()}")

        self.rqt = ItemRequest.Request()
        self.rqt.robot_id = self.robot_id

        subscriber_callback_group = MutuallyExclusiveCallbackGroup()
        publisher_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()
        client_callback_group = MutuallyExclusiveCallbackGroup()

        self.setup_subscribers(subscriber_callback_group)
    
        self.pick_up_service = self.create_client(ItemRequest, '/pick_up_item', callback_group=client_callback_group)
        self.offload_service = self.create_client(ItemRequest, '/offload_item', callback_group=client_callback_group)

        self.zone_item_publisher = self.create_publisher(ItemZones, '/item_zones', 10, callback_group=publisher_callback_group)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        self.x = self.initial_x
        self.y = self.initial_y
        self.angle = self.initial_yaw

        initial_pose = PoseStamped()
        initial_pose.pose.position.x = self.x
        initial_pose.pose.position.y = self.y
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = self.angle
        self.get_logger().info(f"Initial pose: ({self.x:.2f}, {self.y:.2f}), {self.angle:.2f} degrees")
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        (initial_pose.pose.orientation.x,
         initial_pose.pose.orientation.y,
         initial_pose.pose.orientation.z,
         initial_pose.pose.orientation.w) = quaternion_from_euler(0, 0, math.radians(0), axes='sxyz')
        
        self.current_pose = initial_pose

        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop, callback_group=timer_callback_group)

        self.previous_time = self.get_clock().now()
        self.get_logger().info(f"Starting control loop")

    def odom_callback(self, msg):
        #self.get_logger().info(f"odom: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")
        self.current_pose = msg.pose
        (_, _, self.angle) = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                        msg.pose.pose.orientation.y,
                                                        msg.pose.pose.orientation.z,
                                                        msg.pose.pose.orientation.w])
    # def true_pose_callback(self, msg):
    #     self.x = pose.x
    #     self.y = pose.y
    #     self.get_logger().info(f"set new pose: {pose}")

    #     pose_stamped = PoseStamped()
    #     pose_stamped.header.stamp = self.get_clock().now().to_msg()
    #     pose_stamped.pose = pose

       

    def item_callback(self, msg):
        self.items = msg
    
    def zone_callback(self, msg):
        self.zones = msg
    
    def robot_callback(self, msg):
        self.robots = msg

    def item_holders_callback(self, msg):
        self.item_holders = msg
        for item_holder in self.item_holders.data:
            if item_holder.robot_id == self.robot_id:
                self.item_holder = item_holder
                return    

    
    def item_zones_callback(self, msg):
        self.item_zones = msg

    def scan_callback(self, msg):
            pass


    async def control_loop(self):
        if self.first_run:
            self.navigator.spin(spin_dist=math.radians(360), time_allowance=10)
            self.first_run = False
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
            #self.angle = math.atan2(self.y, self.x) # angle from x axis
           # self.get_logger().info(f"x, y: {self.x, self.y}")
        except TransformException as e:
            self.get_logger().info(f"{e}")    

       # self.get_logger().info(f"Initial pose - x: {self.initial_x}, y: {self.initial_y}, yaw: {self.initial_yaw}")

        time_difference = self.get_clock().now() - self.previous_time

        if time_difference > Duration(seconds = 300):
            self.navigator.cancelTask()
            self.previous_time = self.get_clock().now()
            self.get_logger().info(f"SPINNING...")
            self.state = State.SPINNING

        #self.get_logger().info(f"State: {self.state}")

        match self.state:
            case State.SET_GOAL:
                #if not holding an item, set goal to item
                #if no items found, set goal to random coordinates
                #self.get_logger().info(f"holding_item: {self.item_holder.holding_item}")
                if not self.item_holder.holding_item:
                    if len(self.items.data) > 0:
                        self.get_logger().info(f"Items: {self.items.data}")
                        item = self.items.data[0]
                        self.get_logger().info(f"Item: {self.items.data[0]}")
                        estimated_distance = 32.4 * float(item.diameter) ** -0.75 #69.0 * float(item.diameter) ** -0.89

                        x = self.current_pose.pose.position.x
                        y = self.current_pose.pose.position.y
                        orientation = self.current_pose.pose.orientation
                        (_, _, theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

                        direction_angle = math.atan2(item.y, item.x)
                        
                        item_global_x = x - (estimated_distance * math.cos(theta + direction_angle) )
                        item_global_y = y + (estimated_distance * math.sin(theta + direction_angle) )

                        self.get_logger().info(f"Goal: ({item_global_x:.2f}, {item_global_y:.2f})")
                        self.current_goal = Point(x = item_global_x, y = item_global_y)
                        self.navigating_to_item = True  
                    else:
                        self.get_logger().info(f"items not found, getting random coordinate...")
                        self.current_goal = Point(x=random.uniform(self.min_x, self.max_x), y=random.uniform(self.min_y, self.max_y))
                        self.get_logger().info(f"Goal: ({self.current_goal.x:.2f}, {self.current_goal.y:.2f})")
                
                #if holding an item, set goal to zone
                #if no zones, spin
                elif self.item_holder.holding_item:
                    if len(self.zone_goals) == 0:
                        self.state = State.SPINNING
                        return
                    self.get_logger().info(f"Setting goal to zone...")
                    self.current_goal = self.pick_zone()
                

                goal_pose = self.create_goal_pose(self.current_goal)
                
                #self.get_logger().info(f"Navigating to: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})")

                self.navigator.goToPose(goal_pose)
                self.state = State.NAVIGATING
                
                self.get_logger().info(f"Navigating...")


            case State.NAVIGATING:
                #if not holding an item or navigating to and item, set goal to item
                if self.should_redirect_to_item():
                    if len(self.items.data) > 0:
                        self.state = State.SET_GOAL
                        self.get_logger().info(f"Setting goal...")
                        self.navigator.cancelTask()
                if self.item_holder.holding_item and len(self.zones.data) > 0:
                    self.zone = self.zones.data[0]
                else:
                    self.zone = None
                if not self.navigator.isTaskComplete():

                    feedback = self.navigator.getFeedback()
                    

                    if feedback.number_of_recoveries > 5:
                        self.get_logger().info(f"Recovery failed... cancelling ... spinning")
                        self.navigator.cancelTask()
                        self.navigator.clearLocalCostmap()
                        self.navigator.spin(spin_dist=math.radians(360), time_allowance=10)
                        self.state = State.SPINNING
                        return
                        
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds = 15):
                        self.get_logger().info(f"Navigation took too long... cancelling")
                        self.navigator.cancelTask()

                else:

                    result = self.navigator.getResult()

                    match result:

                        case TaskResult.SUCCEEDED:
                            self.get_logger().info(f"Goal succeeded!")
                            #if holding an item, offload item
                            #if not holding an item, pick up item
                            if self.navigating_to_item or self.item_holder.holding_item:
                                self.state = State.ITEM_HANDLER
                                self.get_logger().info(f"Item handler...")
                            else:
                                #if not holding an item, set goal to item
                                self.state = State.SET_GOAL
                                self.get_logger().info(f"Setting goal...")


                        case TaskResult.CANCELED:
                            self.get_logger().info(f"Goal was canceled!")
                            
                            self.state = State.SET_GOAL
                            self.get_logger().info(f"Setting goal...")
                            self.navigating_to_item = False

                        case TaskResult.FAILED:
                            self.get_logger().info(f"Goal failed!")

                            self.state = State.SET_GOAL
                            self.get_logger().info(f"Setting goal...")
                            self.navigating_to_item = False

                        case _:
                            self.get_logger().info(f"Goal has an invalid return status!")

            case State.SPINNING:
                if not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                   # self.get_logger().info(f"Turned: {math.degrees(feedback.angular_distance_traveled):.2f} degrees")
                    if self.should_redirect_to_item():
                        self.get_logger().info(f"Setting goal...")
                        self.state = State.SET_GOAL
                        self.navigator.cancelTask()
                else:

                    result = self.navigator.getResult()

                    match result:

                        case TaskResult.SUCCEEDED:
                            self.get_logger().info(f"Spin completed!")
                            self.state = State.SET_GOAL
                            self.get_logger().info(f"Setting goal...")

                        case TaskResult.CANCELED:
                            self.get_logger().info(f"Spin was canceled!")

                            self.state = State.SET_GOAL
                            self.get_logger().info(f"Setting goal...")

                        case TaskResult.FAILED:
                            self.get_logger().info(f"Spin failed!")

                            self.state = State.SET_GOAL
                            self.get_logger().info(f"Setting goal...")

                        case _:
                            self.get_logger().info(f"Spin has an invalid return status!")

            case State.ITEM_HANDLER:
                #todo: save item held to add to data if successful
                temp_item_data = self.item_holder
                try:
                    if self.item_holder.holding_item:
                        if self.zone != None:
                            future = await self.offload_service.call_async(self.rqt)
                        else:
                            self.zone_goals.remove(self.current_goal)
                    else:
                        future = await self.pick_up_service.call_async(self.rqt)
    
                    response = future

                    if response is None:
                        if self.item_retry_count < 5:
                            self.get_logger().info('No response received, retrying...')
                            self.item_retry_count += 1
                            return
                    elif response.success:
                        self.get_logger().info("Item Handler response: {response.message}") #Robot 'robot1' collected/offloaded item successfully
                        self.navigator.spin(spin_dist=math.radians(180), time_allowance=10)
                        self.state = State.SPINNING
                        self.get_logger().info(f"Spinning...")
                        self.item_retry_count = 0
                        if not self.navigating_to_item:
                            self.update_item_zones(temp_item_data)
                            self.zone_item_publisher.publish(self.item_zones)
                        #self.items.data = []
                        self.navigating_to_item = False
                        return 
                    
                    self.get_logger().info(f"Item Handler response: {response.message}")
                    self.state = State.SET_GOAL
                    self.get_logger().info(f"Setting goal...")
                    self.item_retry_count = 0
                    self.navigating_to_item = False

                    # if response is None and self.item_retry_count < 5:
                    #     self.get_logger().info('No response received')
                    #     self.item_retry_count += 1
                    # elif response is not None and response.success:
                    #     self.get_logger().info(response.message)
                    #     self.navigator.spin(spin_dist=math.radians(180), time_allowance=10)
                    #     self.state = State.SPINNING
                    #     self.item_retry_count = 0
                    #     self.items.data = []
                    #     self.navigating_to_item = False
                    # else:
                    #     self.get_logger().info(response.message)
                    #     self.state = State.SET_GOAL
                    #     self.item_retry_count = 0
                except Exception as e:
                    self.get_logger().info(f'Exception {e}')  
    
            case _:
                pass


    def setup_subscribers(self, subscriber_callback_group):
        self.odom_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
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

        self.item__zones_subscriber = self.create_subscription(
            ItemZones,
            '/item_zones',
            self.item_callback,
            10, callback_group=subscriber_callback_group
        )

        # self.true_pose_subscriber = self.create_subscription(
        #     Odometry,
        #     '/world',
        #     self.true_pose_callback,
        #     10, callback_group=subscriber_callback_group)

    def pick_zone(self):
        self.get_logger().info("picking zone...")
        self.get_logger().info(f"zone list: {self.item_zones.data}")
        self.get_logger().info(f"item holder: {self.item_holder}")
        for zone in self.item_zones.data:
            if self.item_holder.item_colour == zone.item_colour:
                return Point(x= float(zone.zone_x), y= float(zone.zone_y))
            
        closest_zone = self.zone_goals[0]
        shortest_distance = math.sqrt((closest_zone.x-self.current_pose.pose.position.x)**2 + 
                                      (closest_zone.y-self.current_pose.pose.position.y)**2)
        for zone in self.zone_goals:
            new_distance = math.sqrt((zone.x-self.current_pose.pose.position.x)**2 + 
                                     (zone.y-self.current_pose.pose.position.y)**2)
            if new_distance < shortest_distance:
                closest_zone = zone
                shortest_distance = new_distance  
        #todo: store zones not just points so can be removed when items added
        self.get_logger().info(f"closest zone: {closest_zone}")
        return closest_zone
    
    def update_item_zones(self, item_holder):
        for item_zone in self.item_zones.data:
            if item_zone.item_colour == item_holder.item_colour:
                item_zone.x = self.current_pose.pose.position.x
                item_zone.y = self.current_pose.pose.position.y
                item_zone.zone = self.zone.zone
                return
        self.item_zones.data.append(ItemZone(
            item_colour = item_holder.item_colour, 
            zone_y = self.current_pose.pose.position.y, 
            zone_x = self.current_pose.pose.position.x,
            zone = self.zone.zone))
        
        #self.zone_goals.remove(Point(self.zone.x, self.zone.y))

    def get_angle_to_origin(self, x, y):
        theta = math.atan2(x,y)
        self.get_logger().info(f"x,y: ({x:.2f},{y:.2f})")
        self.get_logger().info(f"theta: ({theta:.2f})")
        return (theta)

    def create_goal_pose(self, goal):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()                
        goal_pose.pose.position = goal

        angle = self.get_angle_to_origin(goal.x, goal.y)

        (goal_pose.pose.orientation.x,
         goal_pose.pose.orientation.y,
         goal_pose.pose.orientation.z,
         goal_pose.pose.orientation.w) = (quaternion_from_euler(0, 0, float(0), axes='sxyz'))

        self.get_logger().info(f"Goal pose: {goal_pose}")
        return goal_pose
   
    def should_redirect_to_item(self):
        return (not self.item_holder.holding_item and len(self.items.data) > 0 and not self.navigating_to_item)

    def destroy_node(self):
        self.get_logger().info(f"Shutting down")
        self.navigator.lifecycleShutdown()
        self.navigator.destroyNode()
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args)

    node = RobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    except Exception as e:
        node.get_logger().info(f'Exception: {e}')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()