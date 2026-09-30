import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from robot_navigation_action.action import Navigate

STATE_GO_TO_GOAL = 0
STATE_WALL_FOLLOW = 1
STATE_ROTATE_TO_FINAL = 2

TARGET_WALL_DIST = 0.4   
SAFE_FRONT_DIST = 0.5    

KP = 3.5   
KD = 15.0  

BASE_LINEAR_SPEED = 0.3   
MAX_ANGULAR_SPEED = 1.2   

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom_fake', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        

        self.scan_ranges = []
        self.scan_angle_min = 0.0
        self.scan_angle_inc = 0.0

        self.prev_error = 0.0
        self.last_loop_time = time.time()


        self.state = STATE_GO_TO_GOAL
        self.start_point = (0.0, 0.0) 
        self.hit_point = None         
        self.hit_dist_to_goal = float('inf')

        self.get_logger().info("Navigation server started.")

    # --- CALLBACKS ---

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        self.scan_ranges = [r if not math.isinf(r) and not math.isnan(r) and r >= msg.range_min else msg.range_max for r in msg.ranges]
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    # NAV

    async def execute_callback(self, goal_handle):
        tx, ty = goal_handle.request.x, goal_handle.request.y
        ttheta = math.radians(goal_handle.request.theta)
        
        self.get_logger().info(f'Navigation goal received: ({tx}, {ty})')

        self.start_point = (self.current_x, self.current_y)
        
        self.state = STATE_GO_TO_GOAL
        self.hit_point = None
        self.prev_error = 0.0 
        
        feedback = Navigate.Feedback()
        result = Navigate.Result()
        rate = self.create_rate(20) 

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_robot()
                return result

            dist_to_goal = math.sqrt((tx - self.current_x)**2 + (ty - self.current_y)**2)
            

            feedback.distance_to_goal = dist_to_goal
                
            goal_handle.publish_feedback(feedback)

            if not self.scan_ranges:
                rate.sleep()
                continue

            # STATES
            
            if self.state == STATE_GO_TO_GOAL:
                if dist_to_goal < 0.15:
                    self.stop_robot()
                    self.state = STATE_ROTATE_TO_FINAL
                
                elif self.is_obstacle_ahead():
                    self.get_logger().info(f"Obstacle at {dist_to_goal:.2f}m. Following Wall.")
                    self.state = STATE_WALL_FOLLOW
                    self.hit_point = (self.current_x, self.current_y)
                    self.hit_dist_to_goal = dist_to_goal
                    self.prev_error = 0.0 
                
                else:
                    self.go_to_goal(tx, ty)

            elif self.state == STATE_WALL_FOLLOW:
                dist_from_hit = math.sqrt((self.current_x - self.hit_point[0])**2 + 
                                        (self.current_y - self.hit_point[1])**2)
                
                dist_to_m_line = self.distance_to_line(self.current_x, self.current_y, 
                                                     self.start_point[0], self.start_point[1], 
                                                     tx, ty)
                
                is_closer = dist_to_goal < self.hit_dist_to_goal

                if dist_from_hit > 0.5 and dist_to_m_line < 0.15 and is_closer:
                    if not self.is_obstacle_ahead():
                        self.get_logger().info(f"Crossed M-Line closer to goal ({dist_to_goal:.2f}m). Leaving Wall.")
                        self.state = STATE_GO_TO_GOAL
                        continue

                self.follow_wall_pd()

            elif self.state == STATE_ROTATE_TO_FINAL:
                if self.rotate_to_heading(ttheta):
                    self.stop_robot()
                    goal_handle.succeed()
                    result.success = True
                    return result

            rate.sleep()


    def distance_to_line(self, p_x, p_y, start_x, start_y, end_x, end_y):
        numerator = abs((end_x - start_x) * (start_y - p_y) - (start_x - p_x) * (end_y - start_y))
        denominator = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        
        if denominator == 0: return float('inf')
        return numerator / denominator

    def follow_wall_pd(self):
        msg = Twist()
        current_time = time.time()
        dt = current_time - self.last_loop_time
        if dt <= 0: dt = 0.05
        self.last_loop_time = current_time

        d_left = self.get_min_in_sector(70, 110)
        d_front = self.get_min_in_sector(-10, 10)
        d_fleft = self.get_min_in_sector(20, 60)
        
        current_dist = min(d_left, d_fleft * 1.2)
        
        error = current_dist - TARGET_WALL_DIST
        d_term = (error - self.prev_error) / dt
        self.prev_error = error

        angular_z = (KP * error) + (KD * d_term)

        # Panic mode
        if d_front < SAFE_FRONT_DIST:
            angular_z = -MAX_ANGULAR_SPEED
            linear_x = 0.05
        else:
            # Dynamic speed
            turn_severity = abs(angular_z) / MAX_ANGULAR_SPEED
            linear_x = BASE_LINEAR_SPEED * (1.0 - 0.8 * min(turn_severity, 1.0))
            linear_x = max(linear_x, 0.05)

        angular_z = max(min(angular_z, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)

        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def go_to_goal(self, tx, ty):
        desired_yaw = math.atan2(ty - self.current_y, tx - self.current_x)
        err_yaw = self.normalize_angle(desired_yaw - self.current_yaw)
        
        msg = Twist()
        if abs(err_yaw) > 1.5:
            msg.linear.x = 0.0
            msg.angular.z = 0.8 if err_yaw > 0 else -0.8
        else:
            msg.linear.x = BASE_LINEAR_SPEED
            msg.angular.z = 2.0 * err_yaw 
            msg.angular.z = max(min(msg.angular.z, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)
        self.cmd_vel_pub.publish(msg)

    def rotate_to_heading(self, target_yaw):
        err = self.normalize_angle(target_yaw - self.current_yaw)
        if abs(err) < 0.05: return True
        
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.5 if err > 0 else -0.5
        self.cmd_vel_pub.publish(msg)
        return False

    def is_obstacle_ahead(self):
        return self.get_min_in_sector(-15, 15) < SAFE_FRONT_DIST

    def get_min_in_sector(self, start_deg, end_deg):
        if not self.scan_ranges: return float('inf')
        start_rad, end_rad = math.radians(start_deg), math.radians(end_deg)
        idx_start = int((start_rad - self.scan_angle_min) / self.scan_angle_inc)
        idx_end = int((end_rad - self.scan_angle_min) / self.scan_angle_inc)
        N = len(self.scan_ranges)
        idx_start = max(0, min(idx_start, N-1))
        idx_end = max(0, min(idx_end, N-1))
        if idx_start > idx_end: idx_start, idx_end = idx_end, idx_start
        subset = self.scan_ranges[idx_start:idx_end+1]
        return min(subset) if subset else float('inf')

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
