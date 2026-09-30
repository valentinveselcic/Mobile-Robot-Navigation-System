import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_navigation_action.action import Navigate

ACTION_TIMEOUT_SEC = 120  

class NavigationClient(Node):

    def __init__(self):
        super().__init__('navigation_action_client')
        self._action_client = ActionClient(self, Navigate, 'navigate')

    def send_goal(self, x, y, theta):
        goal_msg = Navigate.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.theta = float(theta)

        self.get_logger().info(f'Sending goal: X={x}, Y={y}, Theta={theta}')
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        try:
            dist = feedback.distance_to_goal
        except AttributeError:
            dist = getattr(feedback, 'distance_to_goal', -1.0)
            
        self.get_logger().info(f'Distance to goal: {dist:.2f} m')

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().info('Goal failed or aborted.')
        
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = NavigationClient()

    print("--- Navigation Action Client ---")
    try:
        x_in = input("Enter X [m] (-14.0 to 14.0): ")
        y_in = input("Enter Y [m] (-7.0 to 7.0): ")
        theta_in = input("Enter Theta [deg]: ")
        
        action_client.send_goal(x_in, y_in, theta_in)
        
        # Spinaj dok ne završi
        rclpy.spin(action_client)
        
    except ValueError:
        print("Invalid input! Please enter numbers.")
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        action_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
