from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl  # Assuming SuctionCupControl service is available for suction cup control

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

class PTP_MOVE(Node):

    def __init__(self, name_suffix = ""):
        super().__init__(f'dobot_PTP_client_1{name_suffix}')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')

        # Create a client for the suction cup control service
        self._suction_cup_client = self.create_client(SuctionCupControl, '/dobot_suction_cup_service')
        while not self._suction_cup_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for suction cup control service...')

    def cancel_done(self, future): 
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')
        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self._goal_handle = goal_handle

        self.get_logger().info('Goal accepted :)')

        # Start a 0.5 second timer
        # self._timer = self.create_timer(0.5, self.timer_callback)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.current_pose))

    def timer_callback(self):
        self.get_logger().info('Canceling goal')
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        # Cancel the timer
        self._timer.cancel()

    def send_goal(self, target, mode):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = target
        goal_msg.motion_type = mode
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def control_suction_cup(self, enable):
        request = SuctionCupControl.Request()
        
        # Adjust this line according to the correct field found in the service definition
        request.enable_suction = enable  # Replace 'suction_state' with the correct field name
        self._suction_cup_client.call_async(request)
        self.get_logger().info('Suction cup {}...'.format('enabled' if enable else 'disabled'))

def blue_back_panel():
    action_client = PTP_MOVE(name_suffix="blue_back")

    # Blue Back Panel UP
    action_client.send_goal(target = [51.1, 138.7, 69.0, 70.0], mode = 1)
    time.sleep(2)
    
    # Blue Back Panel
    action_client.send_goal(target = [51.1, 138.7, -37.0, 70.0], mode = 1)
    time.sleep(2)

    action_client.control_suction_cup(True)  # Turn on suction cup
    time.sleep(2)

    # Blue Back Panel UP
    action_client.send_goal(target = [51.1, 138.7, 69.0, 70.0], mode = 1)
    time.sleep(2)

    # Home Down
    action_client.send_goal(target = [126.2, 0.0, 0.0, 90.0], mode = 1)
    time.sleep(2)

    action_client.control_suction_cup(False)  # Turn on suction cup
    time.sleep(2)

def blue_board_panel():
    action_client = PTP_MOVE(name_suffix="blue_board")

    # Blue Board Panel UP
    action_client.send_goal(target = [55.4, 204.2, 69.0, 74.8], mode = 1)
    time.sleep(2)
    
    action_client.control_suction_cup(True)  # Turn on suction cup
    time.sleep(2)
    
    # Blue Board Panel
    action_client.send_goal(target = [55.4, 204.2, -38.0, 74.8], mode = 1)
    time.sleep(2)

    # Blue Board Panel UP
    action_client.send_goal(target = [55.4, 204.2, 69.0, 74.8], mode = 1)
    time.sleep(2)

    # Home Down
    action_client.send_goal(target = [126.2, 0.0, 0.0, 90.0], mode = 1)
    time.sleep(2)

    action_client.control_suction_cup(False)  # Turn on suction cup
    time.sleep(2)

def white_back_panel():
    action_client = PTP_MOVE(name_suffix="white_back")

    # White Back Panel UP
    action_client.send_goal(target = [91.0, 144.6, 69.0, 57.8], mode = 1)
    time.sleep(2)
    
    # White Back Panel
    action_client.send_goal(target = [91.0, 144.6, -37.0, 57.8], mode = 1)
    time.sleep(2)

    action_client.control_suction_cup(True)  # Turn on suction cup
    time.sleep(2)

    # White Back Panel UP
    action_client.send_goal(target = [91.0, 144.6, 69.0, 57.8], mode = 1)
    time.sleep(2)

    # Home Down
    action_client.send_goal(target = [126.2, 0.0, 0.0, 90.0], mode = 1)
    time.sleep(2)

    action_client.control_suction_cup(False)  # Turn on suction cup
    time.sleep(2)

def white_board_panel():
    action_client = PTP_MOVE(name_suffix="white_board")

    # White Board Panel UP
    action_client.send_goal(target = [93.2, 211.5, 69.0, 66.2], mode = 1)
    time.sleep(2)
    
    # White Board Panel
    action_client.send_goal(target = [93.2, 211.5, -38.0, 66.2], mode = 1)
    time.sleep(2)

    action_client.control_suction_cup(True)  # Turn on suction cup
    time.sleep(2)

    # White Board Panel UP
    action_client.send_goal(target = [93.2, 211.5, 69.0, 66.2], mode = 1)
    time.sleep(2)

    # Home Down
    action_client.send_goal(target = [126.2, 0.0, 0.0, 90.0], mode = 1)
    time.sleep(2)

    action_client.control_suction_cup(False)  # Turn on suction cup
    time.sleep(2)

def red_back_panel():
    action_client = PTP_MOVE(name_suffix="red_back")

    # Red Back Panel UP
    action_client.send_goal(target = [130.5, 153.2, 69.0, 49.6], mode = 1)
    time.sleep(2)
    
    # Red Back Panel
    action_client.send_goal(target = [130.5, 153.2, -38.0, 49.6], mode = 1)
    time.sleep(2)

    action_client.control_suction_cup(True)  # Turn on suction cup
    time.sleep(2)

    # Red Back Panel UP
    action_client.send_goal(target = [130.5, 153.2, 69.0, 49.6], mode = 1)
    time.sleep(2)

    # Home Down
    action_client.send_goal(target = [126.2, 0.0, 0.0, 90.0], mode = 1)
    time.sleep(2)

    action_client.control_suction_cup(False)  # Turn on suction cup
    time.sleep(2)

def red_board_panel():
    action_client = PTP_MOVE(name_suffix="red_board")

    # Red Board Panel UP
    action_client.send_goal(target = [137.0, 217.9, 69.0, 57.8], mode = 1)
    time.sleep(2)
    
    # Red Board Panel
    action_client.send_goal(target = [137.0, 217.9, -37.0, 57.8], mode = 1)
    time.sleep(2)

    action_client.control_suction_cup(True)  # Turn on suction cup
    time.sleep(2)

    # Red Board Panel UP
    action_client.send_goal(target = [137.0, 217.9, 69.0, 57.8], mode = 1)
    time.sleep(2)

    # Home Down
    action_client.send_goal(target = [126.2, 0.0, 0.0, 90.0], mode = 1)
    time.sleep(2)

    action_client.control_suction_cup(False)  # Turn on suction cup
    time.sleep(2)

def main(args=None):

    rclpy.init(args=args)

    action_client = PTP_MOVE()

    # Home
    action_client.send_goal(target = [126.2, 0.0, 69.0, 0.0], mode = 1)
    time.sleep(1)

    blue_board_panel()

    # white_board_panel()

    # red_board_panel()

    # Home
    action_client.send_goal(target = [126.2, 0.0, 69.0, 0.0], mode = 1)
    time.sleep(1)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()