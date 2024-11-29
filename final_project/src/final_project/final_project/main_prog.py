import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import socket
import threading

import time

def start_conveyor_server(host='192.168.110.105', port=65432):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f'Server is listening on {host}: {port}....')

        conn, addr = s.accept()
        print(f'Connected by {addr}')
        return conn
    
def handle_conveyor_client(conn, machine, status):
    if machine == 'conv':
        if status == 'conv_run':
            command = '1'

            conn.sendall(command.encode('utf-8'))
            print(f'Sent command {command} to the client.')

        elif status == 'conv_stop':
            command = '2'
            conn.sendall(command.encode('utf-8'))
            print(f'Sent command {command} to the client.')

    elif machine == 'seperator':
        if status == 3:
            command = '3'
            conn.sendall(command.encode('utf-8'))
            print(f'Sent command {command} to the client.')

        elif status == 4:
            command = '4'
            conn.sendall(command.encode('utf-8'))
            print(f'Sent command {command} to the client.')

    else:
        print('Please check the machine name.')

class ObjectDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('objectDetectionsubscriber')

        self.subscription = self.create_subscription(
            String,
            '/detection_results',
            self.listener_callback,
            10
        )

        self.conv_server_conn = False
        
        self.detection_buffer = []
        # 방이 20개 방중에 12개가 차면 로직발동
        self.buffer_size = 20
        self.detection_threshold = 12

        self.get_logger().info("Object Detection Subscriber has started.")
        
        # Conveyor Server On
        server_thread = threading.Thread(target=start_conveyor_server)
        server_thread.daemon = True
        server_thread.start()

    def listener_callback(self, msg):
        detection_results = msg.data
        self.get_logger().info(f"Recieved detection results: {detection_results}")

        detected_objects = self.parse_detection_results(detection_results)

        if 'back_panel' in detected_objects:
            self.detection_buffer.append('back panel')

        elif 'board_panel' in detected_objects:
            self.detection_buffer.append('board panel')
        else:
            self.detection_buffer.append('None')
        
        if len(self.detection_buffer) > self.buffer_size:
            self.detection_buffer.pop(0)

        self.check_object_detection()
    
    def check_object_detection(self):
        if self.detection_buffer.count('back panel') >= self.detection_threshold:
            self.perform_task_for_object('back panel')

        elif self.detection_buffer.count('board panel') >= self.detection_threshold:
            self.perform_task_for_object('board panel')

    def parse_detection_results(self, detection_results):
        try:
            if not detection_results:
                self.get_logger().warn("Empty detection results recieved.")
                return []
            detected_objects = [obj.strip() for obj in detection_results.split(',')]
            return detected_objects
        
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            return []
        
    def perform_task_for_object(self, object_name):
        self.get_logger().info(f'{object_name} detected! Performing task...')

        if object_name == 'back panel':
            self.perform_task_back_panel()
        elif object_name == 'board panel':
            self.perform_task_board_panel()

        self.detection_buffer = []

    def perform_task_back_panel(self):
        self.get_logger().info(f'Executing task for back panel')
        self.wait_for_conveyor_server_connection()

        if self.conv_server_conn:
            handle_conveyor_client(self.conv_server_conn, 'seperator', 3)

        time.sleep(5)
    
    def perform_task_board_panel(self):
        self.get_logger().info(f'Executing task for board panel')
        self.wait_for_conveyor_server_connection()

        if self.conv_server_conn:
            handle_conveyor_client(self.conv_server_conn, 'conv', 'conv_stop')

        time.sleep(5)

    def start_conveyor_server_in_thread(self):
        self.conv_server_conn = start_conveyor_server()

    def wait_for_conveyor_server_connection(self, timeout=10):
        start_time = time.time()

        while self.conv_server_conn is None and time.time() - start_time < timeout:
            self.get_logger().info("Waiting for server connection...")
            time.sleep(1)

        if self.conv_server_conn is None:
            self.get_logger().error("Failed to establish server connection within timeout.")

        else:
            self.get_logger().info("Conveyor Server connection established.")
        
def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetectionSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()