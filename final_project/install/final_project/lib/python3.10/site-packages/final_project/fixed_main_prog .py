import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import socket
import threading

import time

class ObjectDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('objectDetectionsubscriber')

        self.subscription = self.create_subscription(
            String,
            '/detection_results',
            self.listener_callback,
            10
        )

        self.conv_server_conn = None
        
        self.detection_buffer = []
        # 버퍼 크기 설정
        self.buffer_size = 20
        self.detection_threshold = 12

        self.get_logger().info("Object Detection Subscriber has started.")
        
        # 서버 스레드 시작
        server_thread = threading.Thread(target=self.start_conveyor_server)
        server_thread.daemon = True
        server_thread.start()

    def start_conveyor_server(self, host='192.168.110.105', port=65432):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((host, port))
            s.listen()
            self.get_logger().info(f'서버가 {host}:{port}에서 대기 중입니다...')

            while True:
                conn, addr = s.accept()
                self.get_logger().info(f'연결됨: {addr}')
                self.conv_server_conn = conn
                # 클라이언트 처리를 위한 스레드 시작
                client_thread = threading.Thread(target=self.handle_conveyor_client, args=(conn,))
                client_thread.daemon = True
                client_thread.start()

    def handle_conveyor_client(self, conn):
        with conn:
            while True:
                try:
                    data = conn.recv(1024)
                    if not data:
                        break
                    # 수신된 데이터 처리 (필요한 경우)
                    self.get_logger().info(f"클라이언트로부터 수신: {data.decode('utf-8')}")
                except Exception as e:
                    self.get_logger().error(f"클라이언트 연결 오류: {e}")
                    break
        self.get_logger().info("클라이언트 연결이 종료되었습니다.")
        # 연결 변수 초기화
        self.conv_server_conn = None

    def send_command_to_conveyor(self, machine, status):
        if self.conv_server_conn:
            if machine == 'conv':
                if status == 'conv_run':
                    command = '1'
                elif status == 'conv_stop':
                    command = '2'
                else:
                    self.get_logger().error("conv에 대한 잘못된 상태입니다.")
                    return
            elif machine == 'seperator':
                if status == 3:
                    command = '3'
                elif status == 4:
                    command = '4'
                else:
                    self.get_logger().error("seperator에 대한 잘못된 상태입니다.")
                    return
            else:
                self.get_logger().error("잘못된 기계 이름입니다.")
                return

            try:
                self.conv_server_conn.sendall(command.encode('utf-8'))
                self.get_logger().info(f'명령 {command}을(를) 클라이언트에 전송했습니다.')
            except Exception as e:
                self.get_logger().error(f"명령 전송 실패: {e}")
                self.conv_server_conn = None
        else:
            self.get_logger().error("활성화된 컨베이어 클라이언트 연결이 없습니다.")

    def listener_callback(self, msg):
        detection_results = msg.data
        self.get_logger().info(f"수신된 감지 결과: {detection_results}")

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
                self.get_logger().warn("빈 감지 결과를 수신했습니다.")
                return []
            detected_objects = [obj.strip() for obj in detection_results.split(',')]
            return detected_objects
        except Exception as e:
            self.get_logger().error(f'예기치 않은 오류: {e}')
            return []
        
    def perform_task_for_object(self, object_name):
        self.get_logger().info(f'{object_name} 감지됨! 작업 수행 중...')

        if object_name == 'back panel':
            self.perform_task_back_panel()
        elif object_name == 'board panel':
            self.perform_task_board_panel()

        self.detection_buffer = []

    def perform_task_back_panel(self):
        self.get_logger().info(f'백 패널에 대한 작업 실행')
        self.wait_for_conveyor_server_connection()

        self.send_command_to_conveyor('seperator', 3)

        time.sleep(5)
    
    def perform_task_board_panel(self):
        self.get_logger().info(f'보드 패널에 대한 작업 실행')
        self.wait_for_conveyor_server_connection()

        self.send_command_to_conveyor('conv', 'conv_stop')

        time.sleep(5)

    def wait_for_conveyor_server_connection(self, timeout=10):
        start_time = time.time()

        while self.conv_server_conn is None and time.time() - start_time < timeout:
            self.get_logger().info("서버 연결을 기다리는 중...")
            time.sleep(1)

        if self.conv_server_conn is None:
            self.get_logger().error("시간 내에 서버 연결을 확립하지 못했습니다.")
        else:
            self.get_logger().info("컨베이어 서버 연결이 확립되었습니다.")
        
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
