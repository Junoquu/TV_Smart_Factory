import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import socket
import threading

import time
import subprocess

class ObjectDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('objectDetectionsubscriber')

        self.subscription = self.create_subscription(
            String,
            '/detection_results',
            self.listener_callback,
            10
        )

        self.conv_server_conn = None  # Conveyor server connection
        self.robodk_server_conn = None
        self.remote_server_conn = None  # Remote server connection

        self.get_logger().info("Object Detection Subscriber has started.")

        # Conveyor server 스레드 시작
        server_thread = threading.Thread(target=self.start_conveyor_server)
        server_thread.daemon = True
        server_thread.start()

        robodk_server_thread = threading.Thread(target=self.start_robodk_server)
        robodk_server_thread.daemon = True
        robodk_server_thread.start()

        # Remote client 스레드 시작
        client_thread = threading.Thread(target=self.connect_to_robodk_server)
        client_thread.daemon = True
        client_thread.start()

    # Conveyor 서버
    def start_conveyor_server(self, host='192.168.110.105', port=65432):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((host, port))
            s.listen()
            self.get_logger().info(f'서버가 {host}:{port}에서 대기 중입니다...')

            while True:
                conn, addr = s.accept()
                self.get_logger().info(f'연결됨: {addr}')
                self.conv_server_conn = conn
                client_thread = threading.Thread(target=self.handle_conveyor_client, args=(conn,))
                client_thread.daemon = True
                client_thread.start()

    def handle_conveyor_client(self, conn):
        self.get_logger().info("컨베이어 클라이언트 핸들러 스레드 시작됨.")
        try:
            while True:
                time.sleep(1)
                if conn.fileno() == -1:
                    break
        except Exception as e:
            self.get_logger().error(f"클라이언트 연결 오류: {e}")
        self.conv_server_conn = None
        self.get_logger().info("컨베이어 클라이언트 연결 종료.")

    def start_robodk_server(self, host='192.168.110.105', port=65433):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((host, port))
            s.listen()
            self.get_logger().info(f"RoboDk 서버가 {host}:{port}에서 대기 중입니다...")
            
            while True:
                robodk, addr = s.accept()
                self.get_logger().info(f'robodk 연결됨 : {addr}')
                self.robodk_server_conn = robodk
                client_thread = threading.Thread(target=self.handle_robodk_client, args=(robodk,))
                client_thread.daemon = True
                client_thread.start()

    def handle_robodk_client(self, conn):
        self.get_logger().info("RoboDk 클라이언트 핸들러 스레드 시작됨.")
        try:
            while True:
                # time.sleep(1)
                if conn.fileno() == -1:
                    break
                data = conn.recv(1024)
                if not data:
                    break
                command = data.decode('utf-8').strip()
                self.get_logger().info(f"Remote 서버로부터 명령 수신: {command}")
                
                if command == "wait_sig1":
                    # 특정 작업 1 (주석 처리 위치)
                    self.get_logger().info("Signal 1 작업 실행 중...")
                    
                    self.perform_pick_and_place_board_panel_with_ros2()
                    # self.send_signal_to_remote(1)
                elif command == "wait_sig2":
                    # 특정 작업 2 (주석 처리 위치)
                    self.get_logger().info("Signal 2 작업 실행 중...")
                    
                    self.perform_pick_and_place_back_panel_with_ros2()
                    # self.send_signal_to_remote(2)
                else:
                    self.get_logger().error("알 수 없는 명령 수신.")
        except Exception as e:
            self.get_logger().error(f"클라이언트 연결 오류: {e}")
        self.robodk_server_conn = None
        self.get_logger().info("RoboDk 클라이언트 연결 종료.")

    def connect_to_robodk_server(self, host="192.168.26.51", port=65434):
        while True:
            try:
                self.remote_server_conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.remote_server_conn.connect((host, port))
                self.get_logger().info(f"RoboDK 서버와 연결 성공: {host}:{port}")
                break  # 연결 성공 시 루프 종료
            except socket.error as e:
                self.get_logger().error(f"RoboDK 서버와 연결 실패: {e}. 5초 후 재시도...")
                time.sleep(5)


    def send_signal_to_remote(self, signal):
        try:
            if self.remote_server_conn:
                self.remote_server_conn.sendall(str(signal).encode('utf-8'))
                self.get_logger().info(f"Signal {signal}을 Remote 서버로 전송.")
            else:
                self.get_logger().error("Remote 서버에 연결되어 있지 않습니다.")
        except Exception as e:

            self.get_logger().error(f"Signal 전송 실패: {e}")

    # 기존 함수들
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

        # back_panel의 색상 조합
        back_panel_variants = ['back_panel-red', 'back_panel-blue', 'back_panel-white']
        # board_panel의 색상 조합
        board_panel_variants = ['board_panel-red', 'board_panel-blue', 'board_panel-white']

        if any(obj in detected_objects for obj in back_panel_variants):
            self.wait_for_conveyor_server_connection()
            self.perform_task_for_object("back_panel")
        elif any(obj in detected_objects for obj in board_panel_variants):
            self.wait_for_conveyor_server_connection()
            self.perform_task_for_object("board_panel")
        else:
            self.get_logger().info("해당 조건에 부합하는 객체가 없습니다.")

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

        if object_name == 'back_panel':
            self.perform_task_back_panel()
        elif object_name == 'board_panel':
            self.perform_task_board_panel()

        self.detection_buffer = []

    def perform_task_back_panel(self):
        self.get_logger().info(f'백 패널에 대한 작업 실행')
        self.wait_for_conveyor_server_connection()

        self.send_command_to_conveyor('seperator', 3)

    def perform_task_board_panel(self):
        self.get_logger().info(f'보드 패널에 대한 작업 실행')
        self.wait_for_conveyor_server_connection()

        self.send_command_to_conveyor('seperator', 4)

    def wait_for_conveyor_server_connection(self, timeout=10):
        start_time = time.time()

        while self.conv_server_conn is None and time.time() - start_time < timeout:
            self.get_logger().info("서버 연결을 기다리는 중...")
            time.sleep(1)

        if self.conv_server_conn is None:
            self.get_logger().error("시간 내에 서버 연결을 확립하지 못했습니다.")
        else:
            self.get_logger().info("컨베이어 서버 연결이 확립되었습니다.")

    def perform_pick_and_place_back_panel_with_ros2(self):
        try:
            # ROS 2 명령어 실행
            result = subprocess.run(
                ['ros2', 'run', 'final_project', 'ptp_move_back'],  # 실행할 ros2 명령
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
            # 결과 로깅
            if result.returncode == 0:
                self.get_logger().info(f"Dobot 작업 성공적으로 완료: {result.stdout}")
            else:
                self.get_logger().error(f"Dobot 작업 오류: {result.stderr}")
        except Exception as e:
            self.get_logger().error(f"ROS 2 Dobot 명령 실행 실패: {e}")
    
    def perform_pick_and_place_board_panel_with_ros2(self):
        try:
            # ROS 2 명령어 실행
            result = subprocess.run(
                ['ros2', 'run', 'final_project', 'ptp_move_board'],  # 실행할 ros2 명령
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
            # 결과 로깅
            if result.returncode == 0:
                self.get_logger().info(f"Dobot 작업 성공적으로 완료: {result.stdout}")
            else:
                self.get_logger().error(f"Dobot 작업 오류: {result.stderr}")
        except Exception as e:
            self.get_logger().error(f"ROS 2 Dobot 명령 실행 실패: {e}")

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

