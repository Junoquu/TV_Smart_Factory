import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import cv2
import numpy as np

import pyrealsense2 as rs

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import torch

class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__('realsense_yolov5_node')

        # YOLOv5 모델 로드
        self.yolo_model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/ssafy/final_project/src/final_project/best.pt')
        # self.yolo_model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

        # RealSense 카메라 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # 퍼블리셔 및 브리지 설정
        self.detection_publisher = self.create_publisher(String, '/detection_results', 10)
        self.image_publisher = self.create_publisher(Image, 'detection_image', 10)
        
        self.bridge = CvBridge()

        # 타이머 콜백 설정
        self.timer = self.create_timer(0.1, self.timer_callback)

        # ROI 설정
        self.roi_position = (278, 79, 490, 351)

    def get_color_name(self, hsv_color):
        h, s, v = hsv_color
        if s < 60 and v > 180:  # 흰색: 채도가 낮고 밝기가 높음
            return 'white'
        elif (h < 10 or h > 170) and s > 100 and v > 100:  # 빨간색
            return 'red'
        elif 100 < h < 130 and s > 100 and v > 100:  # 파란색
            return 'blue'
        return 'unknown'  # 나머지는 unknown으로 설정

    def get_color_bgr(self, color_name):
        if color_name == 'white':
            return (255, 255, 255)
        elif color_name == 'red':
            return (0, 0, 255)
        elif color_name == 'blue':
            return (255, 0, 0)
        return (0, 255, 0)  # 기본값은 초록색

    def get_center_color(self, image):
        height, width = image.shape[:2]
        center_y, center_x = height // 2, width // 2
        sample_size = min(width, height) // 4
        start_x = max(0, center_x - sample_size // 2)
        end_x = min(width, center_x + sample_size // 2)
        start_y = max(0, center_y - sample_size // 2)
        end_y = min(height, center_y + sample_size // 2)
        center_region = image[start_y:end_y, start_x:end_x]
        hsv_region = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
        average_color = np.mean(hsv_region, axis=(0, 1))
        
        return average_color

    def timer_callback(self):
        # RealSense 프레임 가져오기
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            return

        # Numpy 배열로 변환
        self.color_image = np.asanyarray(color_frame.get_data())

        roi_x1, roi_y1, roi_x2, roi_y2 = self.roi_position
        cv2.rectangle(self.color_image, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 0, 0), 2)  # 파란색 ROI 사각형

        roi_image = self.color_image[roi_y1:roi_y2, roi_x1:roi_x2]
                    
        # YOLOv5 추론
        results = self.yolo_model(roi_image)

        detected_objects = []

        # 바운딩 박스 그리기 및 결과 저장
        for result in results.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = map(int, result[:6])
            
            object_roi = self.color_image[roi_y1 + y1: roi_y1 + y2, roi_x1 + x1: roi_x1 + x2]
            center_color = self.get_center_color(object_roi)

            color_name = self.get_color_name(center_color)
            print("color_name: ", color_name)
            color_bgr = self.get_color_bgr(color_name)

            label = self.yolo_model.names[class_id]
            detected_objects.append(f"{label}-{color_name}")

            cv2.rectangle(self.color_image, (roi_x1+x1, roi_y1+y1), (roi_x1+x2, roi_y1+y2), (0, 255, 0), 2)
            cv2.putText(self.color_image, f'{label}-{color_name}', (roi_x1+x1, roi_y1+y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 객체 탐지 결과를 문자열로 변환하여 발행
        detection_result_msg = String()
        if detected_objects:
            detection_result_msg.data = ','.join(detected_objects)
        else:
            detection_result_msg.data = ''
        self.detection_publisher.publish(detection_result_msg)

        # 이미지 메시지 발행
        ros_image_message = self.bridge.cv2_to_imgmsg(self.color_image, encoding='bgr8')
        self.image_publisher.publish(ros_image_message)

        # 이미지 표시
        cv2.imshow('frame', self.color_image)
        cv2.waitKey(1)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYoloNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
