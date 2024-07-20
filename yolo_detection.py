#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
from morai_msgs.msg import BoundingBox, BoundingBoxes # Adjust the import to your ROS package structure

class YoloNode:
    def __init__(self):
        rospy.init_node('camera_detection', anonymous=True)
        self.bridge = CvBridge()
        self.model = YOLO('/home/a/Wls_final_yolo.pt')
        self.bbox_publisher = rospy.Publisher('/bounding_boxes', BoundingBoxes, queue_size=1)
        rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.image_callback)
        rospy.spin()

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            results = self.model(cv_image)[0]
            bounding_boxes = []

            for box in results.boxes:
                conf = box.conf[0]  # 확률
                if conf >= 0.6:  # 확률이 0.8 이상인 경우만 처리
                    x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                    cls = box.cls[0].item()  # 객체명
                    label = self.model.names[int(cls)]
                    labell = f'{self.model.names[int(cls)]} {conf:.2f}'

                    # Bounding Box 메시지 생성
                    bbox = BoundingBox()
                    bbox.probability = float(conf)
                    bbox.xmin = int(x1)
                    bbox.ymin = int(y1)
                    bbox.xmax = int(x2)
                    bbox.ymax = int(y2)
                    bbox.id = int(cls)
                    bbox.Class = label
                    bounding_boxes.append(bbox)
                    
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, labell, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("YOLOv8 Detection", cv_image)
            cv2.waitKey(1)

            # BoundingBoxes 메시지 발행
            bboxes_msg = BoundingBoxes()
            bboxes_msg.header.stamp = rospy.Time.now()
            bboxes_msg.header.frame_id = "camera_frame"
            bboxes_msg.bounding_boxes = bounding_boxes
            self.bbox_publisher.publish(bboxes_msg)

        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

if __name__ == '__main__':
    try:
        YoloNode()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
