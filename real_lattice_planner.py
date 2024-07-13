#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import rospy
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseStamped, Point32
from nav_msgs.msg import Odometry,Path
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from morai_msgs.msg import BoundingBox, BoundingBoxes
from tf2_msgs.msg import TFMessage

class latticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        # (1) subscriber, publisher 선언
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber('/tf', TFMessage, self.odom_callback) # run gpsimu_parser.py
        self.cluster_sub = rospy.Subscriber("/cluster_points", PointCloud2, self.cluster_callback)
        #rospy.Subscriber('/bounding_boxes', BoundingBoxes, self.detection_callback)
        
        self.is_detection= False
        self.detection_msg = None

        self.half_circle_pub = rospy.Publisher('/half_circle', Path, queue_size = 1)
        
        self.is_avoiding=False
        self.is_path = False
        self.is_status = False
        self.is_cluster = False
        self.obj_position=0
        self.direction = 1
        self.x_increasing=1.8    # 1.8
        self.is_x_increasing=True
        self.r = 5  # 5

        self.local_ego_vehicle_position_y=0

        rate = rospy.Rate(20) # 30hz
        while not rospy.is_shutdown():

            if self.is_path and self.is_status and self.is_cluster:
                if self.checkObject(self.cluster_data,self.odom_msg) or self.is_avoiding: # 내 앞에 장애물이 있거나 회피중이라면
                    self.latticePlanner(self.local_path, self.odom_msg) # 주행 영역에 장애물이 있을 경우 회피 주행
                else: # 장애물이 없다면 기존 local path 사용
                    self.half_circle_pub.publish(self.local_path)
            rate.sleep()

    def checkObject(self, cluster_data, odom):

        is_Object = False
        # TFMessage 내의 모든 TransformStamped 메시지 순회
        for transform in odom.transforms:
            # 필요한 경우 특정 frame_id에 대해서만 처리
            if transform.child_frame_id == "ego_vehicle":
                # translation의 x와 y 값을 저장
                vehicle_pose_x = transform.transform.translation.x
                vehicle_pose_y = transform.transform.translation.y
                
        # PointCloud2에서 포인트를 읽습니다.
        for point in pc2.read_points(cluster_data, field_names=("x", "y"), skip_nans=True):
            dis = sqrt(pow(vehicle_pose_x - point[0], 2) + pow(vehicle_pose_y - point[1], 2))
            if dis < 8 :
                is_Object = True
                self.is_avoiding=True
                self.obj_position=point
                break

        return is_Object

    def path_callback(self,msg):
        self.is_path = True
        self.local_path = msg  
        
    def odom_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status = True
        self.odom_msg = msg

    def cluster_callback(self,msg):
        self.is_cluster = True
        self.cluster_data = msg
        
    # def detection_callback(self,msg):
    #     self.is_detection=True
    #     self.detection_msg=msg
        
    # def check_car_detected(self):
    #     if self.detection_msg is None:
    #         return False  # 아직 감지된 메시지가 없으면 False 반환

    #     for bbox in self.detection_msg.bounding_boxes:
    #         if bbox.Class == 'car':
    #             return True  # 빨간 신호등 발견

    #     return False  # 빨간 신호등이 없으면 False 반환
    
    # def check_object_detected(self):
    #     if self.detection_msg is None:
    #         return False  # 아직 감지된 메시지가 없으면 False 반환

    #     for bbox in self.detection_msg.bounding_boxes:
    #         if bbox.Class == 'object':
    #             return True  # 빨간 신호등 발견

    #     return False  # 빨간 신호등이 없으면 False 반환

    def latticePlanner(self,ref_path, odom): # 입력으로 local path와  차량 상태
        # TFMessage 내의 모든 TransformStamped 메시지 순회
        for transform in odom.transforms:
            # 필요한 경우 특정 frame_id에 대해서만 처리
            if transform.child_frame_id == "ego_vehicle":
                # translation의 x와 y 값을 저장
                vehicle_pose_x = transform.transform.translation.x
                vehicle_pose_y = transform.transform.translation.y
        
        # 전역 좌표계 기준으로 지역 경로에서의 현재 위치
        global_ref_current_point      = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
        global_ref_current_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)
        #print(f'local_path_x: {ref_path.poses[0].pose.position.x}, local_path_y: {ref_path.poses[0].pose.position.y}')

        # 선택된  local path의 진행 방향에 대한 각도 계산
        theta = atan2(global_ref_current_next_point[1] - global_ref_current_point[1],
                      global_ref_current_next_point[0] - global_ref_current_point[0])  # 지역 경로의 방향 (전역 좌표계 기준)
        translation = [global_ref_current_point[0], global_ref_current_point[1]] # 지역 경로 이동 (전역 좌표계 기준)

        trans_matrix    = np.array([    [cos(theta), -sin(theta),  translation[0]], 
                                        [sin(theta),  cos(theta),  translation[1]], 
                                        [         0,           0,              1 ]     ])

        det_trans_matrix = np.array([   [trans_matrix[0][0], trans_matrix[1][0],        -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                                        [trans_matrix[0][1], trans_matrix[1][1],        -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                        [                 0,                  0,                                                                                   1]     ])
        global_obj_position = np.array([[self.obj_position[0]], [self.obj_position[1]], [1]]) # 전역 좌표계 기준 장애물 위치
        local_obj_position = det_trans_matrix.dot(global_obj_position) # local path 기준 장애물 위치
        #print(f'global_obj_position: {global_obj_position}') # [[196.70239258]]
        #print(f'local_obj_position: {local_obj_position}') # [[196.70239258]]
        
        global_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]]) # 전역 좌표계 기준
        local_ego_vehicle_position = det_trans_matrix.dot(global_ego_vehicle_position) # local path 기준으로 좌표 생성
        self.local_ego_vehicle_position_y=local_ego_vehicle_position[1][0]
        
        local_path_msg=Path()
        local_path_msg.header.frame_id='/map'
        
        
        # if self.check_car_detected() and self.is_x_increasing:
        #     self.x_increasing = 1.8
        #     self.r = 5
        #     self.is_x_increasing = False
        # elif self.check_object_detected() and self.is_x_increasing:
        #     self.x_increasing = 1.2
        #     self.r = 4
        #     self.is_x_increasing = False
                
        # 장애물을 중심으로 반타원 모양 처럼 회피 경로를 얻어서 회피할거임
        # 초기 설정
        # 시작 좌표 (장애물 뒤쪽에서 시작)
        x = -self.r
        y = 0
        
        angle_between_obstacle_and_ego_vehicle = abs(atan2(local_obj_position[0][0] - local_ego_vehicle_position[0][0], 
                                                           local_obj_position[1][0] - local_ego_vehicle_position[1][0])) * pi/180.0
        

        # 회피 경로 생성
        for i in range(37):  # 180도 회전
            theta = (pi / 36) * i
            if angle_between_obstacle_and_ego_vehicle <= theta:
                path_y = y + local_obj_position[1][0]
                if path_y >= 2.8:            # 왼쪽 차선을 넘어가지 않도록 최대값 설정
                    path_y = 2.8
                pose = np.array([[x + local_obj_position[0][0]], [path_y], [1]])
                global_pose = trans_matrix.dot(pose)  # 전역 좌표계로 변환

                # ROS 메시지 업데이트
                tmp_pose = PoseStamped()
                tmp_pose.pose.position.x = global_pose[0][0]
                tmp_pose.pose.position.y = global_pose[1][0]
                tmp_pose.pose.orientation.w = 1
                local_path_msg.poses.append(tmp_pose)

                # 현재 좌표를 회전 행렬로 업데이트하면서 x, y를 타원의 방정식에 맞게 조정
                if i > 18:  # 절반 이상 지난 후 x값 증가율 변경
                    x = -self.r * cos(theta) * self.x_increasing  # x 값 증가율
                else:
                    x = -self.r * cos(theta)
                y = self.direction * 2.5 * sin(theta)
        
        # 마지막 5개 포즈의 좌표 추출
        last_poses = local_path_msg.poses[-10:]
        for pose in last_poses:
            last_x = pose.pose.position.x
            last_y = pose.pose.position.y
            # 거리 계산
            dis = sqrt(pow(vehicle_pose_x - last_x, 2) + pow(vehicle_pose_y - last_y, 2))
            if dis <= 1:
                if self.is_avoiding == True:
                    self.is_avoiding = False  # 회피 끝났으니 다시 local_path로 복귀
                    self.is_x_increasing = True
                    break

        self.half_circle_pub.publish(local_path_msg)

if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass

