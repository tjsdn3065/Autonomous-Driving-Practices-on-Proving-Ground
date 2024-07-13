#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import os
import rospy
import numpy as np
import cv2

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from nav_msgs.msg import Odometry,Path
from tf.transformations import euler_from_quaternion
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point
from morai_msgs.msg import BoundingBox, BoundingBoxes
from tf2_msgs.msg import TFMessage

class SCANCluster:
    def __init__(self):
        rospy.init_node('velodyne_clustering', anonymous=True)
        rospy.Subscriber('/tf', TFMessage, self.odom_callback) # 장애물을 전역 좌표계 기준으로 표시할거임
        rospy.Subscriber("/local_path", Path, self.path_callback) # 경로에 있는 물체만 클러스터링 할거임
        #rospy.Subscriber('/bounding_boxes', BoundingBoxes, self.detection_callback)
        self.scan_sub = rospy.Subscriber("/ouster/points", PointCloud2, self.scan_callback)
        self.clusterpoints_pub = rospy.Publisher("/cluster_points", PointCloud2, queue_size=10)
        
        self.is_detection= False
        self.detection_msg = None
        
        self.pc_np = None
        self.dbscan = DBSCAN(eps=2.0, min_samples=10) # eps(반경), min_samples(군집이 되기 위한 최소 점)

        self.is_path = False
        self.is_odom = False
        self.current_postion = Point()

    def odom_callback(self,msg):
        self.is_odom=True
        # TFMessage 내의 모든 TransformStamped 메시지 순회
        for transform in msg.transforms:
            # 필요한 경우 특정 frame_id에 대해서만 처리
            if transform.child_frame_id == "ego_vehicle":
                odom_quaternion=(transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w)
                _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
                # translation의 x와 y 값을 저장
                self.current_postion.x = transform.transform.translation.x
                self.current_postion.y = transform.transform.translation.y
        
    def path_callback(self,msg):
        self.is_path = True
        self.local_path = msg
        
    # def detection_callback(self,msg):
    #     self.is_detection=True
    #     self.detection_msg=msg
        
    # def check_object_or_car_detected(self):
    #     """감지된 메시지에서 'object' 또는 'car'가 있는지 확인하고 결과를 반환하는 함수"""
    #     if self.detection_msg is None:
    #         return False  # 아직 감지된 메시지가 없으면 False 반환

    #     for bbox in self.detection_msg.bounding_boxes:
    #         if bbox.Class == 'object' or bbox.Class == 'car':
    #             return True  # 'object' 또는 'car' 발견

    #     return False  # 'object' 또는 'car'가 없으면 False 반환
        
        
    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])
            if point[0] > 0 and 1.50 > point[2] > -0.5 and dist < 50:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))

        point_np = np.array(point_list, np.float32)
        return point_np
        
            
    def publish_point_cloud(self, points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        # Create PointCloud2 message
        pc2_msg = pc2.create_cloud(header, fields, points)

        # Publish PointCloud2 message
        self.clusterpoints_pub.publish(pc2_msg)

    def scan_callback(self, msg):
        cluster_points = []
        if True:
            self.pc_np = self.pointcloud2_to_xyz(msg)
            if len(self.pc_np) == 0:
                return
            
            ref_path=self.local_path
            
            
            vehicle_position=self.current_postion
            
            translation=[vehicle_position.x, vehicle_position.y]
            lidar_to_gps = [0.0, -0.18, 0.1]  # 라이다에서 GPS로 이동 # 실차에서는  0, -0.18, 0.1

            lidar_to_gps_transform = np.array([
                [1, 0, lidar_to_gps[0]],
                [0, 1, lidar_to_gps[1]],
                [0, 0,               1]
            ])

            # GPS에서 전역 좌표계로의 변환
            gps_to_global = np.array([
                [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
                [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
                [0, 0, 1]
            ])      

            pc_xy = self.pc_np[:, :2]
            angles = np.arctan2(pc_xy[:, 1], pc_xy[:, 0])  # y축과 x축의 비율로 각도 계산
            angle_filter = (angles > -np.pi/18) & (angles < np.pi/18)  # -45도에서 +45도 사이 필터

            pc_xy = pc_xy[angle_filter]  # 필터된 포인트만 선택
            db = self.dbscan.fit_predict(pc_xy) # DBSCAN 알고리즘을 사용하여 X, Y 데이터 포인트를 클러스터링하고, 각 포인트의 클러스터 ID를 반환
            n_cluster = np.max(db) + 1 # 총 클러스터 수

            for c in range(n_cluster):
                c_tmp = np.mean(pc_xy[db==c, :], axis=0)
                c_tmp_position=[c_tmp[0], c_tmp[1], 1]
                if c_tmp[0]>=1.3:
                    local_c_tmp_position=lidar_to_gps_transform.dot(c_tmp_position)
                    global_c_tmp_position=gps_to_global.dot(local_c_tmp_position)
                    for path in ref_path.poses:
                        dis = sqrt(pow(path.pose.position.x - global_c_tmp_position[0], 2) + pow(path.pose.position.y - global_c_tmp_position[1], 2))
                        if dis < 1:  # 경로상에 클러스터링된 물체만 추가
                            #print(global_c_tmp_position)
                            cluster_points.append(global_c_tmp_position)  # Adding Z coordinate as 0.5 
                            break
            self.publish_point_cloud(cluster_points)
        else:
            self.publish_point_cloud(cluster_points)

if __name__ == '__main__':
    scan_cluster = SCANCluster()
    rospy.spin() 
