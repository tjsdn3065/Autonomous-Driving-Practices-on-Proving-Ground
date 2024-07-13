#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import rospkg
import time
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry,Path
import numpy as np
import tf
from tf.transformations import euler_from_quaternion
from shapely.geometry import Polygon, LineString
from tf2_msgs.msg import TFMessage
from morai_msgs.msg import erpCmdMsg, erpStatusMsg
from morai_msgs.msg import BoundingBox, BoundingBoxes

class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/half_circle", Path, self.path_callback)
        #rospy.Subscriber('/bounding_boxes', BoundingBoxes, self.detection_callback)
        
        # self.is_detection= False
        # self.detection_msg = None
        
        rospy.Subscriber('/tf', TFMessage, self.odom_callback)
        rospy.Subscriber("/erp42_status",erpStatusMsg, self.status_callback) 
        self.ctrl_cmd_pub = rospy.Publisher('erp42_ctrl_cmd',erpCmdMsg, queue_size=1)
        self.ctrl_cmd_msg = erpCmdMsg()

        self.is_path = False
        self.is_odom = False 
        self.is_status = False
        self.is_global_path = False
        
        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_postion = Point()

        self.prev_i=self.read_initial_index() # current_waypoint를 위한 변수

        self.vehicle_length = 1.05
        self.lfd = 3
        self.target_velocity = 5

        self.pid = pidControl()
        self.velocity_list=self.get_vel_plan()

        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            os.system('clear')
            if self.is_path == True and self.is_odom == True and self.is_status == True: #and self.is_detection == True:
                self.current_waypoint = self.get_current_waypoint(self.status_msg,self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint]
                if False:
                        self.ctrl_cmd_msg.brake = self.unsigned8(33)
                        self.ctrl_cmd_msg.speed=self.unsigned8(0)
                        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                else:
                    if False:
                        self.ctrl_cmd_msg.brake = self.unsigned8(33)
                        self.ctrl_cmd_msg.speed=self.unsigned8(0)
                        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    else:
                        curr_vel=self.status_msg.speed/10
                        steering = self.calc_pure_pursuit()
                        if self.is_look_forward_point :
                            self.ctrl_cmd_msg.steer = -int(steering * 71)
                        else : 
                            print("no found forward point")
                            self.ctrl_cmd_msg.steer=0
                        output = self.pid.pid(self.target_velocity,curr_vel)
                        vel_output=int(output)
                        
                        if self.target_velocity == 0:
                            self.ctrl_cmd_msg.brake = self.unsigned8(33)
                            self.ctrl_cmd_msg.speed=self.unsigned8(0)
                        elif self.target_velocity == 8:
                            brake=int(curr_vel)
                            self.ctrl_cmd_msg.speed=self.unsigned8(0)
                            self.ctrl_cmd_msg.brake = self.unsigned8(brake)
                        else:
                            if output > 0.0:
                                self.ctrl_cmd_msg.speed = self.unsigned8(vel_output*10)
                                self.ctrl_cmd_msg.brake = self.unsigned8(0)
                            else:
                                self.ctrl_cmd_msg.speed = self.unsigned8(0)
                                self.ctrl_cmd_msg.brake = self.unsigned8(vel_output)

                        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            else:
                print(f"self.is_path (/lattice_path) : {self.is_path}")
                print(f"self.is_status (/Ego_topic)  : {self.is_status}")
                print(f"self.is_odom (/odom)         : {self.is_odom}")

            rate.sleep()
            
    def unsigned8(self,n):
        return n & 0xFF

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg
        
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True  

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
        
    # def detection_callback(self,msg):
    #     self.is_detection=True
    #     self.detection_msg=msg

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg    
        
    def read_initial_index(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('erp42')
        full_path = pkg_path + '/path/global_path_prev_i.txt'
        try:
            with open(full_path, 'r') as file:
                line = file.readline().strip()
            return int(line)
        except Exception as e:
            rospy.logerr(f"Failed to read initial index: {e}")
            return 0
        
    # def check_red_traffic_light_detected(self):
    #     """감지된 메시지에서 빨간 신호등이 있는지 확인하고 결과를 반환하는 함수"""
    #     if self.detection_msg is None:
    #         return False  # 아직 감지된 메시지가 없으면 False 반환

    #     for bbox in self.detection_msg.bounding_boxes:
    #         if bbox.Class == 'traffic_light_red':
    #             self.start = False
    #             return True  # 빨간 신호등 발견

    #     return False  # 빨간 신호등이 없으면 False 반환
    
    # def check_large_person_detected(self):
    #     """감지된 메시지에서 'person'이 감지되었으며 그 넓이가 8000 이상이고 밑변이 도로 영역 내에 있는지 확인하고 결과를 반환하는 함수"""
    #     if self.detection_msg is None:
    #         return False  # 아직 감지된 메시지가 없으면 False 반환

    #     for bbox in self.detection_msg.bounding_boxes:
    #         if bbox.Class == 'person':
    #             xmin, ymax, xmax = bbox.xmin, bbox.ymax, bbox.xmax
    #             width = xmax - xmin
    #             height = bbox.ymax - bbox.ymin
    #             area = width * height
    #             print(area)
                
    #             # 도로 영역 정의
    #             road_polygon = Polygon([(610, 361), (660, 361), (0, 720), (1280, 720)])

    #             # 선분 정의
    #             bottom_line = LineString([(xmin, ymax), (xmax, ymax)])
                
    #             # 밑변이 도로 영역 내에 완전히 포함되고, 넓이 조건 만족 확인
    #             if area >= 45000:
    #                 # 폴리곤과 선분의 교차 검사
    #                 if road_polygon.intersects(bottom_line):
    #                 # if left_road_polygon.contains(left_bottom_line) or right_road_polygon.contains(right_bottom_line) :
    #                     print('stop!!!!!!!!!!!')
    #                     return True  # 'person'의 bounding box 밑변이 도로 영역 내에 있고, 넓이 조건 만족

    #     return False  # 해당하는 'person'이 없거나 조건 미충족
    
    def get_vel_plan(self):
        # ROS 패키지 경로 얻기
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('erp42')
        full_path = pkg_path + '/path/Wls_final_path.txt'

        # 속도 매핑
        speed_map = {'0': 9, '1': 7, '2': 7, '3': 4, '4': 0, '5': 8,'6':10}

        # 파일 읽기 및 속도 계획 생성
        try:
            with open(full_path, 'r') as file:
                lines = file.readlines()
                out_vel_plan = [speed_map.get(line.split()[2], 0) for line in lines]
        except Exception as e:
            print(f"Error reading or processing the file: {e}")
            return []

        return out_vel_plan
    
    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        current_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            if self.prev_i - 10 < i and i < self.prev_i + 100:
                dx = self.current_postion.x - pose.pose.position.x
                dy = self.current_postion.y - pose.pose.position.y

                dist = sqrt(pow(dx,2)+pow(dy,2))
                if min_dist > dist :
                    min_dist = dist
                    current_waypoint = i
                    self.prev_i=current_waypoint

        return current_waypoint

    def calc_pure_pursuit(self,):
        
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]

        #TODO: (3) 좌표 변환 행렬 생성
        trans_matrix = np.array([
                [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                [0                    ,0                    ,1            ]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num,i in enumerate(self.path.poses) :
            path_point=i.pose.position
            global_path_point = [path_point.x,path_point.y,1]
            local_path_point = det_trans_matrix.dot(global_path_point)    
            if local_path_point[0]>0 :
                dis = sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                if dis >= self.lfd :
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break
        
        #TODO: (4) Steering 각도 계산
        theta = atan2(local_path_point[1],local_path_point[0])
        steering = atan2(2*self.vehicle_length*sin(theta),self.lfd)* 180/pi
        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.07
        self.d_gain = 0.04
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.1

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        #TODO: (5) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
