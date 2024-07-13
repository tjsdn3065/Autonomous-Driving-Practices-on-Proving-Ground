#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os
import rospkg
import numpy as np
from math import cos,sin,pi,sqrt,pow,atan2,tan

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import erpCmdMsg
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage


class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber('/tf', TFMessage, self.odom_callback)
        self.ctrl_cmd_pub = rospy.Publisher('erp42_ctrl_cmd',erpCmdMsg, queue_size=1)
        self.ctrl_cmd_msg = erpCmdMsg()

        self.is_path = False
        self.is_odom = False

        self.forward_point = Point()
        self.current_postion = Point()
        self.is_look_forward_point = False
        self.vehicle_length = 1.05
        self.lfd = 3
        self.target_vel=9

        if self.vehicle_length is None or self.lfd is None:
            print("you need to change values at line 30~31 ,  self.vegicle_length , lfd")
            exit()
        rate = rospy.Rate(20) # 15hz
        while not rospy.is_shutdown():

            if self.is_path ==True and self.is_odom==True  :
                
                vehicle_position=self.current_postion
                self.is_look_forward_point= False

                translation=[vehicle_position.x, vehicle_position.y]

                t=np.array([  # 전역 좌표계를 기준으로 로봇의 방향과 평행이동을 표현(즉 로봇좌표계)
                        [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                        [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                        [0                    ,0                    ,1            ]])

                det_t=np.array([ # 전역 좌표계를 기준으로 되어 있는 것을 로봇의 로컬 좌표계로 변환하기 위한 역변환 행렬
                       [t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])],
                       [t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])],
                       [0      ,0      ,1                                               ]])

                for i,value in enumerate(self.path.poses) : # 지역 경로
                    path_point=value.pose.position
                    global_path_point=[path_point.x,path_point.y,1] # global_path_point는 전역 좌표계 원점이 기준인 좌표 , local_path_point는 로봇의 위치(전역 좌표계 기준)가 원점인 좌표
                    local_path_point=det_t.dot(global_path_point)  # 로봇의 역변환 행렬 과 입력으로 받은 지역 경로의 좌표(전역 좌표계 기준) 곱해서 로봇의 현재 위치(전역 좌표계 기준)를 기준으로 지역 경로 좌표 생성 
                    if local_path_point[0]>0 : # 이 점이 차량의 전방에 위치한다면
                        dis=sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                        if dis>= self.lfd: #* self.pure_pursuit_gain : # 거리가 lfd보다 크다면 전방 목표 좌표로 설정
                            self.forward_point=path_point
                            self.is_look_forward_point=True
                            break
                
                theta=atan2(local_path_point[1],local_path_point[0])

                if self.is_look_forward_point :
                    steer=atan2(2*self.vehicle_length*sin(theta),self.lfd) * 180/pi
                    self.ctrl_cmd_msg.steer = -int(steer * 71)
                    self.ctrl_cmd_msg.speed = self.unsigned8(self.target_vel * 10)
                    if self.ctrl_cmd_msg.steer is None:
                        print("you need to change the value at line 70")
                        exit()

                    os.system('clear')
                    print("-------------------------------------")
                    print(" steering (deg) = ", steer)
                    print(" velocity (kph) = ", self.target_vel)
                    #print(self.lfd * self.pure_pursuit_gain)
                    print("-------------------------------------")
                else : 
                    print("no found forward point")
                    self.ctrl_cmd_msg.steer=0
                    self.ctrl_cmd_msg.speed=self.unsigned8(0)
                
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            else:
                os.system('clear')
                if not self.is_path:
                    print("[1] can't subscribe '/local_path' topic...")
                if not self.is_odom:
                    print("[2] can't subscribe '/odom' topic...")
            
            self.is_path = self.is_odom = False
            rate.sleep()
            
    def unsigned8(self,n):
        return n & 0xFF

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

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

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
