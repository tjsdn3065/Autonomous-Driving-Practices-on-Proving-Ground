#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import sqrt,pow
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path
import tf
from tf2_msgs.msg import TFMessage

class path_pub :

    def __init__(self):
        rospy.init_node('path_pub', anonymous=True)
        #rospy.Subscriber("/odom", Odometry, self.odom_callback) # run gpsimu_parser.py
        rospy.Subscriber('/tf', TFMessage, self.odom_callback)
        rospy.Subscriber("/global_path",Path, self.global_Path_callback)

        self.local_path_pub = rospy.Publisher('/local_path',Path, queue_size=10)
        
        # 초기화
        self.global_path_msg=Path()
        self.global_path_msg.header.frame_id='/map'
        
        self.is_status=False
        self.local_path_size=50

        self.x = 0
        self.y = 0
        

        self.prev_i=self.read_initial_index()
        self.update_count = 0  # 추가된 변수: 업데이트 카운터


        rate = rospy.Rate(20) # 8hz
        while not rospy.is_shutdown():
   
            if self.is_status == True :
                local_path_msg=Path()
                local_path_msg.header.frame_id='/map'
                
                x=self.x
                y=self.y
                min_dis=float('inf')
                current_waypoint=-1
                for i,waypoint in enumerate(self.global_path_msg.poses) :
                    if self.prev_i - 10 < i and i < self.prev_i + 100:
                        distance=sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2))
                        if distance < min_dis :
                            min_dis=distance
                            current_waypoint=i
                            self.prev_i=current_waypoint
                            self.update_count += 1  # 카운터 증가

                            if self.update_count >= 90:  # 90번 변경될 때마다 파일에 쓰기
                                self.update_index_file()
                                self.update_count = 0  # 카운터 리셋
                            
                
                if current_waypoint != -1 :
                    if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                        for num in range(current_waypoint,current_waypoint + self.local_path_size ) :
                            tmp_pose=PoseStamped()
                            tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                            tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                            tmp_pose.pose.orientation.w=1
                            local_path_msg.poses.append(tmp_pose)
                    
                    else :
                        for num in range(current_waypoint,len(self.global_path_msg.poses) ) :
                            tmp_pose=PoseStamped()
                            tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                            tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                            tmp_pose.pose.orientation.w=1
                            local_path_msg.poses.append(tmp_pose)

                self.local_path_pub.publish(local_path_msg)

            rate.sleep()
            
            
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
        
    def update_index_file(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('erp42')
        full_path = pkg_path + '/path/global_path_prev_i.txt'
        try:
            with open(full_path, 'w') as file:
                file.write(str(self.prev_i))
            #rospy.loginfo("Updated global_path_prev_i.txt with new index.")
        except Exception as e:
            rospy.logerr(f"Failed to write to index file: {e}")


    def odom_callback(self, msg):
        self.is_status=True

        # TFMessage 내의 모든 TransformStamped 메시지 순회
        for transform in msg.transforms:
            # 필요한 경우 특정 frame_id에 대해서만 처리
            if transform.child_frame_id == "ego_vehicle":
                # translation의 x와 y 값을 저장
                self.x = transform.transform.translation.x
                self.y = transform.transform.translation.y


    def global_Path_callback(self,msg):
        self.global_path_msg = msg       

if __name__ == '__main__':
    try:
        test_track=path_pub()
    except rospy.ROSInterruptException:
        pass
