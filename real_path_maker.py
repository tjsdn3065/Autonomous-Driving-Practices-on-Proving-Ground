#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import rospkg
from math import sqrt
from tf2_msgs.msg import TFMessage

class pathMaker:
    
    def __init__(self, pkg_name, path_name):
        rospy.init_node('path_maker', anonymous=True)
        self.subscriber = rospy.Subscriber('/tf', TFMessage, self.status_callback)
        
        # 패키지 경로 로드 & 파일 쓰기 모드
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        full_path = pkg_path + '/' + path_name + '.txt'
        self.file = open(full_path, 'w')

        self.prev_x = 0
        self.prev_y = 0
        self.is_status = False

        rospy.spin()

    def status_callback(self, msg):
        for transform in msg.transforms:
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))
            
            if distance > 0.3:
                data = '{0}\t{1}\n'.format(x, y)
                self.file.write(data)
                self.prev_x = x
                self.prev_y = y
                print("write : ", x, y)

    def __del__(self):
        self.file.close()
        print("File closed successfully")

if __name__ == '__main__':
    try:
        path_maker = pathMaker("erp42", "real_test_path6")
    except rospy.ROSInterruptException:
        pass
