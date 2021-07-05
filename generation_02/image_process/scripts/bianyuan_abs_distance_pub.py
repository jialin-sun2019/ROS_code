#! /usr/bin/env python
# encoding: utf-8
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import *
from image_process.msg import ping
from image_process.msg import abs_distance

class Image_Process:
    def __init__(self):
        #### 图像处理算法的参数
        self.kernel = np.ones((5, 5), np.uint8)
        self.Low_H =5
        self.High_H=26
        self.Low_S = 110
        self.High_S = 245
        self.Low_V = 154
        self.High_V = 255
        self.Image_Window_Name="YelloBarTracker"
        self.Control_Window_Name='Control'
        self.Mask_Window_Nane='Mask'
        self.lower_yellow = np.array([self.Low_H,self.Low_S,self.Low_V])
        self.upper_yellow = np.array([self.High_H,self.High_S,self.High_V])
        self.ok_image_msg = Image()
        ####相机的内参数
        self.camera_factor = 1000.0
        self.camera_cx = 321.798
        self.camera_cy = 239.607
        self.camera_fx = 615.899
        self.camera_fy = 616.468

        ####图像信息
        self.image_bridge = CvBridge()
        self.rgb_image = np.zeros((640, 480, 3), np.uint8)
        self.dep_image = np.zeros((640, 480, 3), np.uint8)
        ####图像的状态
        self.rgb_ok = False
        self.dep_ok = False

        ####乒乓球在图像中的坐标信息和半径信息
        self.x = 0
        self.y = 0
        self.r = 0
        ####求距离时的空间距离信息
        self.s_x = .0
        self.s_y = .0
        self.s_z = .0
        ####  ********最后的结果  相对距离  相对机器人的x和y
        self.abs_x = .0
        self.abs_y = .0


        self.distance = .0
        self.d = list()

        self.fushi_angle = math.radians(90)    #根据摄像头安装位置得到   这里是弧度
        self.pianj_angle = math.radians(90)    #根据图像得到，  同样是弧度
        self.angle_distance = .0

        ####发布和订阅消息相关
        rospy.init_node("image_process", anonymous=True)  # 先初始化
        self.okimage_pub = rospy.Publisher('/ok/image', Image, queue_size=1)
        self.image_pub   = rospy.Publisher('/ping_info', ping, queue_size=1)
        self.abs_distance = rospy.Publisher('/abs_info', abs_distance, queue_size=1)
        self.abs_distance_msg = abs_distance()
        self.ping_msg = ping()
        self.rate = rospy.Rate(45) #然后设置参数
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.Rgb_ImageCallback)
        rospy.Subscriber("/camera/depth/image",   Image, self.Dep_ImageCallback)
    def Dep_ImageCallback(self,msg):
        try:
            self.dep_image = self.image_bridge.imgmsg_to_cv2(msg,'32FC1')
            cv2.imshow("depth_image", self.dep_image)
            cv2.waitKey(1)
            self.dep_ok = True
        except :
            rospy.loginfo("depth 图像变换错误...")
            self.rgb_ok = False
    def Rgb_ImageCallback(self,msg):
        try:
            self.rgb_image = self.image_bridge.imgmsg_to_cv2(msg,'bgr8')
            cv2.imshow("rgb_image", self.rgb_image)
            cv2.waitKey(1)
            self.rgb_ok = True
        except :
            rospy.loginfo("depth 图像变换错误...")
            self.rgb_ok = False
    def rgb_process(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        e_mask = cv2.erode(mask, self.kernel, iterations=1)
        p_mask = cv2.dilate(e_mask, self.kernel, iterations=1)
        binary, contours, hierarchy = cv2.findContours(p_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas = [cv2.contourArea(contours[i]) for i in range(len(contours))]
        contour = list()
        if len(areas) != 0:
            contour.append(contours[areas.index(max(areas))])
            x, y = (int(np.mean(contour[0][:, 0, 0])), int(np.mean(contour[0][:, 0, 1])))
            cv2.drawContours(image, contour, -1, (0, 0, 255), 3)
            self.okimage_pub.publish(self.image_bridge.cv2_to_imgmsg(image, 'bgr8'))
            return True, x, y
        else:
            x, y = 0, 0
            return False, x, y
    def angle_location(self):
        #首先根据   二维图像中的坐标x,y  求得球的角度self.pianj_angle
        self.pianj_angle = 10
        #根据      距离和摄像头安装角度求得车到球的直线距离
        self.angle_distance = math.sin(self.fushi_angle)*self.distance  #得到车和球平面的直线距离
        self.abs_x = math.sin(self.pianj_angle)*self.angle_distance#左右的距离信息
        self.abs_y = math.cos(self.pianj_angle)*self.angle_distance#前方的距离信息
        self.abs_distance_msg.abs_x = self.abs_x
        self.abs_distance_msg.abs_y = self.abs_y


    def main(self):
        while not rospy.is_shutdown():
            if self.rgb_ok and self.dep_ok:
                # rospy.loginfo("rgb and depth ok ...")
                self.rgb_ok = False
                self.dep_ok = False
                ret,self.x,self.y = self.rgb_process(self.rgb_image)
                if ret:
                    self.d=list()#列表置空
                    for i in range(3):
                        for j in range(3):
                            try:
                                if np.isnan(self.dep_image[self.x-1+i][self.y-1+i]):
                                    pass
                                else:
                                    self.d.append(self.dep_image[self.x-1+i][self.y-1+i])
                            except:
                                pass
                    if len(self.d)==0:
                        self.distance =.0
                    else:
                        self.s_z = np.mean(self.d)/self.camera_factor
                        self.s_x = (self.y - self.camera_cx) * self.s_z / self.camera_fx
                        self.s_y = (self.x - self.camera_cy) * self.s_z / self.camera_fy
                        self.distance = math.sqrt(math.pow(self.s_x,2)+math.pow(self.s_y,2)+math.pow(self.s_z,2))*100000
                        rospy.loginfo("distance  :   %ld", self.distance)
                else:
                    rospy.loginfo("未识别到乒乓球，不进行测距...")
                    self.distance = 0
            """     截止到现在已经得到了  二维图像中的坐标  摄像头到球的距离
                    二维图像中的坐标   self.x = 0, self.y = 0
                    摄像头到球的距离   self.distance

                    1、通过  self.x = 0, self.y = 0                 求得 self.pianj_angle
                    2、通过  self.fushi_angle, self.distance        求得 self.angle_distance
                    3、通过  self.angle_distance和self.pianj_angle  求得 self.abs_x, self.abs_y
            """
            # self.angle_location()
            # rospy.loginfo("distance  :   %ld",self.distance)
            self.rate.sleep()
if __name__ == '__main__':
    process = Image_Process()
    process.main()
