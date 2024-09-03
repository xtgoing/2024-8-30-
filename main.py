#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
from decimal import Decimal
from std_msgs.msg import Int32, String
from move_base_msgs.msg import MoveBaseActionResult
import tf
import configparser
import json
import requests
from std_msgs.msg import Empty 
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
import base64
bridge = CvBridge()
from position import *
from robot_work import *
ecareConfig = configparser.ConfigParser()
ecareConfig.read("/home/elephant/dev/team2/carebot_config.ini")
service_ip = ecareConfig.get("config","service_ip")
global tar_id
tar_id = -1

def move(where, id):
    rospy.loginfo("正在去{}, id：{}".format(where, id))
    position_goal = search_position(id)       
    pose = change_pose(position_goal)
    pub.publish(pose)
    pre_statusid = search_status(id)
    
    # 每隔1秒查看一次状态码是否改变
    while not rospy.is_shutdown():
        cur_statusid = search_status(id)
        rospy.loginfo("{}号目标点状态码:{}".format(id, cur_statusid))
        if cur_statusid != pre_statusid:
            break
        time.sleep(1)
    rospy.loginfo("{}的状态码为{}".format(where, cur_statusid))
    if cur_statusid == 1:
        rospy.loginfo("已到{}".format(where))
        return cur_statusid

def result_callback(msg):
    global tar_id
    rospy.loginfo(msg.status.status)
    # if len(msg.status_list) > 0 and msg.status_list[0].status == 3:
    if msg.status.status == 3:
        rospy.loginfo("成功到达目标！")
        update_statusid(tar_id, 1)

def control_callback(data):   
    global tar_id
    # 去零食区
    if data.data == 21:
        tar_id = search_id('snacks')
        result = move('零食区', tar_id)
        if result == 1:
            publish_tts_result("已到达零食区，零食有益达、每益添、果冻、陈皮丹")
            init_status()
    
    # 去咖啡区
    if data.data == 22:
        tar_id = search_id('cafe')
        result = move('咖啡区', tar_id)
        if result:
            publish_tts_result("已到达咖啡区")
            init_status()
            time.sleep(2)
            publish_tts_result("正在接咖啡，请稍等")
            try:
                # arm move
                # take_cof(ml_obj, ml_arm, mr_obj, mr_arm)
                
                take_cup(mr_obj, mr_arm)
                wait(mr_arm)
                take_coffee(ml_obj, ml_arm)
               
                
            except Exception as e:
                # 记录异常信息
                rospy.logerr("发生错误: %s", str(e))
                rospy.loginfo("arm move error!!!")

    # 原点
    if data.data == 23:
        tar_id = search_id('init')
        result = move('原点', tar_id)
        if result:
            publish_tts_result("已到达原点")
            init_status()

    # 拿果冻
    if data.data == 24:
        # 语音播报
        publish_tts_result("正在抓取果冻")
        try:
            # 抓取果冻
            ml_obj.catch_goods(0,0,ml_arm)    # 左下
            wait(ml_arm)
        except Exception as e:
            # 记录异常信息
            rospy.logerr("发生错误: %s", str(e))
            rospy.loginfo("arm move error!!!")
        

    # 拿饮料
    if data.data == 25:
        # 语音播报
        publish_tts_result("正在抓取每益添")
        try:
            # 抓取饮料
            mr_obj.catch_goods(0,1,mr_arm)    # 右下
            wait(mr_arm)
        except Exception as e:
            # 记录异常信息
            rospy.logerr("发生错误: %s", str(e))
            rospy.loginfo("arm move error!!!")
    
    # 拿陈皮丹
    if data.data == 26:
        # 语音播报
        publish_tts_result("正在抓取陈皮丹")
        try:
            # 抓取陈皮丹
            mr_obj.catch_goods(1,1,mr_arm)    # 右上
            wait(mr_arm)

            r = [73.79, 42.13, -74.84, -36.98, 26.29, -0.01] 
            mr_arm.send_angles(r,20)
            mr_arm.set_gripper_value(0, 100)  
        except Exception as e:
            # 记录异常信息
            rospy.logerr("发生错误: %s", str(e))
            rospy.loginfo("arm move error!!!")

    # 口香糖
    if data.data == 27:
        # 语音播报
        publish_tts_result("正在抓取口香糖")
        try:
            # 抓取益达
            ml_obj.catch_goods(1,0,ml_arm)    # 左上
            wait(ml_arm)
            l = [-67.12, 49.84, -72.52, 39.8, 35.75, 0.22]
            ml_arm.send_angles(l,20)
            ml_arm.set_gripper_value(0, 100) 
            
        except Exception as e:
            # 记录异常信息
            rospy.logerr("发生错误: %s", str(e))
            rospy.loginfo("arm move error!!!")
        
    
    # 一号桌
    if data.data == 28:
        # # 拿起盒子
        # ml_obj.catch_box(0,ml_arm)
        # wait(ml_arm)
        # mr_obj.catch_box(1,mr_arm)
        # wait(mr_arm)
        # put_down(ml_arm,mr_arm,1)   #双臂拿起动作
        # 导航到table1
        tar_id = search_id('table1')
        result = move('一号桌', tar_id)
        if result == 1:
            publish_tts_result("已到达一号桌")
            init_status()
            time.sleep(2)
            # 放零食框
            put_down(ml_arm,mr_arm,0)   #双臂放下动作

    # 二号桌
    if data.data == 29:
        # # 拿起盒子
        # ml_obj.catch_box(0,ml_arm)
        # wait(ml_arm)
        # mr_obj.catch_box(1,mr_arm)
        # wait(mr_arm)
        # time.sleep(2)
        # put_down(ml_arm,mr_arm,1)   #双臂拿起动作
        # 导航到table2
        tar_id = search_id('table2')
        result = move('二号桌', tar_id)
        if result == 1:
            publish_tts_result("已到达二号桌")
            init_status()
            time.sleep(2)
            # 放零食框
            put_down(ml_arm,mr_arm,0)   #双臂放下动作

    # 去一号桌
    if data.data == 30:
        print("in table1")
        tar_id = search_id('table1')
        result = move('一号桌', tar_id)
        if result == 1:
            publish_tts_result("已到达一号桌")
            init_status()
            time.sleep(2)
            # 放咖啡
            put_down_sigl(mr_arm)

    # 去二号桌
    if data.data == 31:
        print("in table2")
        tar_id = search_id('table2')
        result = move('二号桌', tar_id)
        if result == 1:
            publish_tts_result("已到达二号桌")
            init_status()
            time.sleep(2)
            # 放咖啡
            # put_down_sigl(mr_arm)
            mr_arm.set_gripper_value(0, 100)


# 发布语音合成的结果
def publish_tts_result(text):
    rospy.loginfo("[TTS Result]: %s", text)
    tts_pub.publish(text)

def power_on():
    while True:
        if ml_arm.get_angles() == None:
            ml_arm.power_on()
            time.sleep(0.5)
        else:
            print("ml_arm", ml_arm.get_angles())
            break
    while True:
        if mr_arm.get_angles() == None:
            mr_arm.power_on()
            time.sleep(0.5)
        else:
            print("mr_arm", mr_arm.get_angles())
            break
    print("上电成功")

def main():
    rospy.init_node('ros_detect_node')
    init_status()

    # 机械臂
    camera_params = np.load("/home/elephant/dev/team2/src/ros_detection/src/camera_params.npz")  # 相机配置文件
    mtx, dist = camera_params["mtx"], camera_params["dist"]
    global ml_obj, mr_obj, ml_arm, mr_arm
    ml_arm = Mercury("/dev/left_arm")  # 设置左臂端口
    mr_arm = Mercury("/dev/right_arm")  # 设置右臂端口
    power_on()
    ml_obj = camera_detect(4, 25, mtx, dist, 0)
    mr_obj = camera_detect(2, 25, mtx, dist, 1)
    # 初始位置  # 闭合夹爪
    l = [-67.12, 49.84, -72.52, 39.8, 35.75, 0.22]
    r = [73.79, 42.13, -74.84, -36.98, 26.29, -0.01] 
    ml_arm.send_angles(l,20)
    ml_arm.set_gripper_value(0, 100) 
    wait(ml_arm)
    mr_arm.send_angles(r,20)
    mr_arm.set_gripper_value(0, 100)  

    global pub 
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    global tts_pub 
    tts_pub = rospy.Publisher('/voice_system/tts_topic', String, queue_size=10)
    rospy.Subscriber("/voice_system/control", Int32, control_callback)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, result_callback)
    rospy.spin()

if __name__ == '__main__':
    main()


