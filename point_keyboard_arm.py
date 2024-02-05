#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from time import time
import threading

# 전역 변수 초기화
a_key_press_interval = 10.0  # 메시지를 전송할 수 있는 최소 간격(초)
last_a_message_time = time() - a_key_press_interval

# 메시지 전송 함수
def publish_message(action):
    global last_a_message_time, pub
    current_time = time()

    if (current_time - last_a_message_time) >= a_key_press_interval:
        for char in action.split(", "):
            rospy.sleep(3)  # 3초 간격
            pub.publish(String(char))
        last_a_message_time = current_time

# 메시지를 받을 때 실행될 콜백 함수
def character_callback(msg):
    action = ""
    if msg.data == "a":
        action = "i, g, d, s, f, d, i, o, g, i"
    elif msg.data == "b":
        action = "i, g, a, f, i, o, g, i"
    elif msg.data == "c":
        action = "i, g, x, z, f, x, i, o , g, i"

    if action:
        threading.Thread(target=publish_message, args=(action,)).start()

def listener():
    global pub

    # ROS 노드 초기화
    rospy.init_node('keyboard_control_listener', anonymous=True)

    # 퍼블리셔 생성
    pub = rospy.Publisher('/keyboard_action_result', String, queue_size=10)

    # 구독자 생성
    rospy.Subscriber('/UI_result', String, character_callback)

    # ROS 노드 유지
    rospy.spin()

if __name__ == '__main__':
    listener()


