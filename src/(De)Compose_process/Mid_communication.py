#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import rospy
from indy_driver.msg import cell_state_info  # msg: uint8 flag

HOST = '127.0.0.1'
PORT = 9999

if __name__ == "__main__":
    rospy.init_node("flag_server", anonymous=True)
    pub = rospy.Publisher("cell_state_info", cell_state_info, queue_size=1)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # 재실행 시 포트 점유 문제 방지
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print(f"[*] 서버가 {HOST}:{PORT}에서 연결을 기다리는 중입니다...")

        while not rospy.is_shutdown():
            conn, addr = s.accept()
            with conn:
                print(f"[+] {addr} 에서 연결됨.")
                
                while not rospy.is_shutdown():
                    data = conn.recv(1024)
                    if not data:
                        break

                    # 수신된 데이터 로그
                    print(f"Received data: {data}")

                    # 바이트 데이터를 정수로 변환
                    try:
                        flag = int.from_bytes(data, byteorder='big')  # 바이트 데이터를 정수로 변환
                    except ValueError:
                        rospy.logwarn(f"[flag_server] 수신된 값이 정수가 아님: {data}")
                        continue

                    # 유효한 값인지 체크 (0 ~ 255 범위 내의 uint8 값만 허용)
                    if 0 <= flag <= 255:
                        msg = cell_state_info()
                        msg.flag = flag  # 수신된 값을 uint8로 처리
                        pub.publish(msg)
                        rospy.loginfo(f"[flag_server] published flag={msg.flag}")
                    else:
                        rospy.logwarn(f"[flag_server] 유효하지 않은 값: '{flag}' ")
