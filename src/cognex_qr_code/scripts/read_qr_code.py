#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket  # for sockets
import sys  # for exit
import time
import traceback
import math
from std_stamped_msgs.msg import (
    StringAction,
    StringStamped,
    StringResult,
    StringFeedback,
    StringGoal,
    Int8Stamped,
    EmptyStamped,
    Float32Stamped,
)
from cognex_qr_code.srv import QrCode, QrCodeResponse


class Cognex_driver:
    def __init__(self) -> None:
        rospy.init_node("read_qr_code")
        # Param
        self.host = rospy.get_param("~ip_address", "192.168.0.100")
        self.port = rospy.get_param("~port", 23)
        self.rate = rospy.Rate(30)

        self.read_success = False
        self.socket_timeout = rospy.get_param("~socket_timeout", 3)
        self.read_attempts = rospy.get_param("~read_attempts", 3)
        self.connect_attempts = 2
        connect_success = False

        self.pub_qrcode = rospy.Publisher(
            "/qr_sensor", StringStamped, queue_size=10
        )
        self.qr_code_serv = rospy.Service(
            "ReadQrCode", QrCode, self.qr_code_callback
        )
        # self.connect_socket()

    def connect_socket(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            rospy.loginfo("Connected to Cognex QR code sensor successfully")
        except:
            rospy.logerr(
                "Failed to connect to Cognex QR code sensor: %s",
                traceback.format_exc(),
            )
            self.close_socket()
            sys.exit()

    def close_socket(self):
        try:
            self.socket.close()
        except:
            rospy.logerr("Failed to close socket: %s", traceback.format_exc())

    def read_data(self, timeout):
        response = QrCodeResponse()
        msg = StringStamped()
        connect_time = 0
        connect_success = False
        read_error = ""
        while connect_time < self.connect_attempts:
            if not connect_success:
                try:
                    self.socket = socket.socket(
                        socket.AF_INET, socket.SOCK_STREAM
                    )
                    rospy.loginfo(f"Connecting to {self.host}:{self.port}")
                    self.socket.connect((self.host, self.port))
                    self.socket.settimeout(self.socket_timeout)
                    rospy.loginfo(
                        "Connected to Cognex QR code sensor successfully"
                    )
                    connect_success = True
                    connect_time += 1
                except:
                    rospy.logerr(
                        "Failed to connect to Cognex QR code sensor: %s",
                        traceback.format_exc(),
                    )
                    connect_success = False
                    connect_time += 1
                    rospy.loginfo(
                        f"Restart connection for: {self.connect_attempts} times"
                    )
                    self.close_socket()
                    # rospy.sleep(1)
                    continue
            else:
                read_time = 0
                # Gửi lệnh yêu cầu đọc mã QR code
                rospy.logwarn("request read qr code")
                while read_time < self.read_attempts:
                    try:
                        command = "||>TRIGGER ON\r\n>"
                        self.socket.sendall(command.encode())
                        rospy.sleep(0.1)
                        # Đọc dữ liệu từ cảm biến
                        data = self.socket.recv(1024)
                        code_id = data.decode()
                        code_id = code_id.replace("\n", "").replace("\r", "")
                        response.Res = code_id
                        msg.stamp = rospy.Time.now()
                        msg.data = code_id

                        if code_id == "NG":
                            rospy.logerr("QR code read failed: NG")
                            # read_error = "QR code read failed: NG"
                            read_time += 1
                            continue
                        self.pub_qrcode.publish(msg)
                        rospy.logwarn(
                            f"Read success: {code_id} - Read time: {read_time}"
                        )
                        rospy.loginfo(f"Read success Close connection")
                        # connect_time = 0
                        self.close_socket()
                        return response

                    except UnicodeDecodeError:
                        # rospy.logerr(f"decode error")
                        read_time += 1
                        continue
                    except socket.timeout:
                        # read_error = "Socket timeout"
                        read_time += 1
                        connect_success = False
                        self.close_socket()
                        rospy.sleep(1)
                        break
                    except Exception as e:
                        rospy.logerr(f"Communication error: {e}")
                        connect_success = False
                        self.close_socket()
                        rospy.sleep(1)
                        break
                # Send error response when multiple read attemps fail
                if read_time >= self.read_attempts:
                    rospy.logerr(
                        f"Multiple read attemps fail - Read time: {read_time}"
                    )
                    code_id = ""
                    response.Res = code_id
                    msg.stamp = rospy.Time.now()
                    msg.data = code_id
                    self.pub_qrcode.publish(msg)
                    return response

                if read_error != "":
                    {rospy.logerr(f"{read_error}")}
        # Send error response when multiple reconnect attemps fail
        if connect_time >= self.connect_attempts:
            rospy.logerr(f"Multiple connect attemps fail")
            code_id = "CONNECT_ERROR"
            response.Res = code_id
            msg.stamp = rospy.Time.now()
            msg.data = code_id
            self.pub_qrcode.publish(msg)
            return response

    def qr_code_callback(self, req):
        return self.read_data(req.TimeOut)


if __name__ == "__main__":
    try:
        sensor = Cognex_driver()
        rospy.spin()
    except KeyboardInterrupt:
        print("Caught keyboard interrupt, exiting")
    finally:
        if not sensor.connect_success:
            pass
        else:
            sensor.close_socket()
