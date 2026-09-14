#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy, socket
from qr_irayble.srv import CodeRead, CodeReadResponse

class AGVCodeReader:
    def __init__(self):
        rospy.init_node("agv_code_reader_node")

        self.host = rospy.get_param("~ip_address", "192.168.0.100")
        self.port = rospy.get_param("~port", 23)
        self.socket_timeout = rospy.get_param("~socket_timeout", 3)
        self.connect_attempts = rospy.get_param("~connect_attempts", 2)

        self.service = rospy.Service("ReadCode", CodeRead, self.handle_service)
        rospy.loginfo("AGV Code Reader service ready at /ReadCode")

    def query_tagnum(self):
        cmd = bytes([0xC8, 0x37])  # query
        for attempt in range(self.connect_attempts):
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(self.socket_timeout)
                sock.connect((self.host, self.port))
                sock.sendall(cmd)
                data = sock.recv(1024)
                sock.close()

                if len(data) < 21:
                    return "ERROR"

                b14, b15, b16, b17, b18 = data[13:18]
                tagnum = ((b14 << 28) |
                          (b15 << 21) |
                          (b16 << 14) |
                          (b17 << 7) |
                          b18)
                return "" if tagnum == 0 else str(tagnum)
            except Exception as e:
                rospy.logwarn(f"Query attempt {attempt+1} failed: {e}")
                rospy.sleep(1)
        return "CONNECT_ERROR"

    def handle_service(self, req):
        result = self.query_tagnum()
        return CodeReadResponse(Res=result)

if __name__ == "__main__":
    node = AGVCodeReader()
    rospy.spin()
