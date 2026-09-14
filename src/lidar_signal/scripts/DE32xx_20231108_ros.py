import time
import rospy

# def req_distance_data(self):
    # hex_string = '525363616ED4EA'
    # send_data = bytes.fromhex(hex_string)
    # self.serial_port.flushInput()
    # self.serial_port.write(send_data)
    # time.sleep(0.005)
    # recv_data = self.serial_port.read(1085)
    
    # if (len(recv_data) == 1085 
    #     and recv_data[:5] == bytes.fromhex('525363616E')):
    # # print(len(recv_data))
    #     distance = [int.from_bytes(recv_data[i:i+2], "big") / 1000 for i in range(5, len(recv_data), 2)]
    #     # rospy.logerr(f"CORRECT: {recv_data}")
    #     return distance
    # return recv_data
    # else:
    #     # rospy.logerr(f"ERROR: {recv_data}")
    #     return None

def req_distance_data(self):
    hex_string = '525363616ED4EA'
    send_data = bytes.fromhex(hex_string)
    # self.serial_port.flushInput()
    try:
        self.serial_port.write(send_data)
        # if self.serial_port.out_waiting:
        return True
        # else:
        #     return False
    except Exception as e:
        rospy.logerror(f'Write error: {e}') 
        return False
        
def get_distance_data(self):
    try:
        recv_data = self.serial_port.read(1085)
        return recv_data
    except Exception as e:
        rospy.logerror(f'Read error: {e}')

def get_intensities_data(self):
    hex_string = '52537472652819'
    send_data = bytes.fromhex(hex_string)
    # self.serial_port.flushInput()
    self.serial_port.write(send_data)
    recv_data = self.serial_port.read(1085)
    if len(recv_data) == 1085:
        intensities = [int.from_bytes(recv_data[i:i+2], "big") / 1000 for i in range(5, len(recv_data), 2)]
        return intensities
    else:
        rospy.logerr(f"ERROR: {recv_data}")
        return None