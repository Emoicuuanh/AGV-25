import modbus_tcp_passbox
import time
import rospy
###PASS_BOX###
# INPUT PLC, OUTPUT AGV
open_dirty_side = 201
open_clean_side = 200
# OUTPUT PLC, INPUT AGV
done_open_dirty_side = 201
done_close_dirty_side = 200
done_open_clean_side = 203
done_close_clean_side = 202

######################
###Bộ nâng hạ###
# INPUT PLC, OUTPUT AGV
emg_agv_request = 1
place_agv_request = 2
pick_agv_request = 3
close_barie_dirty_side = 4
open_barie_dirty_side = 5
agv_going_passbox = 6
emg_agv = 7
safety_off = 8
lift_open_door_dirty_side = 9
lift_close_door_dirty_side = 10
lift_stop_door_dirty_side = 11
lift_emg_door_dirty_side = 12
lift_open_door_clean_side = 13
lift_close_door_clean_side = 14
lift_stop_door_clean_side = 15
lift_emg_door_clean_side = 16
# OUTPUT PLC, INPUT AGV
position_state = 1
place_or_pick_state = 2
barie_state = 3
lift_table_state = 4
light_curtain_state = 5
error_code_1 = 6
error_code_2 = 7
mode_ban_nang_ha = 8
# Định nghĩa bảng lỗi: bit index → tên lỗi
PLC_ERROR_BIT_MAP = {
    0:  "EMG_PASSBOX",
    1:  "SENSOR_2_ACTIVE",
    2:  "ERROR",
    3:  "ERROR",
    4:  "ERROR",
    5:  "ERROR",
    6:  "ERROR",
    7:  "ERROR",
    8:  "AGV STOCK",
    9:  "ERROR",
    10: "ERROR",
    11: "ERROR",
    12: "LIGHT_CURTAIN_ERROR",
    13: "MANUAL_ERROR",
    14: "ERROR",
    15: "ERROR",
}
def read_error_plc():
    result = modbus_tcp_passbox.read_slave_2(1, error_code_1, 1)
    if not result:
        print("read_error_plc: Failed to read register {}".format(error_code_1))
        return None
    raw_value = result[0]  # giá trị 16-bit (0–65535)
    print("read_error_plc: raw register {} = {}".format(error_code_1, raw_value))
    # Tách 16 bit: bit 0 = LSB
    bits = [(raw_value >> i) & 1 for i in range(16)]
    # Map bit → tên lỗi nếu bit = 1
    active_errors = [
        PLC_ERROR_BIT_MAP.get(i, "BIT_{}".format(i))
        for i, bit in enumerate(bits)
        if bit == 1
    ]
    if active_errors:
        print("PLC errors active: {}".format(active_errors))
    return {
        "errors": active_errors,
    }
state = "init"
if __name__ == "__main__":
    modbus_tcp_passbox.connect()
    for coil in range(1, 17):
        modbus_tcp_passbox.write_slave(1, coil, 0)
    print("[INIT] Đã clear tất cả output AGV về 0")
    print(read_error_plc())
    # print(modbus_tcp_passbox.read_slave(1,6,1)[0])
    # print(modbus_tcp_passbox.read_slave(1,barie_state,1)[0])
    # modbus_tcp_passbox.write_slave(1, open_barie_dirty_side, 0)
    # modbus_tcp_passbox.write_slave(1, close_barie_dirty_side, 1)
    # modbus_tcp_passbox.write_slave(1, place_agv_request, 1)
    # time.sleep(4)
    # modbus_tcp_passbox.write_slave(1, lift_open_door_clean_side, 0)
    # modbus_tcp_passbox.write_slave(1, lift_open_door_dirty_side, 0)
    # modbus_tcp_passbox.write_slave(1, lift_close_door_dirty_side, 1)
    # while True:
    #     if(state == "init"):
    #         print(f"[STATE] {state}")

    #         # Clear tất cả output AGV về 0
    #         for coil in range(1, 17):
    #             modbus_tcp_passbox.write_slave(1, coil, 0)
    #         print("[INIT] Đã clear tất cả output AGV về 0")
    #         # modbus_tcp_passbox.write_slave(1, open_dirty_side, 0)
    #         # modbus_tcp_passbox.write_slave(1, open_clean_side, 0)
    #         time.sleep(2)
    #         if(modbus_tcp_passbox.read_slave(1,position_state,1)[0]== 1):
    #             # bàn nâng đã ở dưới
    #             modbus_tcp_passbox.write_slave(1,place_agv_request,0) #yêu cầu place
    #             modbus_tcp_passbox.write_slave(1,open_barie_dirty_side,1) #mở barie dirty side
    #             state = "mo_barie"
    #         elif(modbus_tcp_passbox.read_slave(1,position_state,1)[0]== 2): # bàn nâng ở trên
    #             print("dang ha")
    #             modbus_tcp_passbox.write_slave(1,place_agv_request,1) #yêu cầu place
    #         elif(modbus_tcp_passbox.read_slave(1,position_state,1)[0]== 0): # bàn nâng ở trên
    #             print("0")
    #             modbus_tcp_passbox.write_slave(1,place_agv_request,1) #yêu cầu place
    #         else:
    #             print("no define")
    #     elif(state == "mo_barie"):
    #         print(f"[STATE] {state}")
    #         time.sleep(2)
    #         if(modbus_tcp_passbox.read_slave(1,barie_state,1)[0]== 2): # barie dirty side đã mở
    #             print("barie dirty side đã mở")
    #             modbus_tcp_passbox.write_slave(1,open_barie_dirty_side,0) #mở barie dirty side
    #             modbus_tcp_passbox.write_slave(1,agv_going_passbox,1) #agv đi vao ban nang
    #             state = "going_passbox" # agv dang di vo
    #     elif(state == "going_passbox"):
    #         print(f"[STATE] {state}")
    #         time.sleep(2)
    #         modbus_tcp_passbox.write_slave(1,agv_going_passbox,0) #agv đi vao ban nang
    #         modbus_tcp_passbox.write_slave(1,close_barie_dirty_side,1) #dong barie dirty side
    #         if(modbus_tcp_passbox.read_slave(1,barie_state,1)[0]== 1): # barie dirty side đã đóng
    #             state = "close_barie" # dong barie dirty side
    #             time.sleep(3)
    #     elif(state == "close_barie"):
    #         print(f"[STATE] {state}")
    #         time.sleep(2)
    #         modbus_tcp_passbox.write_slave(1,close_barie_dirty_side,0)
    #         modbus_tcp_passbox.write_slave(1,pick_agv_request,1) #yêu cầu pick
    #         if(modbus_tcp_passbox.read_slave(1,position_state,1)[0]== 2): # bàn nâng ở trên
    #             state = "pick" # agv đẫ pick
    #     elif(state == "pick"):
    #         print(f"[STATE] {state}")
    #         modbus_tcp_passbox.write_slave(1,pick_agv_request,0) #yêu cầu pick
    #         time.sleep(2)
    #         if(modbus_tcp_passbox.read_slave(1,position_state,1)[0]== 2): # bàn nâng ở trên
    #             modbus_tcp_passbox.write_slave(1,lift_open_door_dirty_side,1) #yêu cầu open dirty side
    #             state = "open_dirty_side" # agv đang open dirty side
    #     elif(state == "open_dirty_side"):
    #         print(f"[STATE] {state}")
    #         time.sleep(2)
    #         if(modbus_tcp_passbox.read_slave(1,done_open_dirty_side,1)[0]== 1): # dirty side đã mở
    #             state = "done_open_dirty_side" # dirty side đã mở
    #     elif(state == "done_open_dirty_side"):
    #         print(f"[STATE] {state}")
    #         time.sleep(5)
    #         if(modbus_tcp_passbox.read_slave(1,position_state,1)[0]== 2): # bàn nâng ở trên
    #             state = "done_pick" # agv đã pick
    #     elif(state == "done_pick"):
    #         print(f"[STATE] {state}")
    #         time.sleep(2)
    #         if(modbus_tcp_passbox.read_slave(1,position_state,1)[0]== 1): # bàn nâng ở dưới
    #             modbus_tcp_passbox.write_slave(1,pick_agv_request,1) #yêu cầu pick
    #         elif(modbus_tcp_passbox.read_slave(1,position_state,1)[0]== 2): # bàn nâng ở trên
    #             modbus_tcp_passbox.write_slave(1,lift_open_door_dirty_side,0) #yêu cầu open dirty side
    #             modbus_tcp_passbox.write_slave(1,lift_close_door_dirty_side,1) #yêu cầu close dirty side
    #             modbus_tcp_passbox.write_slave(1,pick_agv_request,0) #yêu cầu pick
    #             state = "close_dirty_side" # dirty side đã đóng
    #     elif(state == "close_dirty_side"):
    #         print(f"[STATE] {state}")
    #         time.sleep(2)
    #         if(modbus_tcp_passbox.read_slave(1,done_close_dirty_side,1)[0]  == 1):
    #             modbus_tcp_passbox.write_slave(1,place_agv_request,1) #yêu cầu place
    #             if(modbus_tcp_passbox.read_slave(1,position_state,1)[0]== 1): # bàn nâng ở dưới
    #                 state = "done_place" # agv đã place
    #     elif(state == "done_place"):
    #         print(f"[STATE] {state}")
    #         time.sleep(2)
    #         if(modbus_tcp_passbox.read_slave(1,position_state,1)[0]== 1): # bàn nâng ở dưới
    #             modbus_tcp_passbox.write_slave(1,open_barie_dirty_side,1) #mở barie dirty side
    #             state = "open_barie_dirty_side" # barie dirty side đã mở
    #     elif(state == "open_barie_dirty_side"):
    #         print(f"[STATE] {state}")
    #         time.sleep(2)
    #         if(modbus_tcp_passbox.read_slave(1,barie_state,1)[0]== 2): # barie dirty side đã mở
    #             state = "done_open_barie_dirty_side" # barie dirty side đã mở
    #     elif(state == "done_open_barie_dirty_side"):
    #         print(f"[STATE] {state}")
    #         modbus_tcp_passbox.write_slave(1,open_barie_dirty_side,0)
    #         time.sleep(4)
    #         print("dong barie")
    #         modbus_tcp_passbox.write_slave(1,close_barie_dirty_side,1) #mở barie dirty side
    #         if(modbus_tcp_passbox.read_slave(1,barie_state,1)[0]== 1): # barie dirty side đã đóng
    #             state = "open_barie_clean_side"
    #     elif(state == "open_barie_clean_side"):
    #         print(f"[STATE] {state}")
    #         modbus_tcp_passbox.write_slave(1,close_barie_dirty_side,0)
    #         modbus_tcp_passbox.write_slave(1,lift_open_door_clean_side,1) #mở barie dirty side
    #         if(modbus_tcp_passbox.read_slave(1,done_open_clean_side,1)[0]== 1): # barie dirty side đã đóng
    #             state = "done_open_barie_clean_side"
    #     elif(state == "done_open_barie_clean_side"):
    #         print(f"[STATE] {state}")
    #         modbus_tcp_passbox.write_slave(1,lift_open_door_clean_side,0) #mở barie dirty side
    #         time.sleep(4)
    #         modbus_tcp_passbox.write_slave(1,lift_close_door_clean_side,1) #mở barie dirty side
    #         if(modbus_tcp_passbox.read_slave(1,done_close_clean_side,1)[0]== 1): # barie dirty side đã đóng
    #             state = "hoanthanh"
    #             modbus_tcp_passbox.write_slave(1,lift_close_door_clean_side,0)
    #     elif(state == "hoanthanh"):
    #         print(f"[STATE] {state}")
    #         break