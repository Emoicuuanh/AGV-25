#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import os
import sys
import rospy
import rospkg
import json
import yaml
import socket
import time
from ast import literal_eval
import numpy as np

from std_msgs.msg import Int64, Int16, Int8, String, Empty, Bool

# from agv_msgs.srv import SetDigitalOutput, DataCheck
from std_msgs.msg import Int16MultiArray
from fastech_io.msg import io
from fastech_io.srv import GetIO, SetValueOutput

from std_stamped_msgs.msg import (
    StringStamped,
    StringAction,
    StringFeedback,
    StringResult,
    StringGoal,
    EmptyStamped,
)

# common_func_dir = os.path.join(
#     rospkg.RosPack().get_path("agv_common_library"), "scripts"
# )
# if not os.path.isdir(common_func_dir):
#     common_func_dir = os.path.join(
#         rospkg.RosPack().get_path("agv_common_library"), "release"
#     )
# sys.path.insert(0, common_func_dir)

from structure_frame import *

# from common_function import (
#     MIN_FLOAT,
#     EnumString,
# )


class Library_Name(EnumString):
    FAS_GetInput = 192
    FAS_GetOutput = 197
    FAS_SetOutput = 198
    FAS_SetTrigger = 199
    FAS_SetRunStop = 200
    FAS_GetTrigger = 201
    FAS_GetIOLevel = 202
    FAS_SetIOLevel = 203


class Ezi_io_connect:
    def __init__(self, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        rospy.init_node("ezi_io", anonymous=True)
        rospy.loginfo("Init node " + rospy.get_name())

        self.ip_address = rospy.get_param("~ip_address", "192.168.0.2")
        self.port = rospy.get_param("~port", 3001)
        self.get_io_rate = rospy.get_param("~get_io_rate", 10)
        self.type_G_S = rospy.get_param("~G_type", "OUTPUT")
        self.number_pin = rospy.get_param("~amount_pin", 16)
        self.fasteck_ezi_io = rospy.get_param("~fasteck_ezi_io", dict({}))
        self.program_start = False
        self.amount_pin = self.number_pin
        if self.amount_pin < 17:
            self.real_pin_number = 16
        else:
            self.real_pin_number = 32
        self.per_hafl = int(self.real_pin_number / 2)

        if self.type_G_S == "OUTPUT":
            self.index_set = 0
            self.possition_set = 0
        else:
            if self.real_pin_number < 20:
                self.index_set = 1
                self.possition_set = 8
            else:
                self.index_set = 2
                self.possition_set = 16

        self._frame_default_set = [255, 255, 255, 255, 0, 0, 0, 0]
        self.standard_frame = [0, 0, 0, 0, 0, 0, 0, 0]

        simulation = True
        if simulation:
            self._parameter = (
                b"\xaa\x0c\x04\x00\xc5\x00\xff\xff\xff\xff\xff\xff\xff\xff"
            )
        self.ethernet_protocol = None
        self.pin_out = None
        self.value_pin = None
        self.last_time = rospy.get_time()
        self.bytes_data = 4
        self.check_multiarray = False
        self.frequency_device = self.get_io_rate / 2
        self.time_device_pre = rospy.get_time()
        # Publisher
        # self.pub_io_mornitor = rospy.Publisher(
        #     "fastech_io_mornitor" + "/" + self.type_G_S,
        #     Int16MultiArray,
        #     queue_size=10,
        # )

        # self.logic_io = rospy.Publisher(
        #     "fastech_logic_io" + "/" + self.type_G_S,
        #     Int16MultiArray,
        #     queue_size=10,
        # )
        # self.standard_io_pub = rospy.Publisher(
        #     "fastech_standard_io", StringStamped, queue_size=5
        # )

        rospy.loginfo("Connecting host: {}".format(self.ip_address))
        rospy.loginfo("Set port: {}".format(self.port))
        rospy.loginfo("Connecting port: {}".format(self.port))
        rospy.loginfo("Number pinIO: {}".format(self.number_pin))
        rospy.loginfo("Type pin: {}".format(self.type_G_S))
        self.counter_error_read_data = 0
        self.data_set_output = []
        self.state_set_ouput = False
        self.current_output = []
        self.initial_protocol()

    def setting_pub_subscriber(self):
        if self.type_G_S == "OUTPUT" or self.type_G_S == "BOTH":
            # Subscriber
            rospy.Subscriber("/sub_io", io, self.repon_io_cb)
            rospy.Subscriber("reset_fastech/output", Bool, self.reset_io_cb)
            rospy.Subscriber(
                "fastech_control_multiarray",
                Int16MultiArray,
                self.control_multiarray_cb,
            )
            # rospy.Service("~get_value_io", GetIO, self.get_value_io)
            rospy.Service("~set_value_io", SetValueOutput, self.set_value_io)
            rospy.Service("/add_two_ints", GetIO, self.handle_add_two_ints)

            # Publisher for both
            self.__pub_input = rospy.Publisher(
                "fastech_input",
                Int16MultiArray,
                queue_size=10,
            )
            self.__pub_output = rospy.Publisher(
                "fastech_output",
                Int16MultiArray,
                queue_size=10,
            )

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"]

    def connect_ethernet(self):
        try:
            # socket.setdefaulttimeout(5)
            self.ethernet_protocol = socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM
            )
            server_address = (self.ip_address, self.port)
            self.ethernet_protocol.connect(server_address)
            # self._client, self._address = self.ethernet_protocol.accept()
            self.ethernet_protocol.settimeout(2)
            # Ethernet_protocal.bind(server_address)
            return True
        except Exception as e:
            rospy.logerr("Failed to connect: {}".format(e))
            self.ethernet_protocol = None
            return False

    def initial_protocol(self):

        if self.connect_ethernet():
            self.parameter_adjust()
            if self.amount_pin > 33 or self.amount_pin < 0:
                rospy.logerr(
                    " IO beyond the level, Hardware section is not enough"
                )
            else:
                self.loop()
        else:
            rospy.logerr("Error connect ethernet")

    def start_setting(self, pin, value, level):

        # print("pin" , pin)
        # print("level" , level)
        # print("value" , value)
        value_set = self.check_enough_bytes(value, True)
        logic_level_set = self.check_enough_bytes(level, True)

        value_setting = [logic_level_set, value_set]
        level_bytes_sending = []
        value_set_bytes_sending = []
        value_reset_bytes_sending = []

        # print(" value_set befor {}".format(value))
        # print(" value_set after {}".format(len(pin)))
        # print(" value_set after {}".format((pin)))
        if len(pin) <= self.real_pin_number:
            self.program_start = True
        # print(" logic_level_set after {}".format(logic_level_set))
        # print(" logic_level_set befor {}".format(level))
        # rospy.logerr("len ", len(value_set))
        # print("len x ", len(value_set))
        # print("print value set ", value_set)
        for index in range(self.bytes_data):
            if len(value_set) < self.bytes_data:
                # print("this funtion")
                for z in value_setting:
                    z.append("0")
                _level = self.append_Lint_Lstr(value_setting[0][index])
                _value = self.append_Lint_Lstr(value_setting[1][index])
                _lv = self.bin_dec(_level)
                _vl = self.bin_dec(_value)
            else:
                pass
                # _level = self.append_Lint_Lstr(logic_level_set)
                # _value = self.append_Lint_Lstr(value_set)
                # _lv = self.bin_dec(_level)
                # _vl = self.bin_dec(_value)

            # print("_level", _level)
            # print("_value", _value)
            # print("_lv", _lv)
            # print("_vl", _vl)
            _rs_vl = 255 - _vl
            level_bytes_sending.append(_lv)
            value_set_bytes_sending.append(_vl)
            value_reset_bytes_sending.append(_rs_vl)
            # print(" value_setting[0][index] {}".format(value_setting[0][index]))
            # print(" _level[0][index] {}".format(_level))

        sending_value_start = (
            value_set_bytes_sending + value_reset_bytes_sending
        )
        sending_level = level_bytes_sending
        # print(" sending_level {}".format(sending_level))
        if self.program_start:
            self.set_controller_params(FAS_SetOutput, sending_value_start)
            time.sleep(1)
            self.set_controller_params(FAS_SetIOLevel, sending_level)
            rospy.loginfo("Done setting default")
        else:
            rospy.logerr("Parameter file ERROR")
        self.setting_pub_subscriber()
        # self.program_start = True

    def parameter_adjust(self):

        pin_io = []
        value_io = []
        logic_io = []
        get_parameter = []
        self.io_const = []

        for sensor, params in self.fasteck_ezi_io.items():
            if params["type"] == "Const" or params["type"] == "Input":
                self.io_const.append(params["pin"])
            if self.type_G_S != "OUTPUT":
                if params["type"] == "Output":
                    params["pin"] = params["pin"] + self.per_hafl
                    # print("param", params)
            # print("params" , params)
            get_parameter.append(params)

        get_parameter.sort(key=self.sort_param)

        for x in get_parameter:
            # if self.type_G_S != "OUTPUT":
            #     if x["type"] == "Output":
            #         x["pin"] = x["pin"] + self.per_hafl + 1

            pin_io.append(x["pin"])
            value_io.append(int(x["current_value"]))
            logic_io.append(int(x["logic_type"]))
            # print("pin_io.append  ", (x["pin"]))

        # self.possition_set
        # pin + self.per_hafl
        # print("pin_io  21", pin_io)
        # print("value_io 1", value_io)
        # print("logic_io 1", logic_io)
        # print("len all 1", len(pin_io))
        max_len = 0
        for i in pin_io:
            if i >= max_len:
                max_len = i

        # print("max_len", max_len)
        for i in range(0, max_len):
            if i != pin_io[i]:
                # print("i ", i)
                pin_io.insert(i, i)
                value_io.insert(i, 0)
                logic_io.insert(i, 0)
                # print("pin_io insert ", pin_io)

        # print("pin_io    ", pin_io)
        # print("value_io  ", value_io)
        # print("logic_io  ", logic_io)
        # print("len all 2 ", len(pin_io))
        self.start_setting(pin_io, value_io, logic_io)

    def sort_param(self, parame):
        return parame["pin"]

    def _check_result(self, pin_check, value):

        ___current_response = self.processing_input(FAS_GetOutput, True)
        if ___current_response[pin_check + self.possition_set] == value:
            return True

    def one_pin_valid(self, pin__, value__):
        if value__ != 0 and value__ != 1:
            return False
        else:
            self.set_on_off(pin__, value__)

    def multiarray_control_reponse(self, multiarray, index):
        try:
            current_output = self.current_output
            __standard_frame = current_output[6:]
            origin_standard_frame = current_output[6:10]

            if len(multiarray) != 8:
                # print("ply_frame", current_output)
                # print("multiarray", multiarray)
                for i in range(len(multiarray)):
                    # print(multiarray[i])
                    list_current = self.append_Lint_Lstr(multiarray[i])
                    __pin_hex = int(self.bin_dec(list_current))
                    __standard_frame[index + i] = __pin_hex
                    __standard_frame[index + i + self.bytes_data] = (
                        255 - __pin_hex
                    )
                # print("__standard_frame set ", __standard_frame)

            else:
                list_current = self.append_Lint_Lstr(multiarray)
                __pin_hex = int(self.bin_dec(list_current))
                __standard_frame[index] = __pin_hex
                __standard_frame[index + self.bytes_data] = 255 - __pin_hex
            if origin_standard_frame != __standard_frame[: self.bytes_data]:
                self.set_controller_params(FAS_SetOutput, __standard_frame)
        except:
            pass

    def set_on_off(self, pin, signal):

        if self.type_G_S == "OUTPUT":
            pin_set = pin
        else:
            pin_set = pin + self.per_hafl

        comunic_ation_status = 1
        _v_position = 1
        index = 0
        position = 0

        IO_0_7 = 5
        IO_8_15 = 6
        IO_16_23 = 7
        IO_24_31 = 8

        quantity_1_row = 7
        quantity_2_row = 15
        quantity_3_row = 24
        quantity_4_row = 31

        if pin_set <= quantity_1_row:
            index = IO_0_7 + comunic_ation_status
            position = quantity_1_row - pin_set

        elif quantity_1_row <= pin_set - 1 < quantity_2_row:
            index = IO_8_15 + comunic_ation_status
            position = quantity_2_row - pin_set

        elif pin_set > quantity_2_row and pin_set < quantity_3_row:
            index = IO_16_23 + comunic_ation_status
            position = quantity_3_row - pin_set - _v_position
        else:
            index = IO_24_31 + comunic_ation_status
            position = quantity_4_row - pin_set

        self.change_current_value(index, position, signal)

    def change_current_value(self, index, position, signal):

        reply_frame = self.read_board_io(FAS_GetOutput)
        # for index in range(len(reply_frame)):
        front_change_data = reply_frame[index]
        reply_frame[index] = self.update_value(
            reply_frame[index], position, signal
        )
        after_change = reply_frame[index]
        if front_change_data == after_change:
            rospy.logwarn_throttle(2," You do not change anything")
            # pass
        else:
            # pass
            reply_frame[index + 4] = 255 - reply_frame[index]
            self._parameter = bytes(reply_frame)
            self.fas_control(FAS_SetOutput, reply_frame)

    def fas_control(self, frametype, reply_data_frame):

        len_data_sending = len(reply_data_frame) - frametype.Sending.value
        frame_reply_data = reply_data_frame[len_data_sending:]
        self.set_controller_params(frametype, frame_reply_data)

    def update_value(self, value_io, position, signal):

        level = signal
        value = list(self.decimal_binary(value_io))
        current_value = self.check_enough_bytes(value, False)
        current_value[position] = level
        reversed_value = self.reverse(current_value)

        conver_to_string = ""
        for word in current_value:
            conver_to_string += str(word)
        binary_str = int(conver_to_string)

        reversed_conver_to_string = ""
        for z in reversed_value:
            reversed_conver_to_string += str(z)
        reversed_binary_str = int(reversed_conver_to_string)

        return self.bin_dec(binary_str)

    def set_trigger(self, pin, status, conut_time, period, ontime):

        if self.type_G_S == "OUTPUT":
            pin_set = pin
        else:
            pin_set = pin + self.per_hafl

        _pin = self.dec_to_hex(pin_set, 1, False)
        _status = 1
        _conut_time = self.dec_to_hex(
            conut_time, FAS_SetTrigger.Bytes_Count.value, True
        )
        _period = self.dec_to_hex(
            period, FAS_SetTrigger.Bytes_Period.value, True
        )
        _ontime = self.dec_to_hex(
            ontime, FAS_SetTrigger.Bytes_OnTime.value, True
        )
        _blank = ["00"] * FAS_SetTrigger.Bytes_Blank.value

        if not status:
            self.set_run_stop_trigger(pin_set, status)
        else:
            # set_trigger_data_structure = (
            #     self.dec_to_hex(pin, 1, False)
            #     + self.dec_to_hex(
            #         period, FAS_SetTrigger.Bytes_Period.value, True
            #     )
            #     + ["00"] * FAS_SetTrigger.Bytes_Blank.value
            #     + self.dec_to_hex(
            #         ontime, FAS_SetTrigger.Bytes_OnTime.value, True
            #     )
            #     + ["00"] * FAS_SetTrigger.Bytes_Blank.value
            #     + self.dec_to_hex(
            #         conut_time, FAS_SetTrigger.Bytes_Count.value, True
            #     )
            # )
            set_trigger_data_structure = (
                _pin + _period + _blank + _ontime + _blank + _conut_time
            )
            # print("trigger :", (set_trigger_data_structure))
            frame_reply_data = self.hex_to_dec(set_trigger_data_structure)
            # print("conver_dec :", (frame_reply_data))
            self.set_controller_params(FAS_SetTrigger, frame_reply_data)
            time.sleep(0.1)
            self.set_run_stop_trigger(pin_set, status)

    def set_run_stop_trigger(self, pin, status_run_stop):

        n_pin_per_row = 7
        default_value_pin = [1]
        frame_reply_data = [0] * 8

        if pin > n_pin_per_row:
            index = 1
            position_pin = pin - n_pin_per_row
        else:
            index = 0
            position_pin = pin + 1

        add_new_value = self.n_of_elements(
            default_value_pin, position_pin, 0, 1
        )
        _new_value = self.append_Lint_Lstr(add_new_value)
        value_pin_hex = int(self.bin_dec(_new_value))

        if status_run_stop:
            frame_reply_data[index] = value_pin_hex
        else:
            frame_reply_data[index + 4] = value_pin_hex

        self.set_controller_params(FAS_SetRunStop, frame_reply_data)

    def convert_list_int(self, lists):
        s = [str(integer) for integer in lists]
        a_string = "".join(s)
        res = int(a_string)

    def check_enough_bytes(self, binary, reverse):

        byte = list(binary)
        # 1 byte = 8 bit
        N_Bits = 8
        _byte_enough = []
        if len(byte) < (N_Bits + 1):
            # print("nho hon 8")
            while len(byte) < N_Bits:
                byte.insert(0, "0")
        else:
            for s in self.spl_list(byte, N_Bits):
                # print("s {}".format(len(s)))
                if len(s) < N_Bits:
                    s = self.check_enough_bytes(s, False)
                if reverse:
                    reversed_set = self.reverse(s)
                    _byte_enough.append(reversed_set)
                else:
                    _byte_enough.append(s)
            return _byte_enough

        byte_enough = self.Lsrt_Lint(byte)

        if reverse:
            return self.reverse(byte_enough)
        else:
            return byte_enough

    def __enough_bytes(self, binary, reverse):

        byte = list(binary)
        # 1 byte = 8 bit
        N_Bits = 8
        _byte_enough = []
        if len(byte) < (N_Bits + 1):
            # print("nho hon 8")
            while len(byte) < N_Bits:
                byte.append("0")
                # print("byte", byte)
        else:
            for s in self.spl_list(byte, N_Bits):
                # print("s {}".format(len(s)))
                if len(s) < N_Bits:
                    s = self.__enough_bytes(s, False)
                if reverse:
                    reversed_set = self.reverse(s)
                    _byte_enough.append(reversed_set)
                else:
                    _byte_enough.append(s)
            return _byte_enough

        byte_enough = self.Lsrt_Lint(byte)

        if reverse:
            return self.reverse(byte_enough)
        else:
            return byte_enough

    def n_of_elements(self, l_value, n_size, value_added, positon):
        if len(l_value) < n_size:
            l_value.insert(positon, value_added)
            self.n_of_elements(l_value, n_size, value_added, positon)
        return l_value

    def spl_list(self, lst, n_size):
        for i in range(0, len(lst), n_size):
            yield lst[i : i + n_size]

    def merge(self, value, n_size):
        merge_value = []
        for index in range(len(value)):
            if index % n_size != 0:
                varible_z = value[index - 1] + value[index]
                merge_value.append(varible_z)
        return merge_value

    def reverse(self, string):
        string = string[::-1]
        return string

    def Lsrt_Lint(self, list):
        new_grades = [int(g) for g in list]
        return new_grades

    def append_Lint_Lstr(self, Lint):
        conver_to_string = ""
        if isinstance(Lint, int):
            rospy.logerr(" load Parameter Fail")
            binary_str = 0
        else:
            for word in Lint:
                conver_to_string += str(word)
            binary_str = int(conver_to_string)
        return binary_str

    def bin_dec(self, bin):
        decimal = 0
        power = 1
        while bin > 0:
            rem = bin % 10
            bin = bin // 10
            decimal += rem * power
            power = power * 2

        return decimal

    def decimal_binary(self, dec):
        return bin(dec).replace("0b", "")

    def bin_hexa(self, binary):
        num = int(binary, 2)
        # hex_num = hex(num)
        hex_num = format(num, "x")
        return hex_num

    def dec_to_hex(self, dec, n_byte, reverse):
        hexa = []
        POSITON = 0
        VALUE = "0"
        lengh_byte = n_byte * 2
        while dec != 0:
            remainder = dec % 16
            if remainder < 10:
                hexa.append(chr(remainder + 48))
            else:
                hexa.append(chr(remainder + 55))
            dec = dec // 16

        hexa.reverse()
        _hexa = self.merge(
            self.n_of_elements(hexa, lengh_byte, VALUE, POSITON), 2
        )
        if reverse:
            _hexa.reverse()
        return _hexa

    def hex_to_dec(self, l_hex):
        _dec = []
        for i in l_hex:
            dec = int(i, 16)
            _dec.append(dec)
        return _dec

    def reset_io(self):

        _frame_default_reset = []
        for rv in reversed(self._frame_default_set):
            _frame_default_reset.append(rv)
        self.set_controller_params(FAS_SetOutput, _frame_default_reset)

    def set_io_all(self):
        self.set_controller_params(FAS_SetOutput, self._frame_default_set)

    def update_borad(self, values):
        return values

    def read_level_io(self):
        pass

    def read_board_io(self, frametype):

        Sync_Reserved = N_ByteResponseFrameData.SYNC_NO.value
        Reserved = N_ByteResponseFrameData.RESERVED_FRAME_TYPE.value
        Frame_type = N_ByteResponseFrameData.FRAME_TYPE.value
        length_sending = Sync_Reserved + Reserved + Frame_type

        # Data_Configuration_sending = [
        #     Header.ERORR_CRC.value,
        #     length_sending,
        #     Sync_Reserved,
        #     Reserved,
        #     Library_Name.FAS_GetOutput.value,
        # ]
        # self.ethernet_protocol.send(bytes.fromhex("AA 03 04 00 C5"))
        # if self.type_G_S:
        #     Data_Configuration_sending = [
        #         Header.ERORR_CRC.value,
        #         length_sending,
        #         FAS_GetInput.Sync_no.value,
        #         Value_Reserved.S_VALUE.value,
        #         Library_Name.FAS_GetInput.value,
        #     ]
        # else:
        Data_Configuration_sending = [
            Header.ERORR_CRC.value,
            length_sending,
            frametype.Sync_no.value,
            Value_Reserved.S_VALUE.value,
            frametype.Frame_type.value,
        ]
        _frametype = []
        # print(" Data_Configuration_sending".format(Data_Configuration_sending))
        try:
            read_board = self.device_control(Data_Configuration_sending)
            # print("frametype {}".format(frametype))
            # print("read_board[4] {}".format(read_board[4]))
            if len(read_board) > 4:
                if read_board[4] != frametype.Frame_type.value:
                    _frametype = []
                else:
                    _frametype = read_board
                # print(read_board)
        except:
            rospy.logerr(
                "error when hanlde read data"
            )

        return _frametype

    def send_topic(self):

        array_data_reponse_io = Int16MultiArray()
        array_logic_io = Int16MultiArray()

        if self.type_G_S == "OUTPUT":
            v_current_response = self.processing_input(FAS_GetOutput, True)
        elif self.type_G_S == "INPUT":
            v_current_response = self.processing_input(FAS_GetInput, True)
        else:
            array_input = Int16MultiArray()
            array_output = Int16MultiArray()
            input_current_response = self.processing_input(FAS_GetInput, True)
            output_current_response = self.processing_input(FAS_GetOutput, True)

            in_response = input_current_response[: self.per_hafl]
            out_response = output_current_response[
                self.per_hafl : self.real_pin_number
            ]
            array_input.data = in_response
            array_output.data = out_response
            v_current_response = in_response + out_response
            if len(in_response) > 0:
                self.__pub_input.publish(array_input)
                all_ones_input = all(element == 1 for element in in_response)
                if all_ones_input:
                    rospy.logwarn_throttle(2,"publish all element input = 1")
            if len(out_response) > 0:
                self.__pub_output.publish(array_output)
                all_ones_output = all(element == 1 for element in out_response)
                if all_ones_output:
                    rospy.logwarn_throttle(2,"publish all element output = 1")
            if self.state_set_ouput:
                self.set_ouput(self.data_set_output)
                self.state_set_ouput = False

        # b'\xaa\x04\x06\x00\xcb\x00'
        # logic_current_response = self.processing_input(FAS_GetIOLevel, True)

        # array_data_reponse_io.data = v_current_response
        # array_logic_io.data = logic_current_response

        # self.pub_io_mornitor.publish(array_data_reponse_io)
        # self.logic_io.publish(array_logic_io)

        # self.sensors_msg_dict = {}
        # for sensor, params in self.fasteck_ezi_io.items():
        #     for i in range(len(v_current_response)):
        #         if v_current_response[i]:
        #             if params["pin"] == i:
        #                 self.sensors_msg_dict[i] = sensor

        # std_io_msg = StringStamped()
        # std_io_msg.stamp = rospy.Time.now()
        # std_io_msg.data = json.dumps(self.sensors_msg_dict, indent=2)
        # self.standard_io_pub.publish(std_io_msg)

    def processing_input(self, frametype, bit_reverse):

        response_data = []
        m_line = 4
        frame_set_data = []
        for i in reversed(range(m_line)):
            frame_set_data.append(frametype.Response.value - i)

        try:
            fas_get = self.read_board_io(frametype)
            if frametype == FAS_GetOutput:
                self.current_output = fas_get
            if fas_get == None or len(fas_get) == 0:
                rospy.logwarn_throttle(1,"ignore because len data = 0")
            else:
                if len(fas_get) < 10:
                    # Return 00x00
                    rospy.logerr("ignore data because length < 10")
                else:
                    for index in frame_set_data:
                        current_io = self.decimal_binary(fas_get[index])
                        enough_bytes = self.check_enough_bytes(
                            list(current_io), bit_reverse
                        )
                        response_data += enough_bytes
        except IndexError:
            print("Receive frames not data")

        int_response_data = self.Lsrt_Lint(list(response_data))
        length_format_response = int_response_data[: self.amount_pin]
        # print(int_response_data)
        return length_format_response

    def set_controller_params(self, frametype, reply_data_sending):

        # Response_FrameData = [170, 11, 6, 0, 198, 255, 255, 255, 255, 0, 0, 0, 0]
        # red_board value      [170, 12, 4, 0, 197, 0, 255, 255, 255, 255, 0, 0, 0, 0]
        starting_position = 6

        _data_structure = reply_data_sending
        _SyncNo = starting_position
        _Reserved = Value_Reserved.S_VALUE.value
        # (Sync No. + Reserved + Frame type + Data)
        len_data = len(_data_structure)

        if len_data != frametype.Sending.value:
            rospy.logerr(" DATA Frame is not right ")
        else:
            # print("len_data {}".format(len_data))
            total_Length = (
                N_ByteResponseFrameData.SYNC_NO.value
                + N_ByteResponseFrameData.RESERVED_FRAME_TYPE.value
                + N_ByteResponseFrameData.FRAME_TYPE.value
                + len_data
            )
            Response_Frame_structure = [
                Header.ERORR_CRC.value,
                total_Length,
                _SyncNo,
                _Reserved,
                frametype.Frame_type.value,
            ] + _data_structure

            # rospy.logerr(
            #     "Response_Frame_structure {}".format(Response_Frame_structure)
            # )
            if Response_Frame_structure is None:
                rospy.logerr(" Response_Frame_structure is NoneType")
            else:
                read_board = self.device_control(Response_Frame_structure)

    # CALLBACK
    def repon_io_cb(self, msg):

        self.pin_out = msg.pin
        self.value_pin = msg.value

        for i in self.io_const:
            if msg.pin == i:
                rospy.logwarn("IO is const value")
                return
        if self.pin_out > 33 or self.amount_pin < 0:
            rospy.logerr("Hardware section is not enough")
        else:
            if self.pin_out > self.number_pin:
                rospy.logerr("you have not initialized this IO")
            else:
                self.one_pin_valid(self.pin_out, self.value_pin)
                # self.set_on_off(self.pin_out, self.value_pin)
                if self._check_result(self.pin_out, self.value_pin):
                    pass
                    # print("okkkkkkkk")
                    # self.set_on_off(self.pin_out, self.value_pin)
                # else:
                #     rospy.logerr(" Error data input ")
                return

    def reset_io_cb(self, msg):
        if not msg.data:
            self.reset_io()
        else:
            self.set_io_all()

    def clear_buffer(self):
        """ Clear the receive buffer by reading any remaining data """
        if self.ethernet_protocol is None:
            return
        try:
            while True:
                data = self.ethernet_protocol.recv(4096)
                rospy.logwarn(data)
                if not data:
                    break
        except socket.timeout:
            rospy.logerr("clear_buffer --> No data received because socket.timeout, retrying...")
        except Exception as e:
            rospy.logerr("clear_buffer --> read data error : {}".format(str(e)))


    def device_control(self, frame_type_):

        if self.ethernet_protocol is None:
            rospy.logwarn_throttle(1, "Reconnecting...")
            self.connect_ethernet()
        if self.ethernet_protocol is not None:
            try:
                self.time_device_pre = rospy.get_time()
                # self.clear_buffer()  # Clear buffer before sending
                self.ethernet_protocol.sendall(bytes(frame_type_))
                read_board = list(self.ethernet_protocol.recv(4096))
                rospy.logwarn_throttle(2, "Receive data: {}".format(read_board))
                rospy.logwarn_throttle(2, "len data: {}".format(len(read_board)))
            except socket.timeout:
                read_board = None
                rospy.logerr("No data received because socket.timeout, retrying...")
                self.counter_error_read_data += 1
                if self.counter_error_read_data > 2:
                    self.counter_error_read_data = 0
                    self.ethernet_protocol = None
            except Exception as e:
                read_board = None
                rospy.logerr("read data error : {}".format(str(e)))
                self.counter_error_read_data += 1
                if self.counter_error_read_data > 2:
                    self.counter_error_read_data = 0
                    self.ethernet_protocol = None
        else:
            return None

        return read_board

    def control_multiarray_cb(self, msg):
        self.state_set_ouput = True
        self.data_set_output = msg.data
        if all(element == 1 for element in msg.data):
            rospy.loginfo("RECEIVE CONTROL OUTPUT All elements are equal to 1")

    def set_ouput(self, data):
        self.check_multiarray = False
        for id in data:
            if id != 1 and id != 0:
                self.check_multiarray = False
                break
            else:
                self.check_multiarray = True

        if self.check_multiarray:
            self.multiarray_control_reponse(
                self.__enough_bytes(data, True), self.index_set
            )
        else:
            print(" Error input data ")


    def get_value_io(self, req):
        for sensor, params in self.fasteck_ezi_io.items():
            if params["pin"] == req.pin:
                name = sensor
                # print(sensor)
                # output_voltage = params["current_value"]
        return name

    def set_value_io(self, req):
        # rospy.loginfo("Pin : {}".format(req.pin))
        if (
            req.pin < 0
            or req.pin > 31
            or req.pin > self.amount_pin
            or req.conut_time > 4294967295
            or req.period_ms > 65535
            or req.ontime_ms > 65535
            or req.conut_time < 1
            and req.conut_time != 0
            or req.ontime_ms < 2
            and req.ontime_ms != 0
        ):
            result = False
        else:
            if req.mode == 1:
                self.set_on_off(req.pin, int(req.value))
                # if self._check_result(req.pin, int(req.value)):
                #     result = True
                # else:
                #     result = False
                result = True
            elif req.mode == 2:
                self.set_trigger(
                    req.pin,
                    int(req.value),
                    req.conut_time,
                    req.period_ms,
                    req.ontime_ms,
                )
                result = True

        return result

    def handle_add_two_ints(self, req):
        result = req.a + req.b + req.c + req.b
        rospy.loginfo(
            "Sum of " + str(req.a) + " and " + str(req.b) + " is " + str(result)
        )
        return result

    def loop(self):
        rate = rospy.Rate(self.get_io_rate)
        while not rospy.is_shutdown():

            if rospy.get_time() - self.time_device_pre >= (
                1.0 / self.frequency_device
            ):
                rospy.logerr("frequency_device very slow ")

            self.send_topic()
            rate.sleep()


def parse_opts():
    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option(
        "-d",
        "--ros_debug",
        action="store_true",
        dest="log_debug",
        default=False,
        help="log_level=rospy.DEBUG",
    )
    parser.add_option(
        "-p",
        "--config_path",
        dest="config_path",
        default=os.path.join(
            rospkg.RosPack().get_path("fastech_io"),
            "cfg",
            # "set_safety_goal.json",
        ),
    )

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    Ezi_io_connect(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
