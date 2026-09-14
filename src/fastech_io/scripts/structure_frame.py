import os
import rospkg
import sys
import rospy
from os.path import expanduser


common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

from common_function import EnumString
from agv_msgs.msg import *


class N_ByteResponseFrameData(EnumString):
    HEADER = 1
    LENGTH = 1
    SYNC_NO = 1
    RESERVED_FRAME_TYPE = 1
    DATA = 1
    FRAME_TYPE = 1


class Value_Reserved(EnumString):
    S_VALUE = 0


class Header(EnumString):
    NORMAL = 0
    FRAME_RESPONE_ERROR = 128
    DATA_WITHOUT_RANGE = 129
    FRAME_RECEIVED_ERROR = 130
    ERORR_CRC = 170


class FAS_GetSlaveInfo(EnumString):
    Frame_type = 1
    Sending = 0
    Response_min = 1
    Response_max = 248


class FAS_GetInput(EnumString):
    Frame_type = 192
    Sending = 0
    Response = 9
    Sync_no = 5


class FAS_GetLatchCountAll(EnumString):
    Frame_type = 195
    Sending = 0
    Response = 65
    Sync_no = 6


class FAS_GetOutput(EnumString):
    Frame_type = 197
    Sending = 0
    Response = 9
    Sync_no = 7
    Bytes_SetOutput = 4
    Bytes_ResetOutput = 4


class FAS_SetOutput(EnumString):
    Frame_type = 198
    Sending = 8
    Response = 1
    Sync_no = 8


class FAS_SetTrigger(EnumString):
    Frame_type = 199
    Sending = 13
    Response = 1
    Sync_no = 9
    Bytes_Count = 4
    Bytes_Blank = 2
    Bytes_OnTime = 2
    Bytes_Period = 2
    Bytes_Output = 1


class FAS_SetRunStop(EnumString):
    Frame_type = 200
    Sending = 8
    Response = 1
    Sync_no = 9


class FAS_GetIOLevel(EnumString):
    Frame_type = 202
    Sending = 0
    Response = 9
    Sync_no = 3


class FAS_SetIOLevel(EnumString):
    Frame_type = 203
    Sending = 4
    Response = 1
    Sync_no = 4
