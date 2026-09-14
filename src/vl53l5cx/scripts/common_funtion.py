#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import os
import sys
import rospkg
import math
import json
import numpy as np
import pandas as pd


def convert_dist_coords(current_value):
    """Compute SinCos Tables."""
    # References
    # https://community.st.com/s/question/0D53W000015XpcBSAS/vl53l5cx-multizone-sensor-get-xyz-of-points-relative-to-origin

    # TODO not use Z
    path_default = os.path.join(
        rospkg.RosPack().get_path("vl53l5cx"),
        "cfg",
        "vl53l5cx_config.json",
    )

    # print(path_default)
    # print("/home/mm/catkin_ws/src/vl53l5cx/cfg/vl53l5cx_config.json")
    with open(path_default, "r") as f:
        data = json.load(f)
    df = pd.DataFrame(data)

    pitch_transform = np.array(
        df.params.VL53L5_Zone_Pitch8x8,
        dtype=np.float32,
    )
    yaw_transform = np.array(
        df.params.VL53L5_Zone_Yaw8x8,
        dtype=np.float32,
    )

    lenth_range = [0] * 64
    centimetros_to_metros = [1000] * 64
    sin0_pitch = [0] * 64
    cos0_pitch = [0] * 64
    sin0_yaw = [0] * 64
    cos0_yaw = [0] * 64
    len_pcl = len(lenth_range)
    range_data = np.array(current_value) / np.array(centimetros_to_metros)
    for z in range(0, len_pcl):
        sin0_pitch[z] = math.sin(math.radians(pitch_transform[z]))
        cos0_pitch[z] = math.cos(math.radians(pitch_transform[z]))
        sin0_yaw[z] = math.sin(math.radians(yaw_transform[z]))
        cos0_yaw[z] = math.cos(math.radians(yaw_transform[z]))

    """Converts data axis Y and Z from axis X."""
    hyp = np.array(range_data) / np.array(sin0_pitch)
    x_pos = cos0_yaw * hyp * cos0_pitch
    y_pos = sin0_yaw * hyp * cos0_pitch
    z_pos = range_data
    # safety margin
    # array_2d_square = np.square(range_data) - np.square(x_pos)
    # real_value_outside_angle = np.sqrt(array_2d_square)
    # # print(array_sqrt)
    # hight_sensor_agv = 0.45
    # distant_value = np.square(real_value_outside_angle) - np.square(
    #     hight_sensor_agv
    # )

    # return np.sqrt(distant_value)
    return y_pos


if __name__ == "__main__":
    pass
