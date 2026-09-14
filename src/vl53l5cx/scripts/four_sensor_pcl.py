#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from re import A
import os
import sys
import math
import json
import rospy
import rospkg
import numpy as np
import pandas as pd
import message_filters
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField


# from std_stamped_msgs.msg import StringStamped
# from vl53l5cx.msg import Vl53l5cxRanges, Safety_Esp, vl53l5cx_safety
# import message_filters
from vl53l5cx.msg import Vl53l5cxRanges, Safety_Esp, vl53l5cx_safety

type_mappings = [
    (PointField.INT8, np.dtype("int8")),
    (PointField.UINT8, np.dtype("uint8")),
    (PointField.INT16, np.dtype("int16")),
    (PointField.UINT16, np.dtype("uint16")),
    (PointField.INT32, np.dtype("int32")),
    (PointField.UINT32, np.dtype("uint32")),
    (PointField.FLOAT32, np.dtype("float32")),
    (PointField.FLOAT64, np.dtype("float64")),
]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)


class PointCloudNode:
    def __init__(self, *args, **kwargs):
        self.init_variable(*args, **kwargs)

        rospy.init_node("pointcloud_publisher_node", anonymous=True)
        rospy.loginfo("Init node " + rospy.get_name())
        self.df = None
        self.len_pcl = 64

        self.x_pos = [0] * 64
        self.y_pos = [0] * 64
        self.z_pos = [0] * 64
        self.sin0_yaw = [0] * 64
        self.cos0_yaw = [0] * 64
        self.sin0_pitch = [0] * 64
        self.cos0_pitch = [0] * 64
        self.yaw_transform = [] * 64
        self.pitch_transform = [] * 64

        self.lenth_range = [0] * 64
        self.centimetros_to_metros = [1000] * 64
        self.open_json()

        # Publisher
        self.lidar1_pub_pcl = rospy.Publisher(
            "pointcloud_lidar1", PointCloud2, queue_size=5
        )
        self.lidar2_pub_pcl = rospy.Publisher(
            "pointcloud_lidar2", PointCloud2, queue_size=5
        )
        self.lidar3_pub_pcl = rospy.Publisher(
            "pointcloud_lidar3", PointCloud2, queue_size=5
        )
        self.lidar4_pub_pcl = rospy.Publisher(
            "pointcloud_lidar4", PointCloud2, queue_size=5
        )

        # subscribe
        rospy.Subscriber("/vl35l5cx_r1", Vl53l5cxRanges, self.sensor1_vl53l5cx_cb)
        rospy.Subscriber("/vl35l5cx_r2", Vl53l5cxRanges, self.sensor2_vl53l5cx_cb)
        rospy.Subscriber("/vl35l5cx_r3", Vl53l5cxRanges, self.sensor3_vl53l5cx_cb)
        rospy.Subscriber("/vl35l5cx_r4", Vl53l5cxRanges, self.sensor4_vl53l5cx_cb)
        # rospy.Subscriber("/vl35l5cx_r4", Vl53l5cxRanges, self.sensor4_vl53l5cx_cb)

        self.loop()

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"]

    def vl53l5_pcl(self, pos_x, pos_y, pos_z, frame_id="frame_id"):
        """Sensor lidar 1 to PointCloud2"""
        xyz_zone_coordinates = np.zeros(
            (self.len_pcl,),
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        xyz_zone_coordinates["x"] = pos_x
        xyz_zone_coordinates["y"] = pos_y
        xyz_zone_coordinates["z"] = pos_z
        cloud_msg = self.array_to_pointcloud2(
            xyz_zone_coordinates, stamp=rospy.Time.now(), frame_id=frame_id
        )

        """Publish sensor vl53l5cx to PointCloud2."""
        if frame_id == "base_laser_1":
            self.lidar1_pub_pcl.publish(cloud_msg)
        elif frame_id == "base_laser_2":
            self.lidar2_pub_pcl.publish(cloud_msg)
        elif frame_id == "base_laser_3":
            self.lidar3_pub_pcl.publish(cloud_msg)
        elif frame_id == "base_laser_4":
            self.lidar4_pub_pcl.publish(cloud_msg)
        else:
            return

    def convert_dist_coords(self, range_data, frame_id="frame_id"):
        """Compute SinCos Tables."""
        # References
        # https://community.st.com/s/question/0D53W000015XpcBSAS/vl53l5cx-multizone-sensor-get-xyz-of-points-relative-to-origin
        # print(self.df.VL53L5_Zone_Pitch8x8)

        self.len_pcl = len(self.lenth_range)
        for zone_number in range(0, self.len_pcl):
            self.sin0_pitch[zone_number] = math.sin(
                math.radians(self.pitch_transform[zone_number])
            )
            self.cos0_pitch[zone_number] = math.cos(
                math.radians(self.pitch_transform[zone_number])
            )
            self.sin0_yaw[zone_number] = math.sin(
                math.radians(self.yaw_transform[zone_number])
            )
            self.cos0_yaw[zone_number] = math.cos(
                math.radians(self.yaw_transform[zone_number])
            )

        """Converts Range LEFT data axis Y and Z from axis X."""
        hyp = range_data / self.sin0_pitch
        self.x_pos = self.cos0_yaw * hyp * self.cos0_pitch
        self.y_pos = self.sin0_yaw * hyp * self.cos0_pitch
        self.z_pos = range_data

        self.vl53l5_pcl(self.x_pos, self.y_pos, self.z_pos, frame_id=frame_id)

    def array_to_pointcloud2(self, cloud_arr, stamp=None, frame_id=None):
        """Converts a numpy record array to a sensor_msgs.msg.PointCloud2."""
        # References https://github.com/dimatura/pypcd/blob/master/pypcd/numpy_pc2.py
        # make it 2d (even if height will be 1)
        cloud_arr = np.atleast_2d(cloud_arr)
        cloud_msg = PointCloud2()

        if stamp is not None:
            cloud_msg.header.stamp = stamp
        if frame_id is not None:
            cloud_msg.header.frame_id = frame_id
        cloud_msg.height = cloud_arr.shape[0]
        cloud_msg.width = cloud_arr.shape[1]
        # print("Point_cloud_data {}".format(cloud_arr.dtype.names))
        cloud_msg.fields = self.dtype_to_fields(cloud_arr.dtype)
        cloud_msg.is_bigendian = False  # assumption
        cloud_msg.point_step = cloud_arr.dtype.itemsize
        cloud_msg.row_step = cloud_msg.point_step * cloud_arr.shape[1]
        cloud_msg.is_dense = all(
            [np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names]
        )
        # cloud_msg.data = cloud_arr.tostring()
        cloud_msg.data = cloud_arr.tobytes()

        return cloud_msg

    def dtype_to_fields(self, dtype):
        """Convert a numpy record datatype into a list of PointFields."""
        fields = []
        for field_name in dtype.names:
            np_field_type, field_offset = dtype.fields[field_name]
            pf = PointField()
            pf.name = field_name
            if np_field_type.subdtype:
                item_dtype, shape = np_field_type.subdtype
                pf.count = np.prod(shape)
                np_field_type = item_dtype
            else:
                pf.count = 1

            pf.datatype = nptype_to_pftype[np_field_type]
            pf.offset = field_offset
            fields.append(pf)
        return fields

    # Callback
    def sensor1_vl53l5cx_cb(self, msg):
        vl53l5cxranger = np.array(msg.range) / self.centimetros_to_metros
        self.convert_dist_coords(vl53l5cxranger, frame_id="base_laser_1")

    def sensor2_vl53l5cx_cb(self, msg):
        vl53l5cxranger = np.array(msg.range) / self.centimetros_to_metros
        self.convert_dist_coords(vl53l5cxranger, frame_id="base_laser_2")

    def sensor3_vl53l5cx_cb(self, msg):
        vl53l5cxranger = np.array(msg.range) / self.centimetros_to_metros
        self.convert_dist_coords(vl53l5cxranger, frame_id="base_laser_3")

    def sensor4_vl53l5cx_cb(self, msg):
        vl53l5cxranger = np.array(msg.range) / self.centimetros_to_metros
        self.convert_dist_coords(vl53l5cxranger, frame_id="base_laser_4")

    def open_json(self):
        """Open Json file."""
        with open(self.config_path, "r") as f:
            data = json.load(f)
        self.df = pd.DataFrame(data)

        self.pitch_transform = np.array(
            self.df.params.VL53L5_Zone_Pitch8x8,
            dtype=np.float32,
        )
        self.yaw_transform = np.array(
            self.df.params.VL53L5_Zone_Yaw8x8,
            dtype=np.float32,
        )

    def loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rospy.sleep(0.001)


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
            rospkg.RosPack().get_path("vl53l5cx"),
            "cfg",
            "vl53l5cx_config.json",
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
    PointCloudNode(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
