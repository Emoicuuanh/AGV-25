#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from re import A
import sys
import json
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import numpy as np
import pandas as pd
import math


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
    def __init__(self):
        _log_level = rospy.INFO
        if "debug" in sys.argv:
            _log_level = rospy.DEBUG

        rospy.init_node("pointcloud_publisher_node", anonymous=True)
        rospy.loginfo("Init node " + rospy.get_name())
        self.df = None
        self.Xpos = None
        self.Zpos = None
        self.Ypos = None
        self.len_pcl = 64
        self.SinOfPitch = [1] * 64
        self.SinO_Z = [0] * 64
        self.CosO_X = [0] * 64
        self.open_json()

        # subscribe
        rospy.Subscriber("/vl35l5cx_ranges", Vl53l5cxRanges, self.vl53l5cx_range_cb)
        # Publisher
        self.pub = rospy.Publisher("pointcloud_topic", PointCloud2, queue_size=5)
        self.loop()

    def vl53l5_pcl(self):
        """Publish sensor vl53l5cx to PointCloud2."""
        XYZ_ZoneCoordinates = np.zeros(
            (self.len_pcl,),
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        XYZ_ZoneCoordinates["x"] = self.Xpos
        XYZ_ZoneCoordinates["y"] = self.SinOfPitch
        # XYZ_ZoneCoordinates["y"] = np.random.random((self.len_pcl,))
        XYZ_ZoneCoordinates["z"] = self.Zpos
        cloud_msg = self.array_to_pointcloud2(
            XYZ_ZoneCoordinates, stamp=rospy.Time.now(), frame_id="map"
        )
        self.pub.publish(cloud_msg)

    # Callback
    def vl53l5cx_range_cb(self, msg):
        """Converts data axis Y and Z from axis X."""
        array_sensor = np.array(msg.range)
        self.axisY_transform = np.array(
            self.df.Axes_Y,
            dtype=np.float32,
        )
        self.axisZ_transform = np.array(
            self.df.Axes_z,
            dtype=np.float32,
        )
        lenth_range = len(self.axisZ_transform)

        for ZoneNum in range(0, lenth_range):
            self.SinO_Z[ZoneNum] = math.sin(math.radians(self.axisZ_transform[ZoneNum]))
            self.CosO_X[ZoneNum] = math.cos(math.radians(self.axisZ_transform[ZoneNum]))

        self.Xpos = array_sensor * self.CosO_X
        self.Ypos = array_sensor * self.axisY_transform
        self.Zpos = array_sensor * self.SinO_Z

    def loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.vl53l5_pcl()
            rospy.sleep(0.001)

    def open_json(self):
        """Open Json file."""
        with open("/home/mm/catkin_ws/src/vl53l5cx/cfg/data.json", "r") as f:
            data = json.load(f)
        self.df = pd.DataFrame(data)

    def array_to_pointcloud2(self, cloud_arr, stamp=None, frame_id=None):
        """Converts a numpy record array to a sensor_msgs.msg.PointCloud2."""
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


if __name__ == "__main__":
    PC_vl53l5cx = PointCloudNode()
