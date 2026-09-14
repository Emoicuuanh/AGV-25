#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from re import A
import os
import sys
import math
import json
import rospy
import rospkg
import pandas as pd
import numpy as np

from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
from vl53l5cx.msg import Vl53l5cxRanges, Safety_Esp, vl53l5cx_safety

# import ros_numpy
# import pcl  # pip3 install python-pcl

# from std_stamped_msgs.msg import StringStamped
# from vl53l5cx.msg import Vl53l5cxRanges, Safety_Esp, vl53l5cx_safety
# import message_filters

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


class SafetyPointCloud:
    def __init__(self, *args, **kwargs):
        self.init_variable(*args, **kwargs)

        rospy.init_node("Safety_pcl", anonymous=True)
        rospy.loginfo("Init node " + rospy.get_name())
        self.df = None
        self.len_pcl = 5
        self.markers = []
        self.x_pos = [0] * 5
        self.y_pos = [0] * 5
        self.z_pos = [0] * 5
        self.sin0_yaw = [0] * 4
        self.cos0_yaw = [0] * 4
        self.sin0_pitch = [0] * 4
        self.cos0_pitch = [0] * 4
        self.centimetros_to_metros = [1000] * 4

        self.open_json()
        # Publisher
        self.array_maker = rospy.Publisher("maker_array", MarkerArray, queue_size=1)

        self.pub_line_min_dist = rospy.Publisher(
            "test_visualization_marker", Marker, queue_size=1
        )
        self.pub_safe_zone_n0 = rospy.Publisher(
            "Zone_safety_0", PointCloud2, queue_size=5
        )
        self.pub_safe_zone_n1 = rospy.Publisher(
            "Zone_safety_1", PointCloud2, queue_size=5
        )
        self.pub_safe_zone_n2 = rospy.Publisher(
            "Zone_safety_2", PointCloud2, queue_size=5
        )
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"]

    def vl53l5_pcl(self, x_pose_safety, y_pose_safety, frame_id="frame_id"):
        """Publish sensor vl53l5cx to PointCloud2."""
        XYZ_ZoneCoordinates = np.zeros(
            (self.len_pcl,),
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        XYZ_ZoneCoordinates["x"] = x_pose_safety
        XYZ_ZoneCoordinates["y"] = y_pose_safety
        XYZ_ZoneCoordinates["z"] = self.z_pos
        # XYZ_ZoneCoordinates["y"] = np.random.random((self.len_pcl,))

        cloud_msg = self.array_to_pointcloud2(
            XYZ_ZoneCoordinates, stamp=rospy.Time.now(), frame_id=frame_id
        )

        """Publish sensor vl53l5cx to PointCloud2."""
        if frame_id == "safety_zone_n0":
            self.pub_safe_zone_n0.publish(cloud_msg)
        elif frame_id == "safety_zone_n1":
            self.pub_safe_zone_n1.publish(cloud_msg)
        elif frame_id == "safety_zone_n2":
            self.pub_safe_zone_n2.publish(cloud_msg)
        else:
            return

    def maker_array(self, id, point_1, point_2, color):

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pouring_visualization"
        marker.action = Marker.ADD
        marker.pose = Pose(Point(0, 0, 0), Quaternion(w=1.0))
        marker.scale = Vector3(0.03, 0.03, 0.03)

        marker.id = id
        marker.points = [point_1, point_2]
        marker.color = color

        # self.pub_line_min_dist.publish(marker)
        markerArray = MarkerArray()
        markerArray.markers.append(marker)
        self.array_maker.publish(markerArray)

    def make_table_markers(self):
        # safety area number 0
        self.markers.append(
            self.maker_array(
                1,
                Point(self.x_pose_safety_0[0], self.y_pose_safety_0[0], self.z_pos[0]),
                Point(self.x_pose_safety_0[1], self.y_pose_safety_0[1], self.z_pos[0]),
                ColorRGBA(1.0, 0.0, 0.0, 0.7),
            )
        )
        self.markers.append(
            self.maker_array(
                2,
                Point(self.x_pose_safety_0[1], self.y_pose_safety_0[1], self.z_pos[0]),
                Point(self.x_pose_safety_0[2], self.y_pose_safety_0[2], self.z_pos[0]),
                ColorRGBA(1.0, 0.0, 0.0, 0.7),
            )
        )
        self.markers.append(
            self.maker_array(
                3,
                Point(self.x_pose_safety_0[2], self.y_pose_safety_0[2], self.z_pos[0]),
                Point(self.x_pose_safety_0[3], self.y_pose_safety_0[3], self.z_pos[0]),
                ColorRGBA(1.0, 0.0, 0.0, 0.7),
            )
        )
        self.markers.append(
            self.maker_array(
                4,
                Point(self.x_pose_safety_0[3], self.y_pose_safety_0[3], self.z_pos[0]),
                Point(self.x_pose_safety_0[4], self.y_pose_safety_0[4], self.z_pos[0]),
                ColorRGBA(1.0, 0.0, 0.0, 0.7),
            )
        )

        # safety area number 1
        self.markers.append(
            self.maker_array(
                5,
                Point(self.x_pose_safety_1[0], self.y_pose_safety_1[0], self.z_pos[0]),
                Point(self.x_pose_safety_1[1], self.y_pose_safety_1[1], self.z_pos[0]),
                ColorRGBA(1.0, 1.0, 0.0, 0.7),
            )
        )
        self.markers.append(
            self.maker_array(
                6,
                Point(self.x_pose_safety_1[1], self.y_pose_safety_1[1], self.z_pos[0]),
                Point(self.x_pose_safety_1[2], self.y_pose_safety_1[2], self.z_pos[0]),
                ColorRGBA(1.0, 1.0, 0.0, 0.7),
            )
        )
        self.markers.append(
            self.maker_array(
                7,
                Point(self.x_pose_safety_1[2], self.y_pose_safety_1[2], self.z_pos[0]),
                Point(self.x_pose_safety_1[3], self.y_pose_safety_1[3], self.z_pos[0]),
                ColorRGBA(1.0, 1.0, 0.0, 0.7),
            )
        )
        self.markers.append(
            self.maker_array(
                8,
                Point(self.x_pose_safety_1[3], self.y_pose_safety_1[3], self.z_pos[0]),
                Point(self.x_pose_safety_1[4], self.y_pose_safety_1[4], self.z_pos[0]),
                ColorRGBA(1.0, 1.0, 0.0, 0.7),
            )
        )

        # safety area number 2
        self.markers.append(
            self.maker_array(
                9,
                Point(self.x_pose_safety_2[0], self.y_pose_safety_2[0], self.z_pos[0]),
                Point(self.x_pose_safety_2[1], self.y_pose_safety_2[1], self.z_pos[0]),
                ColorRGBA(0.0, 1.0, 0.0, 0.7),
            )
        )
        self.markers.append(
            self.maker_array(
                10,
                Point(self.x_pose_safety_2[1], self.y_pose_safety_2[1], self.z_pos[0]),
                Point(self.x_pose_safety_2[2], self.y_pose_safety_2[2], self.z_pos[0]),
                ColorRGBA(0.0, 1.0, 0.0, 0.7),
            )
        )
        self.markers.append(
            self.maker_array(
                11,
                Point(self.x_pose_safety_2[2], self.y_pose_safety_2[2], self.z_pos[0]),
                Point(self.x_pose_safety_2[3], self.y_pose_safety_2[3], self.z_pos[0]),
                ColorRGBA(0.0, 1.0, 0.0, 0.7),
            )
        )
        self.markers.append(
            self.maker_array(
                12,
                Point(self.x_pose_safety_2[3], self.y_pose_safety_2[3], self.z_pos[0]),
                Point(self.x_pose_safety_2[4], self.y_pose_safety_2[4], self.z_pos[0]),
                ColorRGBA(0.0, 1.0, 0.0, 0.7),
            )
        )

    def array_to_xyz_pointcloud2(
        self, cloud_arr, stamp=None, frame_id=None, merge_rgb=False
    ):
        """convert an Nx3 float array to an xyz point cloud.
        preserves (scalar) dtype of input.
        TODO: untested
        """
        cloud_arr = np.asarray(cloud_arr)
        if not cloud_arr.ndim == 2:
            raise ValueError("cloud_arr must be 2D array")
        if not cloud_arr.shape[1] == 3:
            raise ValueError("cloud_arr shape must be Nx3")
        xyz = cloud_arr.view(
            np.dtype(
                [("x", cloud_arr.dtype), ("y", cloud_arr.dtype), ("z", cloud_arr.dtype)]
            )
        ).squeeze()
        return self.array_to_pointcloud2(
            xyz, stamp=stamp, frame_id=frame_id, merge_rgb=merge_rgb
        )

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

    def setsafety_pcl(self):

        self.x_pose_safety_0 = []
        self.y_pose_safety_0 = []
        self.x_pose_safety_1 = []
        self.y_pose_safety_1 = []
        self.x_pose_safety_2 = []
        self.y_pose_safety_2 = []

        for safety in self.df.params.safety:
            if safety["index"] == 0:
                for pos_safety_0 in safety["data"]:
                    self.x_pose_safety_0.append(pos_safety_0[0])
                    self.y_pose_safety_0.append(pos_safety_0[1])

                self.vl53l5_pcl(
                    self.x_pose_safety_0,
                    self.y_pose_safety_0,
                    frame_id="safety_zone_n0",
                )

            if safety["index"] == 1:
                for pos_safety_1 in safety["data"]:
                    self.x_pose_safety_1.append(pos_safety_1[0])
                    self.y_pose_safety_1.append(pos_safety_1[1])
                self.vl53l5_pcl(
                    self.x_pose_safety_1,
                    self.y_pose_safety_1,
                    frame_id="safety_zone_n1",
                )

            if safety["index"] == 2:
                for pos_safety_2 in safety["data"]:
                    self.x_pose_safety_2.append(pos_safety_2[0])
                    self.y_pose_safety_2.append(pos_safety_2[1])
                self.vl53l5_pcl(
                    self.x_pose_safety_2,
                    self.y_pose_safety_2,
                    frame_id="safety_zone_n2",
                )

    def open_json(self):
        """Open Json file."""
        with open(self.config_path, "r") as f:
            data = json.load(f)
        self.df = pd.DataFrame(data)

    def loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.setsafety_pcl()
            self.make_table_markers()
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
            "set_safety_goal.json",
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
    SafetyPointCloud(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
