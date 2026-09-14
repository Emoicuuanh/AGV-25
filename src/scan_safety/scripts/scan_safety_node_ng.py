#!/usr/bin/env python3
#!
import sys
import rospy
import rospkg
from sensor_msgs.msg import LaserScan
from math import cos, sin, sqrt
from math import pi as PI
from polar_utils import *
from safety_msgs.msg import SafetyStatus
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
# from scan_safety.cfg import SafetyConfig, Job0Config #, Job2Config, Job3Config, Job4Config
from scan_safety.cfg import SafetyNgConfig, SafetyConfig
from std_msgs.msg import Time, Int16
import threading
import copy
import os
import json
from std_stamped_msgs.msg  import StringStamped, StringFeedback, StringResult, StringAction
import actionlib
from sensor_msgs.msg import PointCloud2
from shapely.geometry import Point, LineString, Polygon
from scan_safety.srv import SafetyEnable, SafetyEnableResponse
from rospy.exceptions import ROSException
from safety_msgs.msg import SafetyJob
import geometry_msgs.msg
# TODO: Update footprint to cover leg of cart (ignore obstacles in footprint)
common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'scripts')
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'release')
sys.path.insert(0, common_func_dir)

from common_function import (
    EnumString,
    lockup_pose,
    offset_pose_x,
    distance_two_pose,
    delta_angle,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_warn,
    print_error,
    print_info,
    get_line_info
)

class ScanSafetyNode():
    _feedback = StringFeedback()
    _result = StringResult()

    def __init__(self):
        logLevel = 2
        print("Args: {}".format(sys.argv))
        if len(sys.argv)>3:
            # for arg in range(len(sys.argv)):
            #     print("arg {} : {}".format(arg,sys.argv[arg]))

            if int(sys.argv[1])>0 and int(sys.argv[1]):
                logLevel = int(sys.argv[1])
        # Safety Job message
        self.safety_job_msg = SafetyJob()
        rospy.init_node("scan_safety_node_ng", log_level=logLevel)
        rospy.loginfo("Init node %s with log_level = %i"%(rospy.get_name(), logLevel))

        # Load default setting
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        # print_debug("current dir: " + self.dir_path)
        f = open(self.dir_path + "/default.json")
        self.defaultData = json.load(f)
        f.close()
        self.job = {} # self.job = {"safety":[{"index":0,"data":[[,],[]...]},{"index":1,"data":[[,],[]...]}....]
        self.job["safety"] = self.defaultData["safety"]
        self.footprint={} # self.footprint = {"footprint":[[,],[],...]}
        self.footprint["footprint"] = self.defaultData["footprint"]

        # Get params
        # print_debug("Default job: {}".format(json.dumps(self.job, indent=2)))
        self.time_delay_scan = rospy.get_param("~time_delay_scan", 0.15)
        self.slice_size = rospy.get_param("~slice_size", 5)
        self.min_ray = rospy.get_param("~min_ray", 3)
        self.move_base_node_name = rospy.get_param("~move_base_node_name", "move_base")

        # time to check
        self.check_laser_time = rospy.get_time()
        self.enable_safety_state = False
        rospy.Subscriber("/safety_job", StringStamped, self.receiveJobCb)
        rospy.Subscriber("/safety_footprint", StringStamped, self.receiveFootprintCb)
        rospy.Subscriber("/safety_set_default", StringStamped, self.receivedSetDefault)

        # Publisher
        self.cfgChangedPub = rospy.Publisher("/scan_safety_cfg_changed_ng", Time, queue_size=10)
        self.scanSafetyStatusPub = rospy.Publisher("/scan_safety_status", SafetyStatus, queue_size=10)
        self.safetyStatusPub = rospy.Publisher("/safety_status", SafetyStatus, queue_size=10)
        self.safetyJobPub = rospy.Publisher("/safety_job_data", SafetyJob, queue_size=10)
        # Service
        srv = Server(SafetyNgConfig, self.reconfigureCB)
        # enable_srv = rospy.Service('enable_safety', SafetyEnable, self.enable_safety_cb)

        self._ac_set_footprint = actionlib.SimpleActionServer('/set_footprint', StringAction, execute_cb=self.set_footprint_cb, auto_start=False)
        self._ac_set_footprint.start()
        self._ac_set_safety_job = actionlib.SimpleActionServer('/set_safety_job', StringAction, execute_cb=self.set_safety_job_cb, auto_start=False)
        self._ac_set_safety_job.start()
        self.updateFields(self.job)
        self.updateFootprint(self.footprint)
        self.checkObstacleSizeConfig()
        self.obstacle = [0]*self.number_of_fields
        self.obstacle.append(111)
        self.slice_arr = [[]]*self.number_of_fields


        rospy.logdebug("Main thread: %s"%threading.current_thread())
        while not rospy.is_shutdown():
            # self.pubSafetyStatus(self.obstacle)
            # self.safetyJobPublish()
            rospy.sleep(2.0)

    def get_max_range_x(self, data_array):
        max = 0.0
        for i in data_array:
            if i[0] > max:
                max = i[0]
        print_info("max: {}".format(max))
        return max

    def get_min_range_x(self, data_array):
        min = 0.0
        for i in data_array:
            if i[0] < min:
                min = i[0]
        print_info("min: {}".format(min))
        return min

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def receiveJobCb(self, msg):
        rospy.loginfo("Set safety job")
        # print(msg.data)
        self.job = json.loads(msg.data)
        # print(self.job)
        rospy.loginfo(json.dumps(self.job))
        rospy.set_param("~job", json.dumps(self.job))

        # print(msgJson)
        self.updateFields(self.job)

    def receiveFootprintCb(self, msg):
        rospy.loginfo("Set footprint")
        # print(msg.data)
        self.footprint = json.loads(msg.data)
        rospy.loginfo(json.dumps(self.footprint))
        rospy.set_param("~footprint", json.dumps(self.footprint))
        self.updateFootprint(self.footprint)

    def receivedSetDefault(self, msg):
        if msg.data == "save":
            self.defaultData.clear()
            self.defaultData.update(self.job)
            self.defaultData.update(self.footprint)
            f = open(self.dir_path + "/default.json", 'w')
            json.dump(self.defaultData, f, indent=4)
            f.close()
            rospy.loginfo("Safety data was set as default!")

    def reconfigureCB(self, config, level):
        rospy.loginfo("Update from scan reconfigure")
        rospy.logdebug("reconfig CB thread: %s"%threading.current_thread())

        jobStr = json.dumps(self.job)
        if config.job == "":
            config.job = jobStr
            job_dict = json.dumps(self.job, indent=2)
            if "params" in job_dict:
                print_warn("Bug - Reconfigure job = Empty:\n{}".format(json.dumps(self.job, indent=2)))
                self.updateFields(json.loads(job_dict)["params"])
            else:
                self.updateFields(self.job)
        if config.job != jobStr:
            self.job = json.loads(config.job)
            job_dict = json.dumps(self.job, indent=2)
            if "params" in job_dict:
                print_warn("Bug - Reconfigure job change:\n{}".format(json.dumps(self.job, indent=2)))
                self.updateFields(json.loads(job_dict)["params"])
            else:
                self.updateFields(self.job)
        # rospy.set_param("~")
        self.slice_size = config.slice_size
        self.min_ray= config.min_ray
        self.checkObstacleSizeConfig()

        if config.footprint == "":
            footprintStr =  json.dumps(self.footprint)
            config.footprint = footprintStr
            self.updateFootprint(self.footprint)
        else:
            footprintStr = '{"footprint": [[0.6, 0.5], [-0.6, 0.5], [-0.6, -0.5], [0.6, -0.5], [0.6, 0.5]]}'
            # footprintStr = config.footprint
            footprintJs = json.loads(footprintStr)
            self.updateFootprint(footprintJs)
        rospy.set_param("~footprint", config.footprint)

        return config

    def checkObstacleSizeConfig(self):
        if self.min_ray > self.slice_size:
            rospy.logerr("Condition fail. min_ray bigger than slice_size")

    def updateFields(self, jsonInput):
        """ Called when init node or change job, change field
            Calculate some factors and append into self.Fields
            self.Fields = [field1[zone1[p, phi, theta_P, theta_Q, k, i],...], field2[zone1..
        """
        # print_debug("Json input:\n{}".format(json.dumps(jsonInput, indent=2)))
        # self.number_of_fields = rospy.get_param("~job_%i/number_of_fields"%self.current_job, 3)

        self.number_of_fields = len(jsonInput["safety"])
        rospy.loginfo(jsonInput["safety"])
        N_fields = self.number_of_fields
        rospy.loginfo("Number of fields: %i"%N_fields)
        self.slice_arr = [[]]*N_fields

        self.Vertexes = []  # [vertexs1[point1[x,y], ...],[vertexs2[... ]
        self.Fields = []     # [field1[zone1[p, phi, theta_P, theta_Q, k, i],...], field2[zone1..]
        self.safety_job_msg.jobs = []
        try:
            # Duyet qua cac vung an toan (vung 1, vung 2, vung 3)
            for i in range(N_fields):
                # param_str = rospy.get_param("~job_%i/field_%i"%(self.current_job, i+1), '')
                # vertexes = str2arr(param_str)
                vertexes = jsonInput["safety"][i]["data"]
                # rospy.loginfo(vertexes)
                self.Fields.append(Polygon(vertexes))
                rospy.logdebug("Fields {}: {}".format(i, self.Fields[i]))
                self.safety_job_msg.jobs.append(self.dict_to_polygon(vertexes))

            self.emitCfgChanged()
            # publish safety Job
            self.safetyJobPublish()

        except Exception as e:
            rospy.logerr(e)
        # # Test fields
        # point_test = []
        # point_test.append(Point(0.8, 0.0))
        # point_test.append(Point(1.0, 0.0))
        # point_test.append(Point(1.5, 0.0))
        # point_test.append(Point(2.0, 0.0))
        # for i in range(len(point_test)):
        #     for j in range(len(self.Fields)):
        #         # check tung field
        #         print("Point {} in Field {} : {}".format(i, j, self.Fields[j].contains(point_test[i])))

    def updateFootprint(self, jsonInput):
        FootprintEdges = []
        Footprint = jsonInput["footprint"]
        self.fp_polygon = Polygon(Footprint)
        self.safety_job_msg.footprint = self.dict_to_polygon(Footprint)
        # publish safety Job
        self.safetyJobPublish()

    def safetyJobPublish(self):
        self.safety_job_msg.header.frame_id = "base_link"
        self.safety_job_msg.header.stamp = rospy.Time.now()
        self.safetyJobPub.publish(self.safety_job_msg)

    def dict_to_polygon(self, point_list):
        rospy.logdebug("Point list: {}".format(point_list))
        polygon_msg = geometry_msgs.msg.Polygon()
        for point in point_list:
            polygon_msg.points.append(geometry_msgs.msg.Point32(point[0], point[1], 0.0))
        return polygon_msg

    def emitCfgChanged(self):
        msg = Time()
        msg.data = rospy.Time.now()
        self.cfgChangedPub.publish(msg)


    def pubSafetyStatus(self, obstacle):
        msg = SafetyStatus()
        msg.job = 0
        msg.fields = copy.deepcopy(obstacle)
        msg.header.stamp = rospy.Time.now()
        self.scanSafetyStatusPub.publish(msg)
        if self.use_ultrasonic == False:
            self.safetyStatusPub.publish(msg)
        else:
            from ultra_sonic.msg import UltrasonicRanges
            # Merge scan vs ultrasonic
            if self.ultrasonic_status == None: return
            if len(msg.fields) != len(self.ultrasonic_status.fields): return
            # Check ultrasonic disconnect
            if rospy.Time.now() - self.ultrasonic_status.header.stamp < rospy.Duration(0.5):
                for i in range(len(msg.fields)):
                    msg.fields[i] = 1 if self.ultrasonic_status.fields[i] == 1 or msg.fields[i] == 1 else 0
            self.safetyStatusPub.publish(msg)

    def send_feedback(self, action, msg):
        self._feedback.data = msg
        action.publish_feedback(self._feedback)

    def set_safety_job_cb(self, goal):
        rospy.loginfo("Set safety job received:\n{}".format(goal.data))


        try:
            goal_dict = json.loads(goal.data)
            msg = StringStamped()
            msg.data = json.dumps(goal_dict["params"], indent=2)
            self.send_feedback(self._ac_set_safety_job, "ACTIVE")
        except:
            rospy.logerr("Goal input error")
            self._ac_set_safety_job.set_aborted(text="Goal input error")
            return

        try:
            self.receiveJobCb(msg)
            # if self.use_ultrasonic:
            #     self.ultrasonic_config()
            self._ac_set_safety_job.set_succeeded()
        except:
            rospy.logerr("Execute error")
            self._ac_set_safety_job.set_aborted(text="Execute error")

    def set_footprint_cb(self, goal):
        rospy.loginfo("Set footprint received:\n{}".format(goal.data))

        try:
            goal_dict = json.loads(goal.data)
            msg = StringStamped()
            len_of_points = len(goal_dict["params"]["footprint"])
            if len_of_points >= 2:
                first_point = goal_dict["params"]["footprint"][0]
                last_point = goal_dict["params"]["footprint"][len_of_points-1]
                if first_point[0] != last_point[0] or first_point[1] != last_point[1]:
                    rospy.logwarn("Set footprint: auto add last point same first point")
                    goal_dict["params"]["footprint"].append(first_point)
                footprint_arr = str(goal_dict["params"]["footprint"])
                rospy.loginfo("{} footprint: {}".format(self.move_base_node_name,footprint_arr))
                param_dict = {"footprint": footprint_arr}
                # # Set cosmap for move base
                # client = Client(self.move_base_node_name + "/global_costmap/", timeout=1)
                # client.update_configuration(param_dict)
                # client = Client(self.move_base_node_name + "/local_costmap/", timeout=1)
                # client.update_configuration(param_dict)

            msg.data = json.dumps(goal_dict["params"], indent=2)
            self.send_feedback(self._ac_set_footprint, "ACTIVE")
        except:
            self._ac_set_footprint.set_aborted(text="Goal input error")
            return

        try:
            self.receiveFootprintCb(msg)
            self._ac_set_footprint.set_succeeded()
        except:
            self._ac_set_footprint.set_aborted(text="Execute error")

if __name__ == "__main__":
    scan_safety = ScanSafetyNode()
