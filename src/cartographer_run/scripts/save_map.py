#!/usr/bin/env python
import rospy
import os
import sys
import subprocess
from std_srvs.srv import Empty
from cartographer_ros_msgs.srv import FinishTrajectory, FinishTrajectoryRequest
from cartographer_ros_msgs.srv import WriteState, WriteStateRequest

def main():
    rospy.init_node('cartographer_save_map_node')

    # Kiểm tra đối số đầu vào
    if len(sys.argv) < 2:
        rospy.logerr("Usage: rosrun cartographer_run save_map.py /full/path/to/map_file")
        return

    # Lấy đường dẫn file từ tham số dòng lệnh
    map_file = sys.argv[1]
    rospy.loginfo("Map file path: %s", map_file)

    rospy.loginfo("Waiting for services...")
    rospy.wait_for_service('/finish_trajectory')
    rospy.wait_for_service('/write_state')
    rospy.loginfo("All services available.")

    # 1. Finish trajectory
    try:
        finish_srv = rospy.ServiceProxy('/finish_trajectory', FinishTrajectory)
        req = FinishTrajectoryRequest()
        req.trajectory_id = 0
        rospy.loginfo("Calling /finish_trajectory...")
        finish_srv(req)
        rospy.loginfo("Finished trajectory.")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call /finish_trajectory: %s", e)
        return

    rospy.sleep(1.0)

    # 2. Save pbstream
    try:
        write_srv = rospy.ServiceProxy('/write_state', WriteState)
        req = WriteStateRequest()
        req.filename = map_file + ".pbstream"
        rospy.loginfo(req.filename)
        rospy.loginfo("Calling /write_state to save: %s", req.filename)
        write_srv(req)
        rospy.loginfo("Saved pbstream.")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call /write_state: %s", e)
        return

    rospy.sleep(1.0)

    # 3. Call map_saver
    # rospy.loginfo("Running map_saver...")
    # try:
    #     subprocess.check_call(['rosrun', 'map_server', 'map_saver', '-f', map_file])
    #     rospy.loginfo("Map saved: %s.[pgm|yaml]", map_file)
    # except subprocess.CalledProcessError as e:
    #     rospy.logerr("Failed to run map_saver: %s", e)
    map_saver_cmd = "rosrun map_server map_saver -f {}".format(map_file)
    rospy.loginfo("Executing: %s", map_saver_cmd)
    ret = os.system(map_saver_cmd)
    if ret == 0:
        rospy.loginfo("Map saved successfully.")
    else:
        rospy.logerr("Map saver command failed with code: %d", ret)

if __name__ == '__main__':
    main()
