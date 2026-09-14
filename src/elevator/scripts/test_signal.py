from datetime import datetime
import agf_mc_protocol
import rospy

def main():
    rospy.init_node("elevator_server_5111")
    rospy.loginfo("Init node " + rospy.get_name())

    plc = agf_mc_protocol
    plc_address = "192.168.3.251"  # "192.168.20.50"
    port = 504  # 5000

    try:
        plc.plc_connect(plc_address, port)
        while not rospy.is_shutdown():
            try:
                if not plc.check_plc_connected() or plc.plc_connect_fail:
                    isConnected = plc.check_plc_connected()
                    plc.plc_close()
                    rospy.logerr("Connect to PLC error. Retry connect after 1s ...")
                    plc.plc_connect(plc_address, port)
                    rospy.sleep(1)
            except Exception as e:
                rospy.logerr(e)

            if not plc.plc_connect_fail:
                # plc.write_m(1070, [1])
                print(plc.read_m(1000, 1)[0])
            rospy.sleep(0.5)

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down due to user interruption (Ctrl + C).")
    finally:
        plc.plc_close()
        rospy.loginfo("PLC connection closed.")
        rospy.signal_shutdown("Node shutting down.")

if __name__ == "__main__":
    main()
