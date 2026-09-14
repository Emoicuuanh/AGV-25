#!/usr/bin/env python3
import pymcprotocol
import traceback
import rospy
import socket
from time import sleep
pymc3e = pymcprotocol.Type3E(plctype="L")

plc_connect_fail = False
error_code_connect = ""
error_code_when_write_read = ""
def check_plc_connected():
    global plc_connect_fail
    try:
        if pymc3e._is_connected:
            return True
        else:
            return False
    except Exception as e:
        rospy.logerr("error when check_plc_connected plc: {}".format(e))
        plc_connect_fail = True
        return False


def plc_close():
    global plc_connect_fail
    plc_connect_fail = True
    try:
        pymc3e.close()
        return True
    except Exception as e:
        rospy.logerr("error when close plc: {}".format(e))
        return False


def plc_connect(plc_address, port):
    """Connect to PLC

    Args:
        ip (str):       ip address(IPV4) to connect PLC
        port (int):     port number of connect PLC

    """
    global plc_connect_fail, error_code_connect
    
    # # Check with socket if address is valid
    # try:
    #     # Try connecting to the PLC address with a timeout
    #     with socket.create_connection((plc_address, port), timeout=3.0) as sock:
    #         rospy.logdebug("PLC is reachable at {}:{}".format(plc_address, port))
    #         plc_connect_fail = False
    #         sock.close()
    #         sleep(0.5)
    # except socket.timeout:
    #     rospy.logwarn("SOCKET connection to {}:{} timed out after {}s.".format(plc_address, port, 3.0))
    #     plc_connect_fail = True
    #     return False

    # except socket.error as e:
    #     rospy.logerr("SOCKET error at {}:{} — {}".format(plc_address, port, e))
    #     plc_connect_fail = True
    #     return False
    # except Exception as e:
    #     rospy.logerr("Unexpected error when checking SOCKET connection: {}".format(e))
    #     plc_connect_fail = True
    #     return False


    # Connect with pymcprotocol
    try:
        if not check_plc_connected():
            rospy.logwarn(
                "Connect to plc with port:{}, plc_address:{}".format(
                    port, plc_address
                )
            )
            pymc3e.connect(ip=plc_address, port=port)
            plc_connect_fail = False
            error_code_connect = ""
            return True
        else:
            plc_connect_fail = True
            return False
    except socket.timeout:
        rospy.logwarn("PLC connection to {}:{} timed out after {}s.".format(plc_address, port, 3.0))
        plc_connect_fail = True
        return False

    except socket.error as e:
        rospy.logerr("PLC socket error at {}:{} — {}".format(plc_address, port, e))
        plc_connect_fail = True
        return False
    except Exception as e:
        rospy.logerr("Unexpected error when checking PLC connection: {}".format(e))
        plc_connect_fail = True
        return False


def read_x(register, size=1):
    """Read input X

    Args:
        device(int):    Read head device. (ex: "X1")
        size(int):          Number of read device points. default size = 1

    Returns:
        values(list[int]):  bit units value(0 or 1) list

    """
    global plc_connect_fail, error_code_when_write_read
    try:
        error_code_when_write_read = ""
        first_check = pymc3e.batchread_bitunits(
            headdevice="X" + str(register), readsize=size
        )
        if plc_connect_fail:
            rospy.sleep(0.05)
            second_check = pymc3e.batchread_bitunits(
            headdevice="X" + str(register), readsize=size
        )
            if first_check == second_check:
                return first_check
            else:
                rospy.logerr("Error when read_x {}: first_check: {}, second_check: {}".format(register, first_check, second_check))
                return [0]

        return first_check

    except Exception as e:
        error_code_when_write_read = e
        rospy.logerr("error when read_x {} plc: {}".format(register, e))
        plc_connect_fail = True
        rospy.sleep(0.1)
        return [0]


def read_y(register, size=1):
    """Read output Y

    Args:
        device(int):    Read head device. (ex: "Y1")
        size(int):          Number of read device points. default size = 1

    Returns:
        values(list[int]):  bit units value(0 or 1) list

    """
    global plc_connect_fail, error_code_when_write_read
    try:
        error_code_when_write_read = ""
        first_check = pymc3e.batchread_bitunits(
            headdevice="Y" + str(register), readsize=size
        )
        if plc_connect_fail:
            rospy.sleep(0.05)
            second_check = pymc3e.batchread_bitunits(
            headdevice="Y" + str(register), readsize=size
        )
            if first_check == second_check:
                return first_check
            else:
                rospy.logerr("Error when read_y {}: first_check: {}, second_check: {}".format(register, first_check, second_check))
                return [0]

        return first_check
    except Exception as e:
        error_code_when_write_read = e
        rospy.logerr("error when read_y {} plc: {}".format(register, e))
        plc_connect_fail = True
        rospy.sleep(0.1)
        return [0]


def read_m(register, size=1):
    """Read bit M

    Args:
        register(int):    Read head device. (ex: "M1")
        size(int):          Number of read device points. default size = 1

    Returns:
        values(list[int]):  bit units value(0 or 1) list

    """
    global plc_connect_fail, error_code_when_write_read
    try:
        error_code_when_write_read = ""
        first_check = pymc3e.batchread_bitunits(
            headdevice="M" + str(register), readsize=size
        )
        if plc_connect_fail:
            rospy.sleep(0.05)
            second_check = pymc3e.batchread_bitunits(
            headdevice="M" + str(register), readsize=size
        )
            if first_check == second_check:
                return first_check
            else:
                rospy.logerr("Error when read_m {}: first_check: {}, second_check: {}".format(register, first_check, second_check))
                return [0]

        return first_check
    except Exception as e:
        error_code_when_write_read = e
        rospy.logerr("error when read_m {} plc: {}".format(register, e))
        plc_connect_fail = True
        rospy.sleep(0.1)
        return [0]


def read_d(register, size=1):
    """Read in word register.

    Args:
        register(int):    Read head device. (ex: "D1000")
        readsize(int):      Number of read device points. default size = 1

    Returns:
        values(list[int]):  word units value list

    """
    global plc_connect_fail, error_code_when_write_read
    try:
        error_code_when_write_read = ""
        first_check = pymc3e.batchread_wordunits(
            headdevice="D" + str(register), readsize=size
        )
        if plc_connect_fail:
            rospy.sleep(0.05)
            second_check = pymc3e.batchread_wordunits(
            headdevice="D" + str(register), readsize=size
        )
            if first_check == second_check:
                return first_check
            else:
                rospy.logerr("Error when read_d {}: first_check: {}, second_check: {}".format(register, first_check, second_check))
                return [0]

        return first_check
    except Exception as e:
        error_code_when_write_read = e
        rospy.logerr("error when read_d {} plc: {}".format(register, e))
        plc_connect_fail = True
        rospy.sleep(0.1)
        return [0]


def read_w(register, size=1):
    """Read in word register.

    Args:
        register(int):    Read head device. (ex: "W1000")
        readsize(int):      Number of read device points. default size = 1

    Returns:
        values(list[int]):  word units value list

    """
    global plc_connect_fail, error_code_when_write_read
    try:
        error_code_when_write_read = ""
        first_check = pymc3e.batchread_wordunits(
            headdevice="W" + str(register), readsize=size
        )
        if plc_connect_fail:
            rospy.sleep(0.05)
            second_check = pymc3e.batchread_wordunits(
            headdevice="W" + str(register), readsize=size
        )
            if first_check == second_check:
                return first_check
            else:
                rospy.logerr("Error when read_w {}: first_check: {}, second_check: {}".format(register, first_check, second_check))
                return [0]

        return first_check
    except Exception as e:
        error_code_when_write_read = e
        rospy.logerr("error when read_w {} plc: {}".format(register, e))
        plc_connect_fail = True
        rospy.sleep(0.1)
        return [0]


def write_y(register, value):
    """Write output Y

    Args:
        device(int):    Write head device. (ex: "Y10")
        values(list[int]):  Write values. each value must be 0 or 1. 0 is OFF, 1 is ON.

    Returns:
        values(bool):  write success is true

    """
    global plc_connect_fail, error_code_when_write_read
    try:
        pymc3e.batchwrite_bitunits(headdevice="Y" + str(register), values=value)
        error_code_when_write_read = ""
        return True
    except Exception as e:
        error_code_when_write_read = e
        rospy.logerr("error when write_y {} plc: {}".format(register, e))
        plc_connect_fail = True
        rospy.sleep(0.1)
        return [0]


def write_x(register, value):
    """Write output X

    Args:
        device(int):    Write head device. (ex: "X10")
        values(list[int]):  Write values. each value must be 0 or 1. 0 is OFF, 1 is ON.

    Returns:
        values(bool):  write success is true

    """
    global plc_connect_fail, error_code_when_write_read
    try:
        pymc3e.batchwrite_bitunits(headdevice="X" + str(register), values=value)
        error_code_when_write_read = ""
        return True
    except Exception as e:
        error_code_when_write_read = e
        rospy.logerr("error when write_x {} plc: {}".format(register, e))
        plc_connect_fail = True
        rospy.sleep(0.1)
        return [0]


def write_m(register, value):
    """Write bit M

    Args:
        device(int):    Write head device. (ex: "M10")
        values(list[int]):  Write values. each value must be 0 or 1. 0 is OFF, 1 is ON.

    Returns:
        values(bool):  write success is true

    """
    global plc_connect_fail, error_code_when_write_read
    try:
        pymc3e.batchwrite_bitunits(headdevice="M" + str(register), values=value)
        error_code_when_write_read = ""
        return True
    except Exception as e:
        error_code_when_write_read = e
        rospy.logerr("error when write_m {} plc: {}".format(register, e))
        plc_connect_fail = True
        rospy.sleep(0.1)
        return [0]


def write_d(register, value):
    """Write register D

    Args:
        device(int):    Write head device. (ex: "D1000")
        values(list[int]):  Write values.

    Returns:
        values(bool):  write success is true

    """
    global plc_connect_fail, error_code_when_write_read
    try:
        pymc3e.batchwrite_wordunits(
            headdevice="D" + str(register), values=value
        )
        error_code_when_write_read = ""
        return True
    except Exception as e:
        error_code_when_write_read = e
        rospy.logerr("error when write_d {} plc: {}".format(register, e))
        plc_connect_fail = True
        rospy.sleep(0.1)
        return [0]


def write_w(register, value):
    """Write register W

    Args:
        device(int):    Write head device. (ex: "W1000")
        values(list[int]):  Write values.

    Returns:
        values(bool):  write success is true

    """
    global plc_connect_fail, error_code_when_write_read
    try:
        pymc3e.batchwrite_wordunits(
            headdevice="W" + str(register), values=value
        )
        error_code_when_write_read = ""
        return True
    except Exception as e:
        error_code_when_write_read = e
        rospy.logerr("error when write_w {} plc: {}".format(register, e))
        plc_connect_fail = True
        rospy.sleep(0.1)
        return [0]
