import serial
import time

serial_waittime = 0.1


def calculate_crc16(data):
    """
    Calculate Modbus CRC16 checksum.
    """
    crc = 0xFFFF  # Initial value
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, byteorder='little')


def convert_hex_to_list(hex_data):
    """Convert hexadecimal data to list of integers."""
    return [int(hex_data[i:i + 2], 16) for i in range(0, len(hex_data), 2)]


def get_area_data(area, port, baudrate=9600, timeout=1):
    # Open serial port
    ser = serial.Serial(port, baudrate, timeout=timeout)
    send_data_list = [0x53, 0x43, 0x68, 0x69, 0x6f, area]  # Example data list, change to your needs
    # Convert data list to bytes
    data = bytes(send_data_list)
    # Calculate CRC16 checksum
    crc_data = calculate_crc16(data)
    # Send data through serial port
    ser.flushInput()
    ser.write(data + crc_data)
    time.sleep(serial_waittime)
    rece_data = ser.read_all()
    ser.close()
    rece_data_list = list(rece_data)
    # Calculate CRC16 checksum and verify data integrity
    crc_data = calculate_crc16(rece_data_list[:-2])
    if list(crc_data) != rece_data_list[-2:]:
        print("CRC16 checksum error!")
        return None
    out_area = rece_data_list[5]
    out1 = rece_data_list[6]
    out2 = rece_data_list[7]
    out3 = rece_data_list[8]
    res = [out_area, out1, out2, out3]
    return res


def get_distance_data(line, port, baudrate=9600, timeout=1):
    # Open serial port
    ser = serial.Serial(port, baudrate, timeout=timeout)
    send_data_list = [0x52, 0x53, 0x63, 0x61, 0x6e, line]
    # Convert data list to bytes
    data = bytes(send_data_list)
    # Calculate CRC16 checksum
    crc_data = calculate_crc16(data)
    # Send data through serial port
    ser.flushInput()
    ser.write(data + crc_data)
    time.sleep(serial_waittime)
    rece_data = ser.read_all()
    ser.close()
    rece_data_list = list(rece_data)
    if rece_data_list[:6] != send_data_list[:6]:
        print("distance data header error")
        return None
    if len(rece_data_list) != 486:
        print("distance data len error")
        return None
    distance = []
    i = 6
    while i < len(rece_data_list):
        high = rece_data_list[i]
        low = rece_data_list[i + 1]
        dis = high * 256 + low
        distance.append(dis)
        i += 2
    return distance


def get_intensities_data(line, port, baudrate=9600, timeout=1):
    # Open serial port
    ser = serial.Serial(port, baudrate, timeout=timeout)
    send_data_list = [0x52, 0x53, 0x74, 0x72, 0x65, line]
    # Convert data list to bytes
    data = bytes(send_data_list)
    # Calculate CRC16 checksum
    crc_data = calculate_crc16(data)
    ser.flushInput()
    ser.write(data + crc_data)
    time.sleep(serial_waittime)
    rece_data = ser.read_all()
    ser.close()
    rece_data_list = list(rece_data)
    if rece_data_list[:6] != send_data_list[:6]:
        print("intensities data header error")
        return None
    if len(rece_data_list) != 486:
        print("intensities data len error")
        return None
    intensities = []
    i = 6
    while i < len(rece_data_list):
        high = rece_data_list[i]
        low = rece_data_list[i + 1]
        inten = high * 256 + low
        intensities.append(inten)
        i += 2
    return intensities


com = 'COM3'
line = 1
area = 0
while True:
    if area > 63:
        area = 0
    if line > 4:
        line = 1
    area_data = get_area_data(area, com, baudrate=115200, timeout=1)
    dist_data = get_distance_data(line, com, baudrate=115200, timeout=1)
    inten_data = get_intensities_data(line, com, baudrate=115200, timeout=1)
    line += 1
    area += 1
    if area_data is not None:
        output = "area:" + str(area_data[0]) + " out1:" + str(area_data[1]) + " out2:" + str(
            area_data[2]) + " out3:" + str([3])
        print(output)
