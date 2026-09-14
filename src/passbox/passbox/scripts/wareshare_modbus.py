from pymodbus.client.sync import ModbusTcpClient
import time

# --- CẤU HÌNH THÔNG SỐ ---
SERVER_IP = '192.168.1.200'  # Thay bằng IP thực tế của board
PORT = 502
UNIT_ID = 1  # Thường là 1 cho Waveshare
DELAY = 0.1  # Tốc độ phản hồi (0.1 giây mỗi lần quét)

# --- ĐỊA CHỈ BIT ---
# INPUT WAREHOUSE, OUTPUT AGV (warehouse gửi tín hiệu đến AGV)
OPEN_DIRTY_SIDE = 1
CLOSE_DIRTY_SIDE = 2
PAUSE_DIRTY_SIDE = 3
EMG_WHEN_DIRTY_SIDE = 4
OPEN_CLEAN_SIDE = 5
CLOSE_CLEAN_SIDE = 6
PAUSE_CLEAN_SIDE = 7
EMG_WHEN_CLEAN_SIDE = 8

# OUTPUT WAREHOUSE, INPUT AGV (AGV gửi tín hiệu về warehouse)
DONE_OPEN_DIRTY_SIDE = 2
DONE_CLOSE_DIRTY_SIDE = 1
DONE_OPEN_CLEAN_SIDE = 3
DONE_CLOSE_CLEAN_SIDE = 4
EMG_DIRTY_SIDE = 5
EMG_CLEAN_SIDE = 6


def check_connect(client):
    """
    Kiểm tra và đảm bảo kết nối Modbus TCP.
    
    Args:
        client: ModbusTcpClient object
        
    Returns:
        True nếu kết nối thành công hoặc đã sẵn sàng, False nếu thất bại.
    """
    try:
        # Trong pymodbus sync client, connect() trả về True nếu đã kết nối hoặc kết nối mới thành công
        return client.connect()
    except Exception as e:
        print(f"✗ Lỗi kết nối Modbus: {e}")
        return False


def disconnect(client):
    """
    Đóng kết nối Modbus TCP.
    
    Args:
        client: ModbusTcpClient object
    """
    try:
        if client:
            client.close()
            print("✓ Đã đóng kết nối Modbus.")
    except Exception as e:
        print(f"✗ Lỗi khi đóng kết nối Modbus: {e}")


def read_input_bit(client, bit_address):
    """
    Đọc 1 bit từ discrete input (từ warehouse xuống AGV)
    
    Args:
        client: ModbusTcpClient object
        bit_address: Địa chỉ bit (1-8)
        
    Returns:
        True/False hoặc None nếu lỗi
    """
    try:
        result = client.read_discrete_inputs(0, 8, unit=UNIT_ID)
        
        if result.isError():
            print(f"✗ Lỗi đọc discrete input: {result}")
            return None
        
        # Lấy bit tại vị trí (bit_address - 1) vì địa chỉ bắt đầu từ 1
        bit_value = result.bits[bit_address - 1]
        return bit_value
        
    except Exception as e:
        print(f"✗ Lỗi khi đọc bit {bit_address}: {e}")
        return None


def read_all_input_bits(client):
    """
    Đọc tất cả 8 bit từ discrete input
    
    Args:
        client: ModbusTcpClient object
        
    Returns:
        List 8 giá trị True/False hoặc None nếu lỗi
    """
    try:
        result = client.read_discrete_inputs(0, 8, unit=UNIT_ID)
        
        if result.isError():
            print(f"✗ Lỗi đọc discrete input: {result}")
            return None
        
        return result.bits[:8]
        
    except Exception as e:
        print(f"✗ Lỗi khi đọc input bits: {e}")
        return None


def write_output_bit(client, bit_address, value):
    """
    Ghi 1 bit vào coil (từ AGV lên warehouse)
    
    CÁCH HOẠT ĐỘNG:
    1. Đọc tất cả 8 bit hiện tại từ coil
    2. Sửa bit tại vị trí cần thiết
    3. Ghi lại tất cả 8 bit
    
    Args:
        client: ModbusTcpClient object
        bit_address: Địa chỉ bit (1-8)
        value: True/False hoặc 1/0
        
    Returns:
        True nếu thành công, False nếu lỗi
    """
    try:
        # Bước 1: Đọc tất cả 8 bit hiện tại
        current_bits = read_all_output_bits(client)
        if current_bits is None:
            return False
        
        # Bước 2: Sửa bit tại vị trí cần thiết
        current_bits[bit_address - 1] = bool(value)
        
        # Bước 3: Ghi lại tất cả 8 bit
        result = client.write_coils(address=0, values=current_bits, unit=UNIT_ID)
        
        if result.isError():
            print(f"✗ Lỗi ghi coil tại bit {bit_address}: {result}")
            return False
        
        return True
        
    except Exception as e:
        print(f"✗ Lỗi khi ghi bit {bit_address}: {e}")
        return False


def read_all_output_bits(client):
    """
    Đọc tất cả 8 bit từ coil
    
    Args:
        client: ModbusTcpClient object
        
    Returns:
        List 8 giá trị True/False hoặc None nếu lỗi
    """
    try:
        result = client.read_coils(address=0, count=8, unit=UNIT_ID)
        
        if result.isError():
            print(f"✗ Lỗi đọc coil: {result}")
            return None
        
        return result.bits[:8]
        
    except Exception as e:
        print(f"✗ Lỗi khi đọc output bits: {e}")
        return None


def write_all_output_bits(client, bits):
    """
    Ghi tất cả 8 bit vào coil
    
    Args:
        client: ModbusTcpClient object
        bits: List 8 giá trị True/False hoặc 1/0
        
    Returns:
        True nếu thành công, False nếu lỗi
    """
    try:
        if len(bits) != 8:
            print(f"✗ Cần chính xác 8 bit, nhận được {len(bits)}")
            return False
        
        result = client.write_coils(address=0, values=bits, unit=UNIT_ID)
        
        if result.isError():
            print(f"✗ Lỗi ghi coil: {result}")
            return False
        
        return True
        
    except Exception as e:
        print(f"✗ Lỗi khi ghi output bits: {e}")
        return False


def clear_all_output_bits(client):
    """
    Ghi tất cả 8 bit output về giá trị False (0)
    
    Args:
        client: ModbusTcpClient object
        
    Returns:
        True nếu thành công, False nếu lỗi
    """
    return write_all_output_bits(client, [False] * 8)


def sync_in_out():
    # Khởi tạo kết nối
    client = ModbusTcpClient(SERVER_IP, port=PORT)
    
    print(f"Bắt đầu chương trình đồng bộ tại {SERVER_IP}...")
    print("Nhấn Ctrl+C để dừng chương trình.")
    
    previous_input = [None] * 8
    previous_output = [None] * 8

    while True:
        try:
            if not check_connect(client):
                print("Mất kết nối! Đang thử kết nối lại...")
                time.sleep(2)
                continue
            # input_data = read_input_bit(client, 2)
            # print(input_data)
            write_output_bit(client, 1, 0)
        except KeyboardInterrupt:
            print("\nĐang dừng chương trình...")
            break
        except Exception as e:
            print(f"Lỗi phát sinh: {e}")
            time.sleep(1)
        
        # Chờ một khoảng thời gian ngắn trước khi quét lại
        time.sleep(DELAY)

    client.close()
    print("Đã đóng kết nối.")

if __name__ == "__main__":
    sync_in_out()
