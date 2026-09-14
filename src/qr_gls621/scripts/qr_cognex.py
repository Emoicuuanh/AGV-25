import socket
import time
from datetime import datetime

# Thông tin camera
ip_address = "192.168.0.150"  # Địa chỉ IP của camera Cognex
port = 2111                     # Cổng mặc định Telnet

# Tạo socket client
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    # Kết nối tới camera
    client_socket.connect((ip_address, port))
    client_socket.settimeout(5)  # Thiết lập timeout 5 giây
    print(f"Kết nối thành công đến camera tại {ip_address}:{port}")

    # Biến lưu thời gian ghi log lần cuối
    last_log_time = 0
    count = 0  # Số lần đọc mã QR
    log_file = "qr_reader_log_cognex.txt"

    while True:
        try:
            # Gửi lệnh để kích hoạt đọc mã QR
            command = "||>TRIGGER ON\r\n"  # Lệnh để kích hoạt đọc mã QR
            client_socket.sendall(command.encode())  # Gửi lệnh
            print(f"Đã gửi lệnh: {command.strip()}")

            # Chờ phản hồi từ camera
            response = client_socket.recv(1024)  # Nhận dữ liệu từ camera
            response_str = response.decode('utf-8').strip()  # Giải mã và làm sạch dữ liệu

            if response_str:
                current_time = time.time()  # Lấy thời gian hiện tại (giây)

                # Chỉ ghi log nếu đã qua ít nhất 2 giây kể từ lần log trước
                if current_time - last_log_time >= 0.5:
                    # Tăng số lần đọc mã QR
                    count += 1

                    # Cập nhật thời gian ghi log
                    last_log_time = current_time

                    # Lấy thời gian hiện tại dạng chuỗi
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                    # Tạo thông điệp log
                    log_message = f"[{timestamp}] Read #{count}: {response_str}"

                    # Ghi log vào file
                    with open(log_file, "a") as log:
                        log.write(log_message + "\n")
                    print(log_message)

            else:
                print("Không có phản hồi hoặc phản hồi trống.")

            # Chờ trước khi lặp lại
            time.sleep(0.01)

        except socket.timeout:
            print("Timeout: Không tìm thấy mã QR. Chờ 1 giây trước khi thử lại.")
            time.sleep(1)  # Đợi 1 giây trước khi thử lại

except Exception as e:
    print(f"Lỗi: {e}")

finally:
    # Ghi dòng phân cách và tổng số lần đọc vào log file
    with open(log_file, "a") as log:
        log.write(f"----------------------------------------------------------------\n")
        log.write(f"Total Reads: {count}\n")
    client_socket.close()
    print("Kết nối đã được đóng.")