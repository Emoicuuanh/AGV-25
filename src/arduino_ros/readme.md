# Arduino_ros

## Phối hợp encoder từ motor driver để loại bỏ nhiễu của encoder arduino

- Mô tả vấn đề:
  - Arduino encoder cập nhật nhanh, đáp ứng đc quá trình di chuyển, tuy nhiên khi rb dừng hẳn thì có xuất hiện nhiễu làm cho phản hồi encoder vẫn nhảy
  - Encoder từ motor driver có độ trễ, tuy nhiên không bị nhiễu
  - Do đó: Trong quá trình di chuyển, sử dụng Encoder Arduino, trong khi robot dừng lại thì kiểm tra encoder từ motor driver xác nhận rb đang dừng và không cập nhật encoder nữa.

- Commit bắt đầu vấn đề này: [1645c2e9](https://gitlab.com/mkac-agv/arduino_ros/-/commit/1645c2e90caad235e5758a162d59550957877bbc)
- Commit hoàn thiện cho vấn đề này là [7690e2f5](https://gitlab.com/mkac-agv/arduino_ros/-/commit/7690e2f5b3be33425867a2eca0869b4ec64be33d)

- Phân tích code dựa theo số dòng tại commit `7690e2f5`

## Chi tiết

- Hàm [encoder_cb](https://gitlab.com/mkac-agv/arduino_ros/-/blob/7690e2f5b3be33425867a2eca0869b4ec64be33d/scripts/arduino_driver.py#L834)

  - Đặt tần số để xét sự dịch chuyển (f=15Hz), nếu tần số này để quá cao thì trong lúc di chuyển dễ bị nhầm với đứng yên, nếu tần số quá thấp thì sẽ phản ứng chậm với sự thay đổi từ di chuyển sang đứng yên. Quá trình thử nghiệm nhận thấy f=15Hz cho kết quả tối ưu.
  - Nếu có sự thay đổi giá trị của encoder từ motor vượt quá ngưỡng --> có di chuyền, ngược lại thì bánh xe đang đứng yên.

- Tại hàm tính odom [odometry_calr](https://gitlab.com/mkac-agv/arduino_ros/-/blob/7690e2f5b3be33425867a2eca0869b4ec64be33d/scripts/arduino_driver.py#L955)

  - Dùng enc_left_raw, enc_right_raw để lấy giá trị encoder từ arduino
  - Nếu đang đứng yên: Tích lũy lỗi (line 963, 969)
  - Nếu đang di chuyển: giá trị encoder trả về bằng giá trị raw trừ đi lỗi tích lũy

- Vì vậy:
  - Phương pháp này sẽ phát hiện được khi nào robot đứng yên, di chuyển.
  - Khi robot đứng yên, nếu encoder từ arduino vẫn có sự thay đổi giá trị thì tích lũy sự thay đổi đó vào lỗi tích lũy, encoder trả về để tính odom vẫn không đổi
  - Khi rb di chuyển thì cần phải trừ đi lỗi tích lũy để đạt đc giá trị đúng
  - Tần số fb của encode từ motor phải lớn hơn hoặc bằng tần số f ở trên.