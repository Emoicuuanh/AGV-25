#include "qrCamDriver.hpp"

bool qrCamDriver::connect() {
  try {
    boost::asio::ip::tcp::endpoint endpoint(
        boost::asio::ip::address::from_string(server_ip_), port_);
    socket_.connect(endpoint);
    ROS_INFO("Connected to server");
    return true; // Connection successful
  } catch (const std::exception &e) {
    ROS_ERROR("Failed to connect: %s", e.what());
    return false; // Connection failed
  }
}

// std::string qrCamDriver::readData() {
//   static std::string internal_buffer;
//   boost::system::error_code ec;

//   if (socket_.available() == 0) {
//     return "";
//   }

//   char buf[1024];
//   std::size_t len = socket_.read_some(boost::asio::buffer(buf), ec);
//   if (ec) {
//     ROS_WARN("Socket read error: %s", ec.message().c_str());
//     return "";
//   }

//   internal_buffer.append(buf, len);

//   // Regex mẫu: X+00070900Y+00036177,20.94,22.89,347.97
//   std::regex
//   qr_pattern(R"(X\+\d+Y\+\d+,-?\d+\.?\d*,-?\d+\.?\d*,-?\d+\.?\d*)");
//   std::smatch match;

//   if (std::regex_search(internal_buffer, match, qr_pattern)) {
//     std::string qr_data = match.str(0);
//     // Xoá phần đã xử lý
//     internal_buffer = internal_buffer.substr(match.position() +
//     match.length()); return qr_data;
//   }

//   return "";  // chưa đủ dữ liệu
// }

// std::string qrCamDriver::readData() {
//   boost::asio::streambuf buffer;
//   boost::system::error_code ec;

//   // ROS_INFO("Call read data");

//   if (socket_.available() == 0) {
//     return "";
//   }

//   boost::asio::read_until(socket_, buffer, static_cast<char>(3), ec);
//   if (ec) {
//     ROS_WARN("Lỗi khi đọc dữ liệu QR: %s", ec.message().c_str());
//     return "";
//   }

//   std::istream input_stream(&buffer);
//   std::string data;
//   std::getline(input_stream, data);
//   ROS_INFO("Received data: %s", data.c_str());

//   return data;
// }

std::string qrCamDriver::readData() {
  boost::asio::streambuf buffer;
  boost::system::error_code ec;

  if (socket_.available() == 0) {
    // ROS_WARN("socket not available()");
    return "";
  }

  // Đọc cho đến khi gặp ETX (char(3))
  boost::asio::read_until(socket_, buffer, static_cast<char>(3), ec);
  if (ec) {
    ROS_WARN("Lỗi khi đọc dữ liệu QR: %s", ec.message().c_str());
    return "";
  }

  std::istream input_stream(&buffer);
  std::string data;

  // Đọc đến ký tự ETX (ASCII 3)
  std::getline(input_stream, data, static_cast<char>(3));

  ROS_INFO("Received data: %s", data.c_str());

  return data;
}

// std::string qrCamDriver::readData() {
//   boost::system::error_code ec;
//   char buf[1024];

//   std::size_t len = socket_.read_some(boost::asio::buffer(buf), ec);
//   if (ec) {
//     ROS_WARN("Socket read_some error: %s", ec.message().c_str());
//     return "";
//   }

//   std::string data(buf, len);

//   // In từng byte để xem camera gửi gì
//   for (char c : data) {
//     ROS_INFO("BYTE: %d [%c]", static_cast<int>(c), std::isprint(c) ? c :
//     '.');
//   }

//   return data;
// }

void qrCamDriver::sendTrigger() {
  std::string command = "||>TRIGGER ON\r\n";
  boost::asio::write(socket_, boost::asio::buffer(command));
}

agv_msgs::DataMatrixStamped qrCamDriver::processData(std::string input) {
  agv_msgs::DataMatrixStamped result;
  result.header.stamp = ros::Time::now();

  if (input.empty()) {
    ROS_WARN("Dữ liệu đầu vào rỗng!");
    throw std::invalid_argument("Empty input");
  }

  std::vector<std::string> tokens;
  std::istringstream iss(input);
  std::string token;

  while (std::getline(iss, token, ',')) {
    tokens.push_back(token);
  }

  if (tokens.size() != 4) {
    ROS_ERROR("Sai định dạng dữ liệu! Có %ld trường (cần 4): %s", tokens.size(),
              input.c_str());
    throw std::invalid_argument("Malformed data");
  }

  try {
    std::string xy_str = tokens[0];
    size_t x_pos = xy_str.find('X');
    size_t y_pos = xy_str.find('Y');

    if (x_pos == std::string::npos || y_pos == std::string::npos ||
        y_pos <= x_pos) {
      throw std::invalid_argument("Không tìm thấy X/Y trong chuỗi nhãn");
    }

    std::string x_str =
        xy_str.substr(x_pos + 1, y_pos - x_pos - 1); // ví dụ: +0007090
    std::string y_str = xy_str.substr(y_pos + 1);    // ví dụ: +0003617

    result.lable.x = std::stoi(x_str);
    result.lable.y = std::stoi(y_str);
    result.possition.x = std::stod(tokens[1]);
    result.possition.y = std::stod(tokens[2]);
    result.possition.angle = normalizeAngle(std::stod(tokens[3]));

    if (qrCamDriver::newQr) {
      ROS_ERROR("RESULT: [ %d | %d | %.3f | %.3f | %.2f ]", result.lable.x,
                result.lable.y, result.possition.x, result.possition.y,
                result.possition.angle);

      std::cout << "label_x: " << result.lable.x << std::endl;
      std::cout << "label_y: " << result.lable.y << std::endl;
      std::cout << "deviation_origin_x: " << result.possition.x << std::endl;
      std::cout << "deviation_origin_y: " << result.possition.y << std::endl;
      std::cout << "deviation_theta: " << result.possition.angle << std::endl;
      std::cout << "---------------------------------" << std::endl;
    }

    return result;
  } catch (const std::exception &e) {
    ROS_ERROR("Lỗi khi phân tích dữ liệu QR: %s", e.what());
    throw std::invalid_argument("Parsing failed");
  }
}

double qrCamDriver::normalizeAngle(double angle_degrees) {
  double angle_radians = angle_degrees * M_PI / 180.0;

  // Normalize angle to range [-pi, pi] using atan2
  double normalized_angle =
      std::atan2(std::sin(angle_radians), std::cos(angle_radians));

  return normalized_angle;
}
