#include "qrIrayble.hpp"
#include <iomanip>
#include <regex>
#include <sstream>

AgvCodeReader::AgvCodeReader(const std::string& server_ip, int port,
                             const std::string& comm_mode,
                             const std::string& result_format,
                             const std::string& transfer_style, int site_id)
    : server_ip_(server_ip), port_(port), socket_(io_service_),
      comm_mode_(comm_mode), result_format_(result_format),
      transfer_style_(transfer_style), site_id_(site_id)
{
  read_buffer_.reserve(1024);
}

bool AgvCodeReader::connect()
{
  try
  {
    boost::asio::ip::tcp::endpoint endpoint(
        boost::asio::ip::address::from_string(server_ip_), port_);
    socket_.connect(endpoint);
    ROS_INFO("Connected to AGV Code Reader at %s:%d", server_ip_.c_str(),
             port_);
    return true;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Failed to connect to AGV Code Reader: %s", e.what());
    return false;
  }
}

void AgvCodeReader::disconnect()
{
  if (socket_.is_open())
  {
    socket_.close();
    ROS_INFO("Disconnected from AGV Code Reader");
  }
}

bool AgvCodeReader::isConnected() { return socket_.is_open(); }

std::vector<uint8_t> AgvCodeReader::createQueryCommand(int address)
{
  std::vector<uint8_t> command(2);

  // Create request message based on site_id/address
  // Format: [Request Byte1] [Request Byte2]
  // For data request: C8 37 (general query)
  // For specific address: 80+address, 7F-address

  if (address >= 0 && address <= 3)
  {
    command[0] = 0x80 + address;
    command[1] = 0x7F - address;
  }
  else
  {
    // General data request
    command[0] = 0xC8;
    command[1] = 0x37;
  }

  return command;
}

std::vector<uint8_t> AgvCodeReader::createLightCommand(int address,
                                                       bool light_on)
{
  std::vector<uint8_t> command(2);

  // Light control commands based on address and on/off state
  uint8_t base_off = 0xF8;
  uint8_t base_on = 0xFC;

  if (address >= 0 && address <= 3)
  {
    if (light_on)
    {
      command[0] = base_on + address;
      command[1] = 0x03 - address;
    }
    else
    {
      command[0] = base_off + address;
      command[1] = 0x07 - address;
    }
  }
  else
  {
    ROS_WARN("Invalid address for light control: %d", address);
    return std::vector<uint8_t>();
  }

  return command;
}

bool AgvCodeReader::sendQuery()
{
  if (!isConnected())
  {
    ROS_WARN("Not connected to AGV Code Reader");
    return false;
  }

  if (result_format_ == "P+F" && transfer_style_ == "Query")
  {
    try
    {
      std::vector<uint8_t> command = createQueryCommand(site_id_);
      boost::asio::write(socket_, boost::asio::buffer(command));
      return true;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("Failed to send query: %s", e.what());
      return false;
    }
  }

  return true;  // For upload mode, no query needed
}

bool AgvCodeReader::sendLightControl(bool light_on)
{
  if (!isConnected())
  {
    ROS_WARN("Not connected to AGV Code Reader");
    return false;
  }

  if (result_format_ == "P+F" && transfer_style_ == "Query")
  {
    try
    {
      std::vector<uint8_t> command = createLightCommand(site_id_, light_on);
      if (command.empty())
        return false;

      boost::asio::write(socket_, boost::asio::buffer(command));

      // Read response (1 byte)
      uint8_t response;
      boost::system::error_code ec;
      boost::asio::read(socket_, boost::asio::buffer(&response, 1), ec);
      if (ec)
      {
        ROS_WARN("Error reading light control response: %s",
                 ec.message().c_str());
        return false;
      }

      ROS_INFO("Light control response: 0x%02X", response);
      return true;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("Failed to send light control: %s", e.what());
      return false;
    }
  }

  return true;
}

std::vector<std::string> AgvCodeReader::readCommonData()
{
  std::vector<std::string> frames;

  if (!isConnected())
  {
    ROS_INFO("No connection");
    return frames;
  }

  boost::system::error_code ec;
  std::size_t available = socket_.available(ec);
  if (ec || available == 0)
  {
    return frames;
  }

  std::vector<char> buf(available);
  std::size_t len = socket_.read_some(boost::asio::buffer(buf), ec);
  if (ec && ec != boost::asio::error::eof)
  {
    ROS_ERROR("Error reading common data: %s", ec.message().c_str());
    return frames;
  }

  std::string data(buf.begin(), buf.begin() + len);
  ROS_INFO("Received raw data: %s", data.c_str());

  // extract all "(...)" frames
  std::regex frame_re(R"(\([^)]*\))");
  auto begin = std::sregex_iterator(data.begin(), data.end(), frame_re);
  auto end = std::sregex_iterator();

  for (auto it = begin; it != end; ++it)
  {
    std::string frame = it->str();
    ROS_INFO("Extracted frame: %s", frame.c_str());
    frames.push_back(frame);  // collect clean frame
  }

  return frames;
}

std::vector<uint8_t> AgvCodeReader::readBinaryData()
{
  if (!isConnected())
    return std::vector<uint8_t>();

  boost::system::error_code ec;

  if (socket_.available() == 0)
  {
    return std::vector<uint8_t>();
  }

  // For P+F format, expect 21 bytes
  std::vector<uint8_t> data(21);
  boost::asio::read(socket_, boost::asio::buffer(data), ec);
  if (ec)
  {
    ROS_WARN("Error reading binary data: %s", ec.message().c_str());
    return std::vector<uint8_t>();
  }

  return data;
}

agv_msgs::DataMatrixStamped
AgvCodeReader::processCommonData(const std::string& input,double angle_cam_qr)
{
  agv_msgs::DataMatrixStamped msg;

  // Strip parentheses
  std::string trimmed = input;
  if (!trimmed.empty() && trimmed.front() == '(')
    trimmed.erase(0, 1);
  if (!trimmed.empty() && trimmed.back() == ')')
    trimmed.pop_back();

  // Split by ';'
  std::vector<std::string> tokens;
  {
    std::stringstream ss(trimmed);
    std::string t;
    while (std::getline(ss, t, ';'))
      tokens.push_back(t);
  }

  if (tokens.size() != 4)
  {
    throw std::runtime_error("Invalid data format! Expected 4 fields, got " +
                             std::to_string(tokens.size()) + ": " + trimmed);
  }

  try
  {
    // 1) possition = grid values + angle
    msg.possition.x = -std::stoi(tokens[0]) / 10;
    msg.possition.y = -std::stoi(tokens[1]) / 10;
    double angle_deg = (-std::stod(tokens[2]) / 10.0)+angle_cam_qr*180/M_PI;
    msg.possition.angle = normalizeAngle(angle_deg);

    double x = msg.possition.x;
    double y = msg.possition.y;
    double theta = msg.possition.angle;

    double x_inv = -x * cos(theta) - y * sin(theta);
    double y_inv = x * sin(theta) - y * cos(theta);
    double theta_inv = -theta;

    // Overwrite with inverted values
    msg.possition.x = static_cast<int>(x_inv);
    msg.possition.y = static_cast<int>(y_inv);
    // msg.possition.angle = theta_inv;

    // 2) parse absolute/lable = from X...Y...
    int abs_x_mm = 0, abs_y_mm = 0;
    const std::string& label = tokens[3];
    size_t xpos = label.find('X');
    size_t ypos = label.find('Y');
    if (xpos == std::string::npos || ypos == std::string::npos ||
        ypos <= xpos + 1)
    {
      throw std::runtime_error("Bad absolute label: " + label);
    }
    std::string xs = label.substr(xpos + 1, ypos - (xpos + 1));
    std::string ys = label.substr(ypos + 1);

    abs_x_mm = std::stoi(xs);
    abs_y_mm = std::stoi(ys);

    // 3) assign directly in mm
    msg.lable.x = abs_x_mm;
    msg.lable.y = abs_y_mm;
    msg.absolute.x = abs_x_mm;
    msg.absolute.y = abs_y_mm;
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(std::string("Error parsing common data: ") +
                             e.what());
  }

  return msg;
}

agv_msgs::DataMatrixStamped
AgvCodeReader::processPepperFuchsData(const std::vector<uint8_t>& input)
{
  agv_msgs::DataMatrixStamped result;
  result.header.stamp = ros::Time::now();

  if (input.size() != 21)
  {
    throw std::invalid_argument("Invalid P+F data size");
  }

  // Parse Pepperl+Fuchs 21-byte format according to manual
  try
  {
    // Check NP bit (No Part detected)
    bool no_code = (input[0] & 0x10) != 0;
    if (no_code)
    {
      // No code detected - return zeros
      result.possition.x = 0.0;
      result.possition.y = 0.0;
      result.possition.angle = 0.0;
      result.lable.x = 0;
      result.lable.y = 0;
      return result;
    }

    // Extract X position (24 bits, bytes 2-5)
    bool x_signed =
        (input[1] & 0x40) != 0;  // TAG bit determines if X is signed
    int32_t x_raw = ((input[2] & 0x07) << 21) | ((input[3] & 0x7F) << 14) |
                    ((input[4] & 0x7F) << 7) | (input[5] & 0x7F);

    if (x_signed && (x_raw >> 23))
    {
      x_raw |= 0xFF000000;  // Sign extend
    }
    result.possition.x =
        x_raw * 0.001;  // Convert to meters (assuming mm resolution)

    // Extract Y position (14 bits, bytes 6-7)
    int16_t y_raw = ((input[6] & 0x7F) << 7) | (input[7] & 0x7F);
    if (y_raw >> 13)
    {
      y_raw |= 0xC000;  // Sign extend 14-bit to 16-bit
    }
    result.possition.y = y_raw * 0.001;  // Convert to meters

    // Extract angle (14 bits, bytes 10-11)
    int64_t angle_raw = (input[10] & 0x7F) << 7 | (input[11] & 0x7F);
    // ROS_INFO("anlge data: %f", angle_raw);
    std::cout << angle_raw << std::endl;
    ROS_INFO("______________________________");
    result.possition.angle = normalizeAngle(angle_raw);

    // Extract tag number (35 bits, bytes 13-17)
    uint64_t tag_num = ((uint64_t)(input[13] & 0x7F) << 28) |
                       ((uint64_t)(input[14] & 0x7F) << 21) |
                       ((uint64_t)(input[15] & 0x7F) << 14) |
                       ((uint64_t)(input[16] & 0x7F) << 7) | (input[17] & 0x7F);

    // Extract grid coordinates from tag number (implementation specific)
    result.lable.x = static_cast<int>(tag_num / 10000);
    result.lable.y = static_cast<int>(tag_num % 10000);

    if (newQr)
    {
      ROS_INFO("P+F Format - Tag: %lu, Pos: (%.3f, %.3f, %.2f°)",
               (unsigned long)tag_num, (double)result.possition.x,
               (double)result.possition.y,
               (double)result.possition.angle * 180.0 / M_PI);
    }

    return result;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error parsing P+F data: %s", e.what());
    throw std::invalid_argument("P+F parsing failed");
  }
}

double AgvCodeReader::normalizeAngle(double angle_degrees)
{
  double angle_radians = angle_degrees * M_PI / 180.0;

  // Normalize angle to range [-pi, pi]
  while (angle_radians > M_PI)
    angle_radians -= 2.0 * M_PI;
  while (angle_radians < -M_PI)
    angle_radians += 2.0 * M_PI;

  return angle_radians;
}

uint8_t AgvCodeReader::calculateParity(uint8_t byte)
{
  uint8_t parity = 0;
  for (int i = 0; i < 7; i++)
  {
    if (byte & (1 << i))
    {
      parity ^= 1;
    }
  }
  return parity << 7;
}