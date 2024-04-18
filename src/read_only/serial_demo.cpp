#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <serial_control/serial_message.h>

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

int n_max = 3000;
float r = 0.08, PI = 3.1415926, B = 0.704;
class SerialInterface {
 private:
  RecMessage SendMsg;
  RecMessage ReadMsg;

 public:
  SerialInterface() {
    try {
      ser.setPort("/dev/ttyUSB0");
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();

    } catch (serial::PortNotOpenedException &e) {
      ROS_ERROR_STREAM("Port not opened.");  // 如果串口未打开，输出错误信息
    } catch (serial::SerialException &e) {
      ROS_ERROR_STREAM("Serial exception.");  // 如果发生串口异常，输出错误信息
    } catch (serial::IOException &e) {
      ROS_ERROR_STREAM("IO exception.");  // 如果发生IO异常，输出错误信息
    }
    if (ser.isOpen()) {
      ROS_INFO_STREAM("Serial Port initialized");
    }
  }
  ~SerialInterface() {}
  bool DecodeSerial(const uint8_t *buffer) {
    if (buffer[11] == ZERO) {
      switch (buffer[1]) {
        case DETECT_MOTOR_SPPED_ID: {
          this->ReadMsg.motor_speed_msg.speed_left =
              (int16_t)((uint16_t)(buffer[3]) | (uint16_t)(buffer[2]) << 8);
          this->ReadMsg.motor_speed_msg.speed_right =
              (int16_t)((uint16_t)(buffer[5]) | (uint16_t)(buffer[4]) << 8);
        }
        case DETECT_TEMPETRUE_ID: {
          this->ReadMsg.system_state_msg.controller_t = buffer[2];
          this->ReadMsg.system_state_msg.motor_left_t = buffer[3];
          this->ReadMsg.system_state_msg.motor_right_t = buffer[4];
          break;
        }
        default:
          break;
      }
    } else {
      this->ReadMsg.motor_speed_msg.speed_left =
          (int16_t)((uint16_t)(buffer[2]) | (uint16_t)(buffer[1]) << 8);
      this->ReadMsg.motor_speed_msg.speed_right =
          (int16_t)((uint16_t)(buffer[4]) | (uint16_t)(buffer[3]) << 8);
      this->ReadMsg.system_state_msg.state_code_high = buffer[5];
      this->ReadMsg.system_state_msg.state_code_low = buffer[6];
      this->ReadMsg.system_state_msg.error_code_high_left = buffer[7];
      this->ReadMsg.system_state_msg.error_code_low_left = buffer[8];
      this->ReadMsg.system_state_msg.error_code_high_right = buffer[9];
      this->ReadMsg.system_state_msg.error_code_low_right = buffer[10];
      this->ReadMsg.system_state_msg.controller_t = buffer[11];
      ROS_INFO_STREAM("Read: speed_left : "
                      << ReadMsg.motor_speed_msg.speed_left
                      << "RPM; speed_right :"
                      << ReadMsg.motor_speed_msg.speed_right << "RPM;");
    }
    return true;
  }
  bool EncodeSerial(const double linear_velocity,
                    const double angular_velocity) {
    int left_speed, right_speed;
    w_trans_n(linear_velocity, angular_velocity, left_speed, right_speed);
    write_data[0] = 0xE0;
    if (left_speed != 0 && right_speed != 0) {
      write_data[1] = 0b00000011;
    } else if (left_speed != 0) {
      write_data[1] = 0b00000010;
    } else if (right_speed != 0) {
      write_data[1] = 0b00000001;
    } else {
      write_data[1] = 0b00000000;
    }
    write_data[2] = 0x00;
    write_data[3] = 0x00;
    auto abs_compare = [](int a, int b) { return std::abs(a) < std::abs(b); };
    int32_t left_command =
        std::min((left_speed * 10000 / max_speed), 10000, abs_compare);
    if (left_command < 0) {
      left_command = ~(-left_command) + 1;
    }
    write_data[4] = left_command >> 24;
    write_data[5] = (left_command >> 16) & 0xff;
    write_data[6] = (left_command >> 8) & 0xff;
    write_data[7] = left_command & 0xff;
    int32_t right_command =
        std::min((right_speed * 10000 / max_speed), 10000, abs_compare);
    if (right_command < 0) {
      right_command = ~(-right_command) + 1;
    }
    write_data[8] = right_command >> 24;
    write_data[9] = (right_command >> 16) & 0xff;
    write_data[10] = (right_command >> 8) & 0xff;
    write_data[11] = right_command & 0xff;
    // std::cout << std::hex << "hex: left: " << left_command << "; right: " <<
    // right_command << std::endl; std::cout << std::dec << "dec: left: " <<
    // left_command << "; right: " << right_command << std::endl;
    std::cout << "write_data: ";
    for (int i = 0; i < 12; i++) {
      std::cout << std::hex << static_cast<int>(write_data[i]) << " ";
    }
    std::cout << std::endl;
    ser.write(write_data, 12);
    return 1;
  }
  void w_trans_n(double v, double w, int &n_a, int &n_b) {
    Eigen::Matrix2d A;
    Eigen::Vector2d k, m;
    A << 1, B / 2, 1, -B / 2;
    k << v, w;
    m = 3600 * (1 / (2 * PI * r)) * A * k;
    n_a = m[0];
    n_b = m[1];
  }
  void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg) {
    // ROS_INFO("I heard: [%f] , [%f] ", msg->linear.x, msg->angular.z);
    EncodeSerial(msg->linear.x, msg->angular.z);
  }
  serial::Serial ser;
  uint8_t write_data[12];
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_control");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Subscriber sub;

  std::unique_ptr<SerialInterface> SerialPtr(new SerialInterface());
  sub = nh.subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 100,
      boost::bind(&SerialInterface::cmd_vel_callback, SerialPtr.get(), _1));
  ROS_INFO_STREAM("ready to read ");

  while (ros::ok()) {
    if (SerialPtr->ser.available()) {
      uint8_t read_data[12];
      SerialPtr->ser.read(read_data, 12);
      SerialPtr->DecodeSerial(read_data);  // 处理接收到的数据
    }
    ros::spinOnce();
  }
  return 0;
}