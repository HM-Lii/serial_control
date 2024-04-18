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

int n_max;
double r, PI, B;

#define DEBUG

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
  void EncodeSerial(const double linear_velocity,
                    const double angular_velocity) {
    int left_speed, right_speed;
    CmdToMotorSpeed(linear_velocity, angular_velocity, left_speed, right_speed);
    auto abs_compare = [](int a,int b){return std::abs(a) < std::abs(b);};
    int16_t left_command = std::min(left_speed, n_max, abs_compare);
    std::cout<<"left_command: "<<left_command<<"left_speed:"<<left_speed<<"n_max:"<<n_max<<std::endl;
    SendCommand(0x01, left_command);
    int16_t right_command = std::min(right_speed, n_max, abs_compare);
    SendCommand(0x02, right_command);
  }
  void SendCommand(uint8_t motor_id, int16_t command) {
    write_data[0] = motor_id;
    write_data[1] = 0x06;
    write_data[2] = 0x00;
    if (command < 0) {
      write_data[3] = 0x23;
      write_data[4] = 0x00;
      write_data[5] = 0x01;
      uint16_t crc = crc16(write_data, 6);
      write_data[6] = crc & 0xFF;         // CRC低字节
      write_data[7] = (crc >> 8) & 0xFF;  // CRC高字节
      ser.write(write_data, 8);
    } else {
      write_data[3] = 0x23;
      write_data[4] = 0x00;
      write_data[5] = 0x00;
      uint16_t crc = crc16(write_data, 6);
      write_data[6] = crc & 0xFF;         // CRC低字节
      write_data[7] = (crc >> 8) & 0xFF;  // CRC高字节
      ser.write(write_data, 8);
    }
    write_data[3] = 0x06;
    write_data[4] = (command >> 8) & 0xff;
    write_data[5] = command & 0xff;
    uint16_t crc = crc16(write_data, 6);
    write_data[6] = crc & 0xFF;         // CRC低字节
    write_data[7] = (crc >> 8) & 0xFF;  // CRC高字节
#ifdef DEBUG
    std::cout << "write_data: ";
    for (int i = 0; i < 8; i++) {
      std::cout << std::hex << static_cast<int>(write_data[i]) << " ";
    }
    std::cout << std::endl;
#endif
    ser.write(write_data, 8);
  }

  void CmdToMotorSpeed(double v, double w, int &n_a, int &n_b) {
    std::cout<<"v: "<<v<<"w: "<<w;
    Eigen::Matrix2d A;
    Eigen::Vector2d k, m;
    A << 1, B / 2, 1, -B / 2;
    k << v, w;
    m = 3600 * (1 / (2 * PI * r)) * A * k;
    n_a = m[0];
    n_b = m[1];
    std::cout << "n_a: " << n_a << "n_b: " << n_b << std::endl;
  }
  void CmdVelCllback(const geometry_msgs::Twist::ConstPtr &msg) {
    // ROS_INFO("I heard: [%f] , [%f] ", msg->linear.x, msg->angular.z);
    EncodeSerial(msg->linear.x, msg->angular.z);
  }
  uint16_t crc16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
      crc ^= data[i];
      for (uint8_t j = 0; j < 8; j++) {
        if (crc & 1) {
          crc = (crc >> 1) ^ 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }
    return crc;
  }
  serial::Serial ser;
  uint8_t write_data[8];
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_control");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Subscriber sub;
  nh.param("motor_spped_max", n_max, 3000);
  nh.param("wheel_radius", r, 0.08);
  nh.param("dis_between_belt", B, 0.704);
  nh.param("PI", PI, 3.1415926);

  std::unique_ptr<SerialInterface> SerialPtr(new SerialInterface());
  sub = nh.subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 100,
      boost::bind(&SerialInterface::CmdVelCllback, SerialPtr.get(), _1));
  ROS_INFO_STREAM("ready to read ");

  while (ros::ok()) {
    if (SerialPtr->ser.available()) {
      // uint8_t read_data[8];
      // SerialPtr->ser.read(read_data, 8);
      // SerialPtr->DecodeSerial(read_data);  // 处理接收到的数据
    }
    ros::spinOnce();
  }
  return 0;
}