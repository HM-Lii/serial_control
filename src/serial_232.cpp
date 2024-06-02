#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int8.h>
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


class SerialInterface {
 private:
  RecMessage SendMsg;
  RecMessage ReadMsg;
  std::string port;
  int baudrate;
  int n_max = 3000,ratio=20;
  double radius_wheel = 0.08, PI = 3.1415926, B = 0.704;

 public:
  SerialInterface(ros::NodeHandle& nh_) {
    try {
      nh_.param("/serial_232/port", port, std::string("/dev/ttyUSB0"));
      nh_.param("/serial_232/baudrate", baudrate, 115200);
      nh_.param("/serial_232/n_max", n_max, 3000);
      nh_.param("/serial_232/ratio", ratio, 20);
      nh_.param("/serial_232/dis_between_bell", B, 0.704);
      nh_.param("/serial_232/wheel_radius", B, 0.08);
      std::cout << "port: " << port << std::endl;
      ser.setPort(port);
      ser.setBaudrate(baudrate);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();

    } catch (serial::PortNotOpenedException &e) {
      ROS_ERROR_STREAM("Port not opened.");
    } catch (serial::SerialException &e) {
      ROS_ERROR_STREAM("Serial exception.");
    } catch (serial::IOException &e) {
      ROS_ERROR_STREAM("IO exception.");
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
    uint8_t write_data[12];
    CmdToMotorSpeed(linear_velocity, angular_velocity, left_speed, right_speed);
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
        std::min((left_speed * 10000 / n_max), 10000, abs_compare);
    EncodeCommand(left_command, &write_data[4]);
    int32_t right_command =
        std::min((right_speed * 10000 / n_max), 10000, abs_compare);
    // std::cout << std::dec << "left_speed: " << left_speed
    //           << " right_speed: " << right_speed << std::endl;
    // std::cout << "left_command: " << left_command
    //           << " right_command: " << right_command;
    EncodeCommand(right_command, &write_data[8]);

    std::cout << "vel_data: ";
    for (int i = 0; i < 12; i++) {
      std::cout << std::hex << static_cast<int>(write_data[i]) << " ";
    }
    std::cout << std::endl;

    ser.write(write_data, 12);
  }

  void EncodeCommand(int32_t command, uint8_t *data) {
    if (command < 0) {
      command = ~(-command) + 1;
    }
    data[0] = command >> 24;
    data[1] = (command >> 16) & 0xff;
    data[2] = (command >> 8) & 0xff;
    data[3] = command & 0xff;
  }

  void CmdToMotorSpeed(double v, double w, int &n_a, int &n_b) {
    Eigen::Matrix2d A;
    Eigen::Vector2d k, m;
    A << 1, B / 2, 1, -B / 2;
    k << v, w;
    m = 60*ratio * (1 / (2 * PI * radius_wheel)) * A * k;
    n_a = m[1];
    n_b = m[0];
  }
  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    // ROS_INFO("I heard: [%f] , [%f] ", msg->linear.x, msg->angular.z);
    EncodeSerial(msg->linear.x, msg->angular.z);
  }
  void CmdWorkCallback(const std_msgs::Int8::ConstPtr &msg) {
      uint8_t write_data[12];      
      write_data[0] = 0xEA;
      write_data[2] = 0x00;
      write_data[3] = 0x00;
      write_data[4] = 0x00;
      write_data[5] = 0x00;
      write_data[6] = 0x00;
      write_data[7] = 0x00;
      write_data[8] = 0x00;
      write_data[9] = 0x00;
      write_data[10] = 0x00;
      write_data[11] = 0x00;
      switch (msg->data)
      {
      case 0:
        write_data[1] = 0x00;
        break;
      case 1:
        write_data[1] = 0x01;
        break;
      case 2:
        write_data[1] = 0x02;
        break;
      case 3:
        write_data[1] = 0x03;
        break;
      case 4:
        write_data[1] = 0x04;
        break;
      default:
        break;
      }
      std::cout << "work_data: ";
      for (int i = 0; i < 12; i++) {
        std::cout << std::hex << static_cast<int>(write_data[i]) << " ";
      }
      std::cout << std::endl;
      ser.write(write_data, 12);
  }
  serial::Serial ser;

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_control");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Subscriber vel_sub,work_sub;

  std::unique_ptr<SerialInterface> SerialPtr(new SerialInterface(nh));
  vel_sub = nh.subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 100,
      boost::bind(&SerialInterface::CmdVelCallback, SerialPtr.get(), _1));
  work_sub = nh.subscribe<std_msgs::Int8>(
      "/cmd_work", 100,
      boost::bind(&SerialInterface::CmdWorkCallback, SerialPtr.get(), _1));
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