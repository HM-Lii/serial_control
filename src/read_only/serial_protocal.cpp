#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/TwistStamped.h>
#include <cstdint>
#include <memory>
#include <array>
#include <mutex>
#include <thread>
#include <functional>
#include <Eigen/Dense>

#include <serial_demo/serial_message.h>

int n_max = 3000;
float r = 0.08, PI = 3.1415926, B = 0.704;
class SerialInterface
{
private:
  AgxMessage SendMsg;
  AgxMessage ReadMsg;

public:
  SerialInterface()
  {

    try
    {
      ser.setPort("/dev/ttyUSB0");
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
    }

    catch (serial::PortNotOpenedException &e)
    {
      ROS_ERROR_STREAM("Port not opened."); // 如果串口未打开，输出错误信息
    }
    catch (serial::SerialException &e)
    {
      ROS_ERROR_STREAM("Serial exception."); // 如果发生串口异常，输出错误信息
    }
    catch (serial::IOException &e)
    {
      ROS_ERROR_STREAM("IO exception."); // 如果发生IO异常，输出错误信息
    }
    if (ser.isOpen())
    {
      ROS_INFO_STREAM("Serial Port initialized");
      while (ros::ok())
      {
        if (ser.available())
        {
          uint8_t read_data[12];
          ser.read(read_data, 12);
          DecodeSerial(read_data); // 处理接收到的数据
          ROS_INFO_STREAM("Read: speed_left : " << ReadMsg.motor_speed_msg.speed_left << "RPM; speed_right :" << ReadMsg.motor_speed_msg.speed_right << "RPM;");
        }
        ros::spinOnce();
      }
    }
  }
  ~SerialInterface()
  {
  }
  bool DecodeSerial(const uint8_t *buffer)
  {
    if (buffer[11] == ZERO)
    {
      switch (buffer[1])
      {
      case DETECT_MOTOR_SPPED_ID:
      {
        this->ReadMsg.motor_speed_msg.speed_left =
            (int16_t)((uint16_t)(buffer[3]) |
                      (uint16_t)(buffer[2]) << 8);
        this->ReadMsg.motor_speed_msg.speed_right =
            (int16_t)((uint16_t)(buffer[5]) |
                      (uint16_t)(buffer[4]) << 8);
      }
      case DETECT_TEMPETRUE_ID:
      {
        this->ReadMsg.system_state_msg.controller_t = buffer[2];
        this->ReadMsg.system_state_msg.motor_left_t = buffer[3];
        this->ReadMsg.system_state_msg.motor_right_t = buffer[4];
        break;
      }
      default:
        break;
      }
    }
    else
    {
      this->ReadMsg.motor_speed_msg.speed_left =
          (int16_t)((uint16_t)(buffer[2]) |
                    (uint16_t)(buffer[1]) << 8);
      this->ReadMsg.motor_speed_msg.speed_right =
          (int16_t)((uint16_t)(buffer[4]) |
                    (uint16_t)(buffer[3]) << 8);
      this->ReadMsg.system_state_msg.state_code_high = buffer[5];
      this->ReadMsg.system_state_msg.state_code_low = buffer[6];
      this->ReadMsg.system_state_msg.error_code_high_left = buffer[7];
      this->ReadMsg.system_state_msg.error_code_low_left = buffer[8];
      this->ReadMsg.system_state_msg.error_code_high_right = buffer[9];
      this->ReadMsg.system_state_msg.error_code_low_right = buffer[10];
      this->ReadMsg.system_state_msg.controller_t = buffer[11];
    }
    return true;
  }
  bool EncodeSerial(const double linear_velocity, const double angular_velocity)
  {
    int left_speed, right_speed;
    w_trans_n(linear_velocity, angular_velocity, left_speed, right_speed);
    write_data[0] = 0xE0;
    if (left_speed != 0 && right_speed != 0)
    {
      write_data[1] = 0b00000011;
    }
    else if (left_speed != 0)
    {
      write_data[1] = 0b00000010;
    }
    else if (right_speed != 0)
    {
      write_data[1] = 0b00000001;
    }
    else
    {
      write_data[1] = 0b00000000;
    }
    write_data[2] = 0x00;
    write_data[3] = 0x00;
    write_data[4] = 0x00;
    write_data[5] = 0x00;
    write_data[8] = 0x00;
    write_data[9] = 0x00;
    bool ret = true;
    auto abs_compare = [](int a, int b)
    { return std::abs(a) < std::abs(b); };
    uint16_t left_command = std::min((left_speed / max_speed), 100, abs_compare) * 100;
    write_data[6] = left_command >> 8;
    write_data[7] = left_command & 0xff;
    uint16_t right_command = std::min((right_speed / max_speed), 100, abs_compare) * 100;
    write_data[10] = right_command >> 8;
    write_data[11] = right_command & 0xff;
    ser.write(write_data, 12);
    return ret;
  }
  void w_trans_n(double v, double w, int &n_a, int &n_b)
  {
    Eigen::Matrix2d A;
    Eigen::Vector2d k, m;
    A << 1, B / 2,
        1, -B / 2;
    k << v,
        w;
    m = (1 / (2 * PI * r)) * A * k;
    n_a = m[0];
    n_b = m[1];
  }
  void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    ROS_INFO("I heard: [%f]", msg->linear.x);
    EncodeSerial(msg->linear.x, msg->angular.z);
  }
  serial::Serial ser;
  uint8_t write_data[12];
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "can_conrol");
  ros::NodeHandle nh;
  ros::Subscriber sub;

  ros::Rate loop_rate(10);
  try
  {
    std::unique_ptr<SerialInterface> ptr(new SerialInterface());
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
  }
  sub = nh.subscribe("/cmd_vel", 100, SerialInterface::cmd_vel_callback);
  return 0;
}