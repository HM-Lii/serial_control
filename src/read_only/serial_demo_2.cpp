#include <ros/ros.h> // 包含ROS头文件
#include <serial/serial.h> // 包含串口通信库头文件
#include <iostream>
#include <sstream>
#include <Eigen/Dense>
int n_max=3000;
float r=0.08,PI=3.1415926,B=0.704;
serial::Serial ser; // 定义一个串口对象
void write_array(uint8_t* n_hex_array,int n_a,int n_b)
{
    n_hex_array[0] = 0xe0;
    n_hex_array[2] = 0x00;
    if (n_a == 0 && n_b == 0)
    {
        n_hex_array[1] = 0x00;
    }
    else if (n_a == 0 && n_b != 0)
    {
        n_hex_array[1] = 0x02;
    }
    else if (n_a != 0 && n_b == 0)
    {
        n_hex_array[1] = 0x01;
    }
    else if (n_a != 0 && n_b != 0)
    {
        n_hex_array[1] = 0x03;
    }
    std::stringstream ss_a, ss_b;
    n_a=std::min(10000,n_a*10000/n_max);
    n_b=std::min(10000,n_b*10000/n_max);
    ss_a << std::setfill('0') << std::setw(8) << std::hex << n_a; // 将十进制数转换为八位的十六进制数
    std::string n_a_hex = ss_a.str();

    for (int i = 3; i < 7; i++)
    {
        std::string hex_str = n_a_hex.substr(i * 2, 2); // 按两位分隔十六进制数
        n_hex_array[i]= static_cast<uint8_t>(std::stoi(hex_str, nullptr, 16)); // 将十六进制字符串转换为整数并存储在数组中
    }

    ss_b << std::setfill('0') << std::setw(8) << std::hex << n_b; // 将十进制数转换为八位的十六进制数
    std::string n_b_hex = ss_b.str();

    for (int i = 7; i < 11; i++)
    {
        std::string hex_str = n_b_hex.substr(i * 2, 2);                         // 按两位分隔十六进制数
        n_hex_array[i] = static_cast<uint8_t>(std::stoi(hex_str, nullptr, 16)); // 将十六进制字符串转换为整数并存储在数组中
    }
}
void w_trans_n(float v, float w, int &n_a, int &n_b)
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
void R_trans_n(float v, float R, int &n_a, int &n_b)
{
    Eigen::Matrix2d A;
    Eigen::Vector2d k, m;

    A << 1, B / 2,
         1, -B / 2;
    k << v,
         v/R;
    m = (1 / (2 * PI * r)) * A * k;
    n_a = m[0];
    n_b = m[1];
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_node"); // 初始化ROS节点
    ros::NodeHandle nh; // 定义节点句柄 
    int v=2.2,w=0.5,R=3,n_a,n_b;
    try {
        ser.setPort("/dev/ttyUSB0"); // 设置串口端口
        ser.setBaudrate(115200); // 设置波特率
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 设置超时时间
        ser.setTimeout(to);
        ser.open(); // 打开串口
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port."); // 如果无法打开串口，输出错误信息
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial port initialized."); // 如果串口打开成功，输出初始化成功信息
    } else {
        return -1;
    }
    std::string result;
    std::stringstream ss(result);
    std::string read_msg;
    uint8_t write_msg[12];
    int read_array[12];      // 定义一个十进制数组，用于存储转换后的数据
    ros::Rate loop_rate(1); // 设置循环频率
    int times;
    while (times<10&&ros::ok()) {
        if (ser.available()) { // 如果串口有可用数据
            
            try {
                result = ser.read(ser.available()); // 读取串口数据
            } catch (serial::PortNotOpenedException& e) {
                ROS_ERROR_STREAM("Port not opened."); // 如果串口未打开，输出错误信息
            } catch (serial::SerialException& e) {
                ROS_ERROR_STREAM("Serial exception."); // 如果发生串口异常，输出错误信息
            }
            catch (serial::IOException &e)
            {
                ROS_ERROR_STREAM("IO exception."); // 如果发生IO异常，输出错误信息
            }
            ROS_ERROR_STREAM("Port get message.");
            int i = 0;
            while (std::getline(ss, read_msg, ' '))
            {                                                    // 将接收到的16进制消息按位存放在数组中
                read_array[i] = std::stoi(read_msg, nullptr, 16); // 将16进制转换为10进制并存储在数组中
                i++;
            }
        }
        ROS_ERROR_STREAM("Start writing.");
        R_trans_n(v,r,n_a,n_b);//把运动状态量通过运动学模型转换成两侧电机转速
        ROS_ERROR_STREAM(" motor speed transform finished.");
        write_array(write_msg, n_a, n_b);
        ROS_ERROR_STREAM("message transform finished.");
        std::cout<<write_msg<<std::endl;
        try
        {
            ser.write(write_msg, 12); // 通过串口发送转换后的十六进制消息
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

        times++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    ser.close();
	return 0;
}
