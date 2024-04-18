//serial_demo.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    
    //创建一个serial对象
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    
    ros::Rate loop_rate(0.1);
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        // size_t n = sp.available();

        // if(n!=0)
        // {
            uint8_t buffer[12];
            buffer[0]=0xE0;
            buffer[1]=0x03;
            buffer[2]=0x00;
            buffer[3]=0x00;
            buffer[4]=0x00;
            buffer[5]=0x00;
            buffer[6]=0x01;
            buffer[7]=0xF4;
            buffer[8]=0x00;
            buffer[9]=0x00;
            buffer[10]=0x01;
            buffer[11]=0xF4;

            //读出数据
            // n = sp.read(buffer, n);
            
            for(int i=0; i<12; i++)
            {
                //16进制的方式打印到屏幕
                std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }

            std::cout << std::endl;
            //把数据发送回去
            sp.write(buffer, 12);
            sp.read();
        // }
        loop_rate.sleep();
    }
    
    //关闭串口
    sp.close();
 
    return 0;
}