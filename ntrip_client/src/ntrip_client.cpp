#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <serial/serial.h>
#include <string>
#include"ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <boost/asio.hpp>


//using namespace boost::asio;

//NTRIP服务器信息
 std::string serverAddress = "203.107.45.154";
 int serverPort = 8002;
 std::string mountPoint = "AUTO";
 std::string username = "qxcorsh00116140";
 std::string password = "8d0353a";
 char* GNGGA_data="$GNGGA,002638.60,3859.13936,N,11720.17556,E,1,12,0.74,17.6,M,-5.1,M,,*6E\r\n";


int init_serialport_recv(serial::Serial& port,int baudrate,char* port_name);
int init_serialport_send(boost::asio::serial_port& port,int baudrate,char* port_send_name);
std::string StringToBase64(const std::string& input) ;
void RecePro(std::string s , double& lat , double& lon, double& high ,int& status);
std::string get_GPS_data(std::string& strRece,serial::Serial& port,sensor_msgs::NavSatFix& GPS_data);
int init_socket(int socket,struct sockaddr_in& server_address,std::string s_serveraddress,int server_port);

ros::Publisher gps_pub;
sensor_msgs::NavSatFix gps_data;

int main(int argc,char** argv) 
{

    //初始化节点
    ros::init(argc, argv, "ntrip_ros");
    //声明节点句柄
    ros::NodeHandle nh;
    //订阅话题
    gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps_data",20);
    //初始化串口
    boost::asio::io_service io;
    boost::asio::serial_port port_send(io);
    init_serialport_send(port_send,115200,"/dev/ttyUSB5");
    serial::Serial port_recv;
    init_serialport_recv(port_recv,115200,"/dev/ttyACM0");

    // 创建套接字
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in serverAddr;
    init_socket(clientSocket,serverAddr,serverAddress,serverPort);
    // 连接到服务器
    if (connect(clientSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
        std::cerr << "Failed to connect to the NTRIP server." << std::endl;
        close(clientSocket);
        return 1;
    }

    std::stringstream requestStream;
    requestStream << "GET /AUTO HTTP/1.0\r\n";//RTCM32_GGB 
    requestStream << "User-Agent: NTRIP ntrip_ros\r\n";
    requestStream << "Accept: */*\r\n";
    requestStream << "Connection: close\r\n";
    requestStream << "Authorization: Basic "<<StringToBase64(username+":"+password)<<"\r\n";
    requestStream << "\r\n";

    std::string request = requestStream.str();
    if (write(clientSocket, request.c_str(), request.length()) == -1) {
        std::cerr << "Failed to send request to the server." << std::endl;
        close(clientSocket);
        return 1;
    }

    const int bufferSize = 1024;
    char buffer2[bufferSize];
    int bytesRead = read(clientSocket, buffer2, bufferSize);
    std::string recv_str=buffer2;
    std::cout<<recv_str<<std::endl;

    std::stringstream gga;
    gga << GNGGA_data;
    std::string s_gga = gga.str();
    write(clientSocket,s_gga.c_str(), s_gga.size());
    char buffer1[bufferSize];
    ros::Rate loop_rate(10);
    int i=0;
    while (ros::ok())  
    {
        i++;
        std::string raw,gga1;
        gga1=get_GPS_data(raw,port_recv,gps_data);
        if(gga1.size()>90)
            continue;
        if(i%10==0)
        {
            int bytesRead = read(clientSocket, buffer1, bufferSize);
            std::string buffer1_s=buffer1;
            write(port_send,boost::asio::buffer(buffer1));
        }
        if(i%2==0)
        gps_pub.publish(gps_data);
        loop_rate.sleep();
    }

    // 清理和关闭连接
    close(clientSocket);
    port_recv.close();
    port_send.close();

    std::cout << "NTRIP data received and sent to serial port." << std::endl;

    return 0;
}

//初始化接收端口
int init_serialport_recv(serial::Serial& port,int baudrate,char* port_recv_name)
  {
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    port.setPort(port_recv_name);
    port.setBaudrate(baudrate);
    port.setTimeout(to);
    port.open();
    if (port.isOpen())
    {
        std::cout<<"串口初始化成功,请连接网络！"<<std::endl;
        return 1;
    }
    else
    {
        std::cout<<"串口未打开"<<std::endl;
        return 0;
    }
  }
  //初始化发送端口
  int init_serialport_send(boost::asio::serial_port& port,int baudrate,char* port_send_name)
  {
    // 设置串口参数
    port.open(port_send_name); // 替换为你的串口设备路径
    port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
    port.set_option(boost::asio::serial_port_base::character_size(8));
    port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));

  }

// Base64编码函数，用于构建Authorization头
std::string StringToBase64(const std::string& input) 
{
    const std::string base64Chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string base64Encoded;
    size_t inputLength = input.length();

    for (size_t i = 0; i < inputLength; i += 3) {
        unsigned char octet1 = input[i];
        unsigned char octet2 = (i + 1 < inputLength) ? input[i + 1] : 0;
        unsigned char octet3 = (i + 2 < inputLength) ? input[i + 2] : 0;

        unsigned char sextet1 = (octet1 & 0xFC) >> 2;
        unsigned char sextet2 = ((octet1 & 0x03) << 4) | ((octet2 & 0xF0) >> 4);
        unsigned char sextet3 = ((octet2 & 0x0F) << 2) | ((octet3 & 0xC0) >> 6);
        unsigned char sextet4 = octet3 & 0x3F;

        base64Encoded += base64Chars[sextet1];
        base64Encoded += base64Chars[sextet2];
        base64Encoded += base64Chars[sextet3];
        base64Encoded += base64Chars[sextet4];
    }

    // Handle padding
    size_t padding = 3 - (inputLength % 3);
    if (padding > 0) {
        base64Encoded.replace(base64Encoded.length() - padding, padding, padding, '=');
    }

    return base64Encoded;
}
//获取GPS数据
std::string get_GPS_data(std::string& strRece,serial::Serial& port,sensor_msgs::NavSatFix& GPS_data)
{
    std::string gga_data;
    if (port.available())
        {
            //1.读取串口信息：
            strRece += port.read(port.available());
            //2.截取数据、解析数据：
            std::string gstart = "$GN";  //GPS起始标志
            std::string gend = "\r\n";   //GPS终止标志
            int i = 0, start = -1, end = -1;
            while ( i < strRece.length() )
            {
                //找起始标志
                start = strRece.find(gstart);
                //如果没找到，丢弃这部分数据，但要留下最后2位,避免遗漏掉起始标志
                if ( start == -1)
                {
                    if (strRece.length() > 2)   
                        strRece = strRece.substr(strRece.length()-3);
                        break;
                }
                //如果找到了起始标志，开始找终止标志
                else
                {
                    //找终止标志
                    end = strRece.find(gend);
                    //如果没找到，把起始标志开始的数据留下，前面的数据丢弃，然后跳出循环
                    if (end == -1)
                    {
                        if (end != 0)
                        strRece = strRece.substr(start);
                        break;
                    }
                    //如果找到了终止标志，把这段有效的数据剪切给解析的函数，剩下的继续开始寻找
                    else
                    {
                        i = end;
                        double secs =ros::Time::now().toSec();
                        //把有效的数据给解析的函数以获取经纬度
                        double lat, lon, high;
                        int status;
                        RecePro(strRece.substr(start,end+2-start),lat,lon,high,status);
                        gga_data=strRece.substr(start,end+2-start);
                        std::cout<<gga_data<<std::endl;
                        //保留位数7位
                        GPS_data.header.frame_id="base_link";
                        GPS_data.header.stamp=ros::Time::now();
                        GPS_data.altitude=high,
                        GPS_data.latitude=lat;
                        GPS_data.longitude=lon;
                        GPS_data.status.status=status;

                        //如果剩下的字符大于等于4，则继续循环寻找有效数据,如果所剩字符小于等于3则跳出循环
                        if ( i+5 < strRece.length())
                            strRece = strRece.substr(end+2);
                        else
                        {   strRece = strRece.substr(end+2);
                            break;
                        }
                    }
                }
            }
        }
        return gga_data;
}

//解析GPS
void RecePro(std::string s , double& lat , double& lon, double& high ,int& status)
{
    //分割有效数据，存入vector中
    std::vector<std::string> v;
    std::string::size_type pos1, pos2;
    pos2 = s.find(",");
    pos1 = 0;
    while ( std::string::npos !=pos2 )
    {
        v.push_back( s.substr( pos1, pos2-pos1 ) );
        pos1 = pos2 + 1;
        pos2 = s.find(",",pos1);
    }
    if ( pos1 != s.length() )
        v.push_back( s.substr( pos1 ));
    //解析出经纬度
    //字段6：GPS状态，0=未定位，1=非差分定位，2=差分定位，3=无效PPS，6=正在估算
    if (v.max_size() >= 6 && (v[6] == "1" || v[6] == "2" || v[6] == "3" || v[6] == "4" || v[6] == "5" || v[6] == "6" || v[6] == "8" || v[6] == "9"))
    {
        //纬度
        if (v[2] != "") lat = std::atof(v[2].c_str()) / 100;
        int ilat = (int)floor(lat) % 100;
        lat = ilat + (lat - ilat) * 100 / 60;
        //经度
        if (v[4] != "") lon = std::atof(v[4].c_str()) / 100;
        int ilon = (int)floor(lon) % 1000;
        lon = ilon + (lon - ilon) * 100 / 60;
        //海拔高度
        if (v[9] != "") high = std::atof(v[9].c_str()) ;
        if(v[6]=="1")
            status=1;
        else if(v[6]=="2")
            status=2;
        else if(v[6]=="3")
            status=3;
        else if(v[6]=="4")
            status=4;
        else if(v[6]=="5")
            status=5;
        else if(v[6]=="6")
            status=6; 
        else 
            status=0;
    }
}
//初始化socket并设置服务器地址
int init_socket(int socket,struct sockaddr_in& server_address,std::string s_serveraddress,int server_port)
{
    if (socket == -1) 
    {
        std::cerr << "Failed to create socket." << std::endl;
        return 1;
    }

    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(server_port);
    if (inet_pton(AF_INET, s_serveraddress.c_str(), &server_address.sin_addr) <= 0) 
    {
        std::cerr << "IP地址无效!" << std::endl;
        close(socket);
        return 1;
    }
}

