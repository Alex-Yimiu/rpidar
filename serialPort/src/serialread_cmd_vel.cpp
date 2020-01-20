/*****************************************************************************
功能：订阅cmd_vel话题并且解析，通过自定义的通讯协议将ros系统中的信息通过串口（COM1）
            发送给下位单片机控制伺服电机。
日期：2019年11月5日
作者：杜一鸣
团队：新疆大学302实验室只能机器人小组

******************************************************************************/





#include"ros/ros.h"
#include"tf/transform_broadcaster.h"
#include"nav_msgs/Odometry.h"
#include"geometry_msgs/Twist.h"
#include"iostream"
#include"serial/serial.h"
#include"std_msgs/String.h"
#include"std_msgs/Empty.h"

serial::Serial ser;
static u_int8_t data_to_bord[11];
// u_int8_t data_to_bord[4]={1,2,3,4};
static u_int8_t a[1]={0x01};
u_int8_t data_num=0;
u_int8_t send_data[1];

void subCallBack(const geometry_msgs::Twist& cmd_vel)
{
/*
    data_to_bord[0]=0x55;
    data_to_bord[1]=cmd_vel.linear.x;
    data_to_bord[2]=cmd_vel.linear.y;
    data_to_bord[3]=cmd_vel.linear.z;
    data_to_bord[4]=cmd_vel.angular.x;
    data_to_bord[5]=cmd_vel.angular.y;
    data_to_bord[6]=cmd_vel.angular.z;
    data_to_bord[7]=cmd_vel.linear.x+cmd_vel.linear.y+cmd_vel.linear.z+cmd_vel.angular.x+cmd_vel.angular.y+cmd_vel.angular.z;
*/

//数据头
    data_to_bord[0]=0x55;
    if(cmd_vel.linear.x>=0.0)
    {
    //左轮
        data_to_bord[1]=0x00;
        data_to_bord[2]=0x00;
        data_to_bord[3]=0x00+(cmd_vel.linear.x)*10-(cmd_vel.angular.z)*10;
        data_to_bord[4]=0x00;
    //右轮
        data_to_bord[5]=0x00;
        data_to_bord[6]=0x00;
        data_to_bord[7]=0xff-(cmd_vel.linear.x)*10+(cmd_vel.angular.z)*10;
        data_to_bord[8]=0xff;
    }
   if(cmd_vel.linear.x<0.0)
    {
        //左轮
        data_to_bord[1]=0x00;
        data_to_bord[2]=0x00;
        data_to_bord[3]=0xff+(cmd_vel.linear.x)*10-(cmd_vel.angular.z)*10;
        data_to_bord[4]=0xff;
        //右轮
        data_to_bord[5]=0x00;
        data_to_bord[6]=0x00;
        data_to_bord[7]=0x00-(cmd_vel.linear.x)*10+(cmd_vel.angular.z)*10;
        data_to_bord[8]=0x00;
    }
//数据检测

    data_to_bord[9]=0x57;
    
//数据尾
    data_to_bord[10]=0x56;

    //data_to_bord[7]=cmd_vel.linear.x+cmd_vel.linear.y+cmd_vel.linear.z+cmd_vel.angular.x+cmd_vel.angular.y+cmd_vel.angular.z;

	ROS_INFO("Received a /cmd_vel message!");
	ROS_INFO("Linear Components:[%f,%f,%f]",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
	ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"cmd_vel_listener");
    ros::NodeHandle n;
    ros::Subscriber sub=n.subscribe("/turtle1/cmd_vel",1000,subCallBack);
try 
      { 
      //设置串口属性，并打开串口 
      ser.setPort("/dev/ttyS4"); 
      ser.setBaudrate(115200); 
      serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
      ser.setTimeout(to); 
      ser.open(); 
      } 
catch (serial::IOException& e) 
      { 
      ROS_ERROR_STREAM("Unable to open port "); 
      return -1; 
      } 
      //检测串口是否已经打开，并给出提示信息 
      if(ser.isOpen()) 
      { 
      ROS_INFO_STREAM("Serial Port initialized"); 
      } 
      else 
      { 
      return -1; 
      } 
      //指定循环的频率 
      ros::Rate loop_rate(50); 
   while(ros::ok()) 
      { 
    /**************************************************************************************************************************
    说明：用串口给stm32发送串口信息，具体协议自己编写
    问题：数据直接发送到串口会丢帧，具体原因不明，这里简单处理了一下数据，单个帧发送没有丢帧现象。测试时发送两个无丢包现象，
    三个以上丢一半，发一个丢一个的现象。
    ***************************************************************************************************************************/
        send_data[0]=data_to_bord[data_num]; 
        data_num++;
        if(data_num==11){data_num=0;}

      ROS_INFO_STREAM("write to serial port\n"); 
      ser.write(send_data,1);

      ROS_INFO("serial sends: 0x%x 0x%x 0X%x 0X%x",data_to_bord[0], data_to_bord[3],data_to_bord[7],data_to_bord[9]);
      ROS_INFO("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx   : 0x%x ",send_data[0]);
      //处理ROS的信息，比如订阅消息,并调用回调函数 
    ros::spinOnce(); 
    loop_rate.sleep();
      } 
    
      

} 
//       ros::spin();
