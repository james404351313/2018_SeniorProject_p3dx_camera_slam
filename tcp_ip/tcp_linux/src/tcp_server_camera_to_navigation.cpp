#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "ros/ros.h"
#include "tcp_linux/tcpserver.h"
#include "geometry_msgs/PoseStamped.h"
 #define PortNum 8845

char inputBuffer[32]={};
float path_x,path_y,goal_theta;
bool inputBuffer_to_data_bool=false;

geometry_msgs::PoseStamped goal;

void inputBuffer_to_data(const char* inputBuffer)
{
  double data_Buffer=0;
  int data_Cout=0;
  int minus_cout=0;
  char data_type[16]="c";
  if(inputBuffer[0]==data_type[0])
  {
     for(int i=2 ; i<=32 ; i++)
     {
        /*讀取資料*/
        if(inputBuffer[i]!=',' && inputBuffer[i]!='*' && inputBuffer[i]!='-')
        {
           data_Buffer=data_Buffer*10+(int)inputBuffer[i]-'0';
        }
        /*判斷正負*/
        else if(inputBuffer[i]=='-')
        {
          minus_cout++;
        }
        /*取出資料*/
        else if(inputBuffer[i]==',')
        {
           data_Cout++;
           if(minus_cout==0)
           {
               if(data_Cout==1)
               {
                 path_x=data_Buffer/10;
                 data_Buffer=0;
               }
               else if(data_Cout==2)
               {
                 path_y=data_Buffer/10;
                 data_Buffer=0;
               }
               else if(data_Cout==3)
               {
                 goal_theta=data_Buffer/10;
                 data_Buffer=0;
               }
           }
           else if(minus_cout==1)
           {
               if(data_Cout==1)
               {
                 path_x=-data_Buffer/10;
                 data_Buffer=0;
                 minus_cout=0;
               }
               else if(data_Cout==2)
               {
                 path_y=-data_Buffer/10;
                 data_Buffer=0;
                 minus_cout=0;
               }
               else if(data_Cout==3)
               {
                 goal_theta=-data_Buffer/10;
                 data_Buffer=0;
                 minus_cout=0;
               }
           }
           else
           {
             ROS_INFO("minus error");
           }
        }
        /*讀取結束*/
        else if(inputBuffer[i]=='*')
       {
         data_Cout=0;
         data_Buffer=0;
         minus_cout=0;
         ROS_INFO("receve position");
         ROS_INFO("goal_x=%f",path_x);
         ROS_INFO("goal_y=%f",path_y);
         ROS_INFO("goal_theta=%f",goal_theta);
         inputBuffer_to_data_bool=true;
         break;
       }
        /*讀取錯誤*/
        else
        {
          ROS_INFO("data error");
          break;
        }
     }
  }
}

void data_to_goal()
{
  goal.header.frame_id="map";
  goal.header.stamp=ros::Time::now();
  goal.pose.position.x=path_x;
  goal.pose.position.y=path_y;
  goal.pose.position.z=0;
  goal.pose.orientation.x=0;
  goal.pose.orientation.y=0;
  goal.pose.orientation.z=sin(goal_theta*M_PI/360);
  goal.pose.orientation.w=cos(goal_theta*M_PI/360);
}

int main(int argc, char ** argv)
{
 ros::init(argc, argv, "tcp_server_camera_to_navigation");
 ros::NodeHandle nh;
 ros::Publisher goal_publisher=nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);

 int sockfd , SocketClient,n ;
 struct sockaddr_in serverInfo , clientInfo;
 socklen_t len;
 sockfd = socket(AF_INET, SOCK_STREAM, 0); // AF_INET:Interent Address
 serverInfo.sin_family = AF_INET;
 serverInfo.sin_port = htons(8787);
 serverInfo.sin_addr.s_addr = inet_addr("127.0.0.1");
 bind(sockfd, (struct sockaddr*)&serverInfo, sizeof(serverInfo));
 printf("binding...\n");
 listen(sockfd, 5);
 printf("listening...\n");
 printf("wait for connection...\n");
 len = sizeof(clientInfo);
 SocketClient = accept(sockfd, (struct sockaddr *)&clientInfo, &len);
 printf("Receive connection\n");
 while(ros::ok())
{
   memset(inputBuffer, 0, sizeof(inputBuffer));
   n = read(SocketClient, inputBuffer, sizeof(inputBuffer));
   inputBuffer_to_data(inputBuffer);
   if(inputBuffer_to_data_bool)
   {
     data_to_goal();
     goal_publisher.publish(goal);
     inputBuffer_to_data_bool=false;
   }
   ros::spinOnce();
}
 close(sockfd);
 return 0;
}
