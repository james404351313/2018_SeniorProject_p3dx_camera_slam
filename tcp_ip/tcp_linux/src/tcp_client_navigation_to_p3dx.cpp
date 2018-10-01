#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

char inputBuffer[32]={};
char message[2048]={};

char goal_x_char[32]={};
char goal_y_char[32]={};
char goal_theta_char[32]={};
int goal_x , goal_y ;

bool pose_to_message_bool=false;
bool global_plan_to_message_bool=false;

int  goal_cout=0;



/*test*/
int now_path_x , now_path_y , last_path_x , last_path_y ;
double distance;

 void pose_to_message(const geometry_msgs::Pose2DConstPtr& pose2D)
{
  int  x, y, theta;
  char x_char[32]={} , y_char[32]={} , theta_char[32]={} ;

  x=1000*pose2D->x;
  y=1000*pose2D->y;
  theta=10*pose2D->theta;

  last_path_x=x;
  last_path_y=y;

  sprintf(x_char,"%d",x);
  sprintf(y_char,"%d",y);
  sprintf(theta_char,"%d",theta);

  strcpy(message,"a,");
  strcat(message,x_char);
  strcat(message,",");
  strcat(message,y_char);
  strcat(message,",");
  strcat(message,theta_char);
  strcat(message,",");
  strcat(message,"*");
  ROS_INFO("%s",message);
  pose_to_message_bool=true;
}

 void NavfnROS_path_to_message(const nav_msgs::PathConstPtr& NavfnROS_path)
 {
   int  path_x, path_y;
   char path_x_char[32]={} , path_y_char[32]={} , size_char[32]={};
   int cout=0;
   bool distance_bool=true;

   if(goal_cout==0)
   {
       int size=NavfnROS_path->poses.size();
       //ROS_INFO("size:%d",size);

       for(int i=0 ; i<size ; i++)
       {
         path_x=1000*NavfnROS_path->poses[i].pose.position.x;
         path_y=1000*NavfnROS_path->poses[i].pose.position.y;


         if(cout<size-1)
         {
           now_path_x=1000*NavfnROS_path->poses[i].pose.position.x;
           now_path_y=1000*NavfnROS_path->poses[i].pose.position.y;
           distance=pow(pow(now_path_x-last_path_x,2)+pow(now_path_y-last_path_y,2),0.5);
           last_path_x=now_path_x;
           last_path_y=now_path_y;
         }

         else if(cout==size-1)
         {
           now_path_x=(goal_x-now_path_x)/2+now_path_x;
           now_path_y=(goal_y-now_path_y)/2+now_path_y;
           distance=pow(pow(now_path_x-last_path_x,2)+pow(now_path_y-last_path_y,2),0.5);
           last_path_x=now_path_x;
           last_path_y=now_path_y;
           ROS_INFO("distance:%f",distance);

           distance=pow(pow(goal_x-last_path_x,2)+pow(goal_y-last_path_y,2),0.5);
           ROS_INFO("distance:%f",distance);
           ROS_INFO("last_path_x:%d",last_path_x);
           ROS_INFO("last_path_y:%d",last_path_y);

           if(distance<18.0)
           {
             last_path_x=1000*NavfnROS_path->poses[size-2].pose.position.x;
             last_path_y=1000*NavfnROS_path->poses[size-2].pose.position.y;
             ROS_INFO("distance<18.0");
             ROS_INFO("last_path_x:%d",last_path_x);
             ROS_INFO("last_path_y:%d",last_path_y);
             distance=pow(pow(goal_x-last_path_x,2)+pow(goal_y-last_path_y,2),0.5);
             ROS_INFO("distance:%f",distance);
             distance_bool=false;
           }
         }


         if(cout==0)
         {
           strcpy(message,"b,");
           sprintf(size_char,"%d",size+1);
           strcat(message,size_char);
           strcat(message,",");
         }
         else if(cout<size-1)
         {
           sprintf(path_x_char,"%d",path_x);
           sprintf(path_y_char,"%d",path_y);
           strcat(message,path_x_char);
           strcat(message,",");
           strcat(message,path_y_char);
           strcat(message,",");
         }

         else if(cout==size-1 && distance_bool)
         {
           path_x=(goal_x-1000*NavfnROS_path->poses[size-2].pose.position.x)/2+1000*NavfnROS_path->poses[size-2].pose.position.x;
           path_y=(goal_y-1000*NavfnROS_path->poses[size-2].pose.position.y)/2+1000*NavfnROS_path->poses[size-2].pose.position.y;

           ROS_INFO("path_x:%d",path_x);
           ROS_INFO("path_y:%d",path_y);
           sprintf(path_x_char,"%d",path_x);
           sprintf(path_y_char,"%d",path_y);
           strcat(message,path_x_char);
           strcat(message,",");
           strcat(message,path_y_char);
           strcat(message,",");
         }

         else if(cout==size-1 && !distance_bool)
         {
           ROS_INFO("path_x:%d",path_x);
           ROS_INFO("path_y:%d",path_y);
           sprintf(path_x_char,"%d",path_x);
           sprintf(path_y_char,"%d",path_y);
           strcat(message,path_x_char);
           strcat(message,",");
           strcat(message,path_y_char);
           strcat(message,",");
         }
         //ROS_INFO("x:%d",x);
         //ROS_INFO("y:%d",y);
         cout++;
       }

       strcat(message,goal_x_char);
       strcat(message,",");
       strcat(message,goal_y_char);
       strcat(message,",");
       strcat(message,goal_theta_char);
       strcat(message,",");
       strcat(message,"*");
       ROS_INFO("%s",message);
       cout=0;
       global_plan_to_message_bool=true;
   }
    goal_cout++;
 }

 void global_cout(const move_base_msgs::MoveBaseActionGoalConstPtr goal)
 {
   goal_x=1000*goal->goal.target_pose.pose.position.x;
   goal_y=1000*goal->goal.target_pose.pose.position.y;
   double orientation_z=goal->goal.target_pose.pose.orientation.z;
   double orientation_w=goal->goal.target_pose.pose.orientation.w;
   int goal_theta=atan2(2*orientation_w*orientation_z, orientation_w*orientation_w - orientation_z* orientation_z)*180/M_PI;

   /*ROS_INFO("x:%d",goal_x);
   ROS_INFO("y:%d",goal_y);
   ROS_INFO("orinetation_z:%f",orientation_z);
   ROS_INFO("orinetation_w:%f",orientation_w);
   ROS_INFO("goal_theta:%d",goal_theta);*/

   sprintf(goal_x_char,"%d",goal_x);
   sprintf(goal_y_char,"%d",goal_y);
   sprintf(goal_theta_char,"%d",goal_theta);
   goal_cout=0;
 }

 int main(int argc, char ** argv)
{

  char inputBuffer[32]={};
  bool server_run_bool=false;

  struct sockaddr_in serverInfo;
  int sockfd ,n ;

  ros::init(argc, argv, "tcp_navigation_to_p3dx_client");
  ros::NodeHandle nh;
  ros::Subscriber tf_echo_pose2D_subscriber=nh.subscribe("/gmapping/Pose2D",10,pose_to_message);
  ros::Subscriber NavfnROS_path_subscriber=nh.subscribe("/move_base/NavfnROS/plan",10,NavfnROS_path_to_message);
  ros::Subscriber global_subscriber=nh.subscribe("/move_base/goal",10,global_cout);

  while(true)
  {
    int err=-1;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    serverInfo.sin_family = AF_INET;
    serverInfo.sin_port = htons(8787);
    inet_pton(AF_INET, "127.0.0.1", &serverInfo.sin_addr.s_addr);

    ROS_INFO("Start Connect");
    while(!err==0)
    {
       err=connect(sockfd, (struct sockaddr *)&serverInfo, sizeof(serverInfo));
    }
    ROS_INFO("Connection success");

    memset(inputBuffer, 0, sizeof(inputBuffer));
    n = read(sockfd, inputBuffer, sizeof(inputBuffer));
    if(inputBuffer[0]=='1')
    {
      server_run_bool=true;
    }

     while(ros::ok() && server_run_bool)
    {
        ros::spinOnce();
        if(pose_to_message_bool || global_plan_to_message_bool)
        {
           write(sockfd , message , sizeof(message));
           n=read(sockfd, inputBuffer, sizeof(inputBuffer));
           if(n==4)
           {
             server_run_bool=true;
           }
           else if(n==0)
           {
             server_run_bool=false;
           }
           pose_to_message_bool=false;
           global_plan_to_message_bool=false;
        }
    }
     close(sockfd);
  }
   return 0;
}

