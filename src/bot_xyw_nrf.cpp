#include <stdio.h>
#include "serial.h"
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "krssg_ssl_msgs/gr_BallReplacement.h"
#include "krssg_ssl_msgs/gr_Robot_Command.h"
#include "krssg_ssl_msgs/gr_RobotReplacement.h"
#include "krssg_ssl_msgs/gr_Packet.h"
#include "krssg_ssl_msgs/gr_Commands.h"
#include "krssg_ssl_msgs/gr_Replacement.h"
#include <krssg_ssl_msgs/sslDebug_Data.h>

#define RATIO1 14.91
#define RATIO 40    
#define FACTOR_T RATIO
#define FACTOR_N RATIO
#define FACTOR_W 90

using namespace std;
const int TEAM_ID = 1;
//const int TEAM_ID = 1;  // yellow
const double Radius=0.087; //bot radius
const double radius=0.025; //wheel radius
const double PI=3.141592653589793;
const int max_vel_bot=1800;
unsigned static char buf[32];
// double max_vel_wheel=5000.0*radius/60.0;
double max_vel_wheel=5000.0; //this is in rpm
double theta[4]={30.0,150.0,225.0,315.0};
double thetaa,v;
HAL::Serial serial;


     double vel_wheel[4];
void duty_calc(int vel_euc[])
{
     int vx=vel_euc[0];
     int vy=vel_euc[1];
     int vw=vel_euc[2];
     // cout << "---------------- vw = " << vw << " --------------------------" <<endl;
     vw*=-1;

     double insignificant_array[4] = { 0.2 , -0.2 , 0.245 , -0.245 };
     
     double insignificant_factor[4] = { 1.732,-1.732,1.414,-1.414 };

     double insignificant_variable = 0;



     v=sqrt(vx*vx+vy*vy);
     thetaa=atan2(vx,vy);

       
         vel_wheel[0] =   (( (Radius * vw) - (vx * sin(theta[0])) + (vy * cos(theta[0]))) )/radius;
      	 vel_wheel[1] =   (( (Radius * vw) - (vx * sin(theta[1])) + (vy * cos(theta[1]))) )/radius;
         vel_wheel[2] =   (( (Radius * vw) - (vx * sin(theta[2])) + (vy * cos(theta[2]))) )/radius;
         vel_wheel[3] =   (( (Radius * vw) - (vx * sin(theta[3])) + (vy * cos(theta[3]))) )/radius;
        
         for (int i = 0; i < 4; ++i)
         {
             vel_wheel[i]=vel_wheel[i]/PI;
         }

    double* da=vel_wheel;
    
    for (int i = 0; i < 4; ++i)
    {
        if(vel_wheel[i]>0)
        {
            da[i] = 127 + ((vel_wheel[i]-max_vel_wheel)*(127.000))/max_vel_wheel;
        }
        else
        {
            da[i] = 256 - ((vel_wheel[i]-0)*(128.000-256.000))/max_vel_wheel;
        }
        cout<<"da "<<vel_wheel[i]<<"    "<<da[i]<<endl;
    }

}

void CallBack(const krssg_ssl_msgs::gr_Commands::ConstPtr& msg)
{
	  // [ VX VY W ] without rotational motion
		int vel_euc[3];

       // cout<<msg->robot_commands.veltangent<<" "<<msg->robot_commands.velnormal<<" "<<msg->robot_commands.velangular<<endl;
		vel_euc[0] = (int)(msg->robot_commands.velnormal * FACTOR_T);
		vel_euc[1] = -1*(int)(msg->robot_commands.veltangent * FACTOR_N);
		vel_euc[2] =(int)(msg->robot_commands.velangular * FACTOR_W); 
//        cout<<"vel_euc\n";
		printf("%d %d %d %d\n",msg->robot_commands.id, vel_euc[0], vel_euc[1], vel_euc[2]);
	 buf[0] = TEAM_ID; 
   if(msg->robot_commands.id==2){
         duty_calc(vel_euc);
         cout<<"after dc "<<vel_wheel[0]<<endl;
         cout<<"after dc "<<vel_wheel[1]<<endl;
         cout<<"after dc "<<vel_wheel[2]<<endl;
         cout<<"after dc "<<vel_wheel[3]<<endl;
    for(int i=1;i<=4;i++){
        if(msg->robot_commands.id-1>=0&&msg->robot_commands.id-1<=5){
        for(int j=0;j<6;j++){
        buf[(j)*5+i]=(int)vel_wheel[i-1];
       }
        //   cout <<msg->robot_commands.id<< " buf: "<<i<<": " << vel_wheel[i-1] << endl;
    }
    }
    //buf[6] = 5;
    //buf[7] = 5;
    //buf[8] = 5;
    //buf[9] = 5;
    for(int i=0;i<32;i++){
        cout<<(int)buf[i]<<" , ";
    }
    cout<<endl;
    serial.Write(buf,32);
   }
	
}

int main(int argc, char *argv[])
{
   ros::init(argc, argv, "bot_comm_node");
   for (int i = 0; i < 4; ++i)
     {
         theta[i]=theta[i]*PI/180;

     }
   ros::NodeHandle n;
   ros::Subscriber subCmd = n.subscribe("/grsim_data", 1000, CallBack);
   

    if(!serial.Open("/dev/ttyUSB0", 19200)) {
        printf("Could not open the fucking parts.\n");
        exit(0);
    }

	ros::spin();

  
    return 0;
  }
   
