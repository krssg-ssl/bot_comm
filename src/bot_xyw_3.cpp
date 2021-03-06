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
#define RATIO 25    
#define FACTOR_T RATIO
#define FACTOR_N RATIO
#define FACTOR_W 80

using namespace std;

const int TEAM_ID = 1;  // yellow
const double Radius=0.087; //bot radius
const double radius=0.025; //wheel radius
const double PI=3.141592653589793;
const int max_vel_bot=1800;

// double max_vel_wheel=5000.0*radius/60.0;
double max_vel_wheel=5000.0; //this is in rpm
double theta[4]={30.0,150.0,225.0,315.0};
double thetaa,v;
HAL::Serial serial;

double* duty_calc(int vel_euc[])
{
     double vel_wheel[4];
     int vx=vel_euc[0];
     int vy=vel_euc[1];
     int vw=vel_euc[2];
//      cout << "---------------- vw = " << vw << " --------------------------" <<endl;
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
            da[i] = 127 + (vel_wheel[i]-max_vel_wheel)*((127.000)/max_vel_wheel);
        }
        else
        {
            da[i] = 256 - (vel_wheel[i]-0)*((128.000-256.000)/max_vel_wheel);
        }
    
    }

  
    return da;

}

void CallBack(const krssg_ssl_msgs::gr_Commands::ConstPtr& msg)
{
	  // [ VX VY W ] without rotational motion
		int vel_euc[3];
        unsigned char buf[8];
        cout<<msg->robot_commands.veltangent<<" "<<msg->robot_commands.velnormal<<" "<<msg->robot_commands.velangular<<endl;
		vel_euc[0] = (int)(msg->robot_commands.velnormal * FACTOR_T);
		vel_euc[1] = -1*(int)(msg->robot_commands.veltangent * FACTOR_N);
		vel_euc[2] =(int)(msg->robot_commands.velangular) * FACTOR_W; 
     //   cout<<"vel_euc\n";
	//	printf("%d %d %d\n", vel_euc[0], vel_euc[1], vel_euc[2]);

        double *d=duty_calc(vel_euc);

        for (int i = 2; i <=5; i++) {
          buf[i] = ((int)d[i-2]);  
        }
        if(msg->robot_commands.id == 3){
            buf[0] = 127;  
            buf[1] = 3;
            // buf[7] = 0;
            buf[7] = msg->robot_commands.spinner;
    		// buf[6] = msg->robot_commands.spinner;
    		if(msg->robot_commands.kickspeedx!=0)
                buf[6] = 1;
            else 
                buf[6] = 0 ;

            buf[7] = 1;
            serial.Write(buf, 8);
            // fstream file ("/home/kgpkubs/Desktop/ddd.txt",ios::app);
            // file<<(int)(buf[6])<< " "<<(int)(buf[7])<<"\n";
            // file.close();
    		printf("sending: 3\n %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu\n", buf[0],buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
        }
}

int main(int argc, char *argv[])
{
   ros::init(argc, argv, "bot_comm_node");
   ros::NodeHandle n;
   ros::Subscriber subCmd = n.subscribe("/grsim_data", 1000, CallBack);
   
   for (int i = 0; i < 4; ++i)
     {
         theta[i]=theta[i]*PI/180;

     }

    if(!serial.Open("/dev/ttyUSB0", 9600)) {
        printf("Could not open the fucking parts.\n");
        exit(0);
    }

	ros::spin();

  
    return 0;
  }
   
