#include <stdio.h>
#include "serial.h"
#include <sys/time.h>
#include <stdlib.h>
#include <sys/time.h>
#include <iostream>
#include <cmath>
#include <ctime>
#include <unistd.h>
#include <chrono>
#include <ratio>

using namespace std;
using namespace std::chrono;

const int TEAM_ID = 'z';  // yellow
const double Radius=0.087; //bot radius
const double radius=0.025; //wheel radius
const double PI=3.141592653589793;
const int max_vel_bot=1800;

// double max_vel_wheel=5000.0*radius/60.0;
double max_vel_wheel=5000.0; //this is in rpm
double theta[4]={30.0,150.0,225.0,315.0};
double thetaa,v;
double* duty_calc(int vel_euc[])
{
     double vel_wheel[4];
     int vx=vel_euc[0];
     int vy=vel_euc[1];
     int vw=vel_euc[2];

     double insignificant_array[4] = { 0.2 , -0.2 , 0.245 , -0.245 };
     
     double insignificant_factor[4] = { 1.732,-1.732,1.414,-1.414 };

     double insignificant_variable = 0;



     v=sqrt(vx*vx+vy*vy);
     thetaa=atan2(vx,vy);
    
    for(int i=0; i <4; ++i){
        // printf(" sin: %f, \tcos: %f ,\tvx*sin: %f ,\tvy*cos: %f ,\t:theta: %f \n", sin(theta[i]) , cos(theta[i]) , vx * sin(theta[0]) , vy * cos(theta[0]) , theta[i]);
    }

        // printf("\n---------------\n%d %d %d  \n-----------\n",vel_euc[0],vel_euc[1],vel_euc[2]);
        // printf("\n---------------\n%d %d %d  \n-----------\n",vx,vy,vw);
         vel_wheel[0] =   (( (Radius * vw) - (vx * sin(theta[0])) + (vy * cos(theta[0]))) )/radius;
        // printf("\n---------------\n%f   \n-----------\n",vel_wheel[0]);
         vel_wheel[1] =   (( (Radius * vw) - (vx * sin(theta[1])) + (vy * cos(theta[1]))) )/radius;
         vel_wheel[2] =   (( (Radius * vw) - (vx * sin(theta[2])) + (vy * cos(theta[2]))) )/radius;
         vel_wheel[3] =   (( (Radius * vw) - (vx * sin(theta[3])) + (vy * cos(theta[3]))) )/radius;
         
         // printf("\n---------------\n%lf %lf %lf %lf \n-----------\n",vel_wheel[0],vel_wheel[1],vel_wheel[2],vel_wheel[3]);
         
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
     //printf("%f -----------------\t",da[i]);
    }
    //printf("\n");

  
    return da;

}
int main(int argc, char const *argv[])
{

   for (int i = 0; i < 4; ++i)
     {
         theta[i]=theta[i]*PI/180;
     }

    int vel_euc[3];
    int vel_wheel[4];
    int wheel_vel[4];
    int updated_vel_euc[3];

    HAL::Serial serial;
    if(!serial.Open("/dev/ttyUSB0", 9600)) {
        printf("Could not open fucking port.\n");
        exit(0);
    }



    // [ VX VY W ] without rotational motion
    while (1) {
        fflush(stdin);
        unsigned char buf[8]; 
        scanf("%d %d %d %hhu %hhu",&vel_euc[0],&vel_euc[1],&vel_euc[2],&buf[6],&buf[7]);

        double *d=duty_calc(vel_euc);

        for (int i = 2; i <=5; i++) {
          buf[i] = ((int)d[i-2]);
          printf("%d \n",buf[i]);      
        }
        buf[1]= 1;
        buf[0] = 1;
        // buf[5] = 250 ;
        // buf[7] = 0 ;

        // buf[1] = vel_euc[0];
        // buf[2] = vel_euc[1];
        // buf[3] = vel_euc[2];
        // buf[4] = vel_euc[3];
        // buf[5] = 0;  


        serial.Write(buf, 8);
        printf("sending: %d %d %d %d %d %hhu %hhu\n", buf[1], buf[2], buf[3], buf[4], buf[5],buf[6],buf[7]);
    }

     return 0;
  }
    
   
