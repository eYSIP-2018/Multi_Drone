#include "ros/ros.h"
#include "std_msgs/String.h"
#include "plutodrone/PlutoPilot.h"
#include <geometry_msgs/PoseArray.h>
#include <sys/time.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <pthread.h>
#include <unistd.h>
//#include <plutodrone/Common.h>
//#include <plutodrone/Protocol.h>
#include <plutodrone/PlutoMsg.h>
#include <stdlib.h>
#include <string>
#include <plutodrone/Communication.h>
// #include <plutodrone/JoystickClient.h>
// #include <plutodrone/Position.h>


#define PORT 23
#define NODES 2
using namespace std;


bool isSocketCreate[NODES];


Communication com[NODES];
//Protocol pro[2];

//vector of all string ips
vector <string> all_ips;

ros::ServiceClient serviceClient[NODES];
plutodrone::PlutoPilot service[NODES];

int userRC[8]={0,0,0,0,0,0,0,0};


struct ip_struct
{
  int index;
  std::string ip;
};


void *createSocket(void *arg)
{
 struct ip_struct local_var = *(struct ip_struct *)arg;
 isSocketCreate[local_var.index]=com[local_var.index].connectSock(local_var.ip);
 //isSocketCreate=com.connectSock(local_var.ip);
 pthread_exit(NULL);
};
void *writeFunction(void *arg)
{
  
  int *index=(int *)arg;
  
  cout<<"write"<<*index;
  std::vector<int> requests;
  requests.push_back(MSP_RC);
  requests.push_back(MSP_ATTITUDE);
  requests.push_back(MSP_RAW_IMU);
  requests.push_back(MSP_ALTITUDE);

  while(1)
  {
    com[*index].sendRequestMSP_SET_RAW_RC(userRC);
    com[*index].sendRequestMSP_GET_DEBUG(requests);
    //com.sendRequestMSP_SET_RAW_RC(userRC);
    //com.sendRequestMSP_GET_DEBUG(requests);
    usleep(22000);
  }
  pthread_exit(NULL);
}

void *readFunction(void *arg)
{
  
  int *index=(int *)arg;
  cout<<"read"<<*index;
  do 
  {
      com[*index].readFrame();
       //com.readFrame();
    //usleep(5);
  }
  while(1);
  pthread_exit(NULL);
}
void *serviceFunction(void *arg)
{
  int *index=(int *)arg;
  
  while (1) 
  {
      //cout<<"service"<<*index;
   
      if (serviceClient[*index].call(service[*index]))
      {
      
      //cout<<"service"<<*index;
       userRC[0]=service[*index].response.rcRoll;
       userRC[1]=service[*index].response.rcPitch;
       userRC[2]=service[*index].response.rcThrottle;
       userRC[3]=service[*index].response.rcYaw;
       userRC[4]=service[*index].response.rcAUX1;
       userRC[5]=service[*index].response.rcAUX2;
       userRC[6]=service[*index].response.rcAUX3;
       userRC[7]=service[*index].response.rcAUX4;
       
       //if(*index==1)
       //cout<<"alt:"<<com[*index].accX;
       service[*index].request.accX=com[*index].accX;
       service[*index].request.accY=com[*index].accY;
       service[*index].request.accZ=com[*index].accZ;
       service[*index].request.gyroX=com[*index].gyroX;
       service[*index].request.gyroY=com[*index].gyroY;
       service[*index].request.gyroZ=com[*index].gyroZ;
       service[*index].request.magX=com[*index].magX;
       service[*index].request.magY=com[*index].magY;
       service[*index].request.magZ=com[*index].magZ;
       service[*index].request.roll=com[*index].roll;
       service[*index].request.pitch=com[*index].pitch;
       service[*index].request.yaw=com[*index].yaw;
       service[*index].request.alt=com[*index].alt;
       
       /*service.request.accX=com.accX;
       service.request.accY=com.accY;
       service.request.accZ=com.accZ;
       service.request.gyroX=com.gyroX;
       service.request.gyroY=com.gyroY;
       service.request.gyroZ=com.gyroZ;
       service.request.magX=com.magX;
       service.request.magY=com.magY;
       service.request.magZ=com.magZ;
       service.request.roll=com.roll;
       service.request.pitch=com.pitch;
       service.request.yaw=com.yaw;
       service.request.alt=com.alt;*/
      }
  }
 pthread_exit(NULL);
}
void Callback(const plutodrone::PlutoMsg::ConstPtr& msg)
{
 userRC[0] = msg->rcRoll;
 userRC[1] = msg->rcPitch;
 userRC[2] = msg->rcThrottle;
 userRC[3] = msg->rcYaw;
 userRC[4] = msg->rcAUX1;
 userRC[5] = msg->rcAUX2;
 userRC[6] = msg->rcAUX3;
 userRC[7] = msg->rcAUX4;
}


int main(int argc, char **argv)
{
    //struct to pass to create thread
    struct ip_struct ipStructVar;
   
    for(int j=0;j<NODES;j++)
    isSocketCreate[j]=false;
    
    
   all_ips.push_back("192.168.43.208");
   //all_ips.push_back("192.168.4.1");
    all_ips.push_back("192.168.43.243");
    //all_ips.push_back("192.168.43.1");
  
    char topic_name[] = "drone_command_ ";
    char service_name[] = "PlutoService_ ";
  
  
    pthread_t thread[NODES], readThread[NODES], writeThread[NODES], serviceThread[NODES];
    int rc;
    int index[NODES];
    ros::init(argc, argv, "plutonode");
    ros::NodeHandle n;
    ros::Subscriber sub[NODES];
    
    
    for(int i=0;i<NODES;i++)
    {
    
    //Add an index to the topic
    topic_name[strlen(topic_name)-1] = 0x30+i;
    //Add an index to the service
    service_name[strlen(service_name)-1] = 0x30+i;
    
    
    sub[i] = n.subscribe(topic_name, 1000, Callback);
    ipStructVar.index=i;
    ipStructVar.ip=all_ips[i];
    index[i]=i;
    rc = pthread_create(&thread[i], NULL, createSocket, (void *)  &ipStructVar);
    if (rc)
    {
     cout << "Error:unable to create communication thread," << rc << endl;
     exit(-1);
    }
    //if(thread[i].joinable())
    pthread_join( thread[i], NULL);
    //cout<<"socket"<<i<<"before confermation";
    if(isSocketCreate[i])
    {
       //cout<<"socket"<<i<<"created";
      //cout << "main() : creating write thread, " << i << endl;
      rc = pthread_create(&writeThread[i], NULL, writeFunction, (void *) &index[i]);
      if (rc)
      {
        cout << "Error:unable to create write thread," << rc << endl;
        exit(-1);
      }
      // cout << "main() : creating read thread, " << i << endl;
      rc = pthread_create(&readThread[i], NULL, readFunction, (void *) &index[i]);
      if (rc)
      {
        cout << "Error:unable to read create thread," << rc << endl;
        exit(-1);
      }

      serviceClient[i] = n.serviceClient<plutodrone::PlutoPilot>(service_name,true);
      //cout << "main() : creating service thread, " << i << endl;
      rc = pthread_create(&serviceThread[i], NULL, serviceFunction,(void *) &index[i]);
      
      if (rc)
      {
        cout << "Error:unable to service create thread," << rc << endl;
        exit(-1);
      }
    }
    
    } 
  
    ros::spin();
    
    cout<<"main ended";
    return 0;
}
