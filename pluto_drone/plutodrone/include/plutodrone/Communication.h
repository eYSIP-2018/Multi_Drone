
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdio.h>
#include <string>
//#include <plutodrone/Protocol.h>









static const int MSP_FC_VERSION=3;
static const int MSP_RAW_IMU=102;
static const int MSP_RC = 105;
static const int MSP_ATTITUDE=108;
static const int MSP_ALTITUDE=109;
static const int MSP_ANALOG=110;
static const int MSP_SET_RAW_RC=200;
static const int MSP_ACC_CALIBRATION=205;
static const int MSP_MAG_CALIBRATION=206;
static const int MSP_SET_MOTOR=214;
static const int MSP_SET_ACC_TRIM=239;
static const int MSP_ACC_TRIM=240;
static const int MSP_EEPROM_WRITE = 250;
static const int MSP_SET_POS= 216;

static const int IDLE = 0, HEADER_START = 1, HEADER_M = 2, HEADER_ARROW = 3, HEADER_SIZE = 4, HEADER_CMD = 5, HEADER_ERR = 6;















using namespace std;

//extern int socketSyckLock;

//extern int socketOpStarted;

class Communication{


public:



/*
int indx=0;
unsigned int len = 0;
uint8_t checksum=0;
uint8_t command=0;
uint8_t payload_size=0;

int optval;
socklen_t optlen = sizeof(optval);

//int socketSyckLock=0;
//int socketOpStarted=0;
int checksumIndex=0;
uint8_t recbuf[1024];

 int c_state = IDLE;
     uint8_t c;
     bool err_rcvd = false;
     int offset = 0, dataSize = 0;
    // uint8_t checksum = 0;
     uint8_t cmd;
     //byte[] inBuf = new byte[256];
     int i = 0;

*/


//int inputBuffer[1024];
//int offset;
//int bufferIndex=0;



//std::string MSP_HEADER="$M<";

uint8_t checksum=0;

int optval;
socklen_t optlen = sizeof(optval);

uint8_t recbuf[1024];

 int c_state = IDLE;
     uint8_t c;
     bool err_rcvd = false;
     int offset = 0, dataSize = 0;
     uint8_t cmd;
     








int socketSyckLock=0;
int socketOpStarted=0;

bool connectSock(std::string);
bool disconnectSock();



/// Protocol.h content
int8_t inputBuffer[1024];
uint8_t bufferIndex=0;


int roll;
int pitch;
int yaw;
float battery;
int rssi;

float accX;
float accY;
float accZ;

float gyroX;
float gyroY;
float gyroZ;

float magX;
float magY;
float magZ;

float alt;

int FC_versionMajor;
int FC_versionMinor;
int FC_versionPatchLevel;

int trim_roll;
int trim_pitch;


float rcThrottle, rcRoll, rcPitch, rcYaw, rcAUX1 , rcAUX2, rcAUX3, rcAUX4 ;









//   Protocol.h content

int read8();
int read16();
int read32();

void evaluateCommand(int command);


void sendRequestMSP(std::vector<int8_t> data);

void sendRequestMSP_SET_RAW_RC(int channels[]);

void sendRequestMSP_SET_POS(int posArray[]);


void sendRequestMSP_GET_DEBUG(std::vector<int> requests);

std::vector<int8_t> createPacketMSP(int msp, std::vector<int8_t>payload);














int writeSock(const void *buf, int count);

uint8_t readSock(void *buf, int count);

void readFrame();





private:

int sockID;






};


#endif
