/// Author: Keegan Neave, 
/// Provided as is, no warranty, no guarantee of functionality, use at your own risk

#include <Arduino.h>
#include <math.h>

#define MIN_INTENSITY 150
#define MIN_DISTANCE 100    // in millimeters
#define MAX_DISTANCE 2500   // in millimeters

#define POINT_PER_PACK 12
#define HEADER 0x54

typedef struct __attribute__((packed)) 
{
  float x_offset;           // in millimeters from the reference point
  float y_offset;           // in millimeters from the reference point
  float angle_offset;          // in degrees as that's what the lidar outputs and this maintains consistency
  HardwareSerial* serial;      // A pointer to the serial port that the lidar is connected to
  uint8_t motor_pin;           // The pin that the motor controller is connected to, if used
} LidarDeviceStructDef;

LidarDeviceStructDef lidarDevices[] = {
  {  0,  81,   0, &Serial1, -1},
  { 80,   0,  90, &Serial2, -1},
  {  0, -81, 180, &Serial3, -1},
  {-80,   0, 270, &Serial4, -1}
};

typedef struct __attribute__((packed)) 
{
  float x;           // distance in millimeters from the reference point
  float y;           // distance in millimeters from the reference point
  uint8_t intensity;    // value of the intensity of the return
  uint8_t id;           // the ID of the lidar that the point came from
} PositionStructDef;

typedef struct __attribute__((packed)) 
{
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) 
{
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
}LiDARFrameTypeDef;

static const uint8_t CrcTable[256] ={
  0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
  0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
  0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
  0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
  0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
  0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
  0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
  0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
  0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
  0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
  0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
  0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
  0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
  0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
  0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
  0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
  0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
  0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
  0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
  0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
  0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
  0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

uint8_t CalCRC8(uint8_t *p, uint8_t len)
{
  uint8_t crc = 0;
  uint16_t i;
  for (i = 0; i < len; i++)
  {
    crc = CrcTable[(crc ^ *p++) & 0xff];
  }
  return crc;
}

LiDARFrameTypeDef lidarFrame;
void ReadLidar(int lidarNum);

void setup() {
  Serial.begin(5000000);  
  lidarDevices[0].serial->begin(230400, SERIAL_8N1);
  lidarDevices[1].serial->begin(230400, SERIAL_8N1);
  lidarDevices[2].serial->begin(230400, SERIAL_8N1);
  lidarDevices[3].serial->begin(230400, SERIAL_8N1);
}

byte lidarFrameBuffer[57];
float deg2rad = 3.14159265359 / 180.0;

void loop() {
  ReadLidar(0);
  ReadLidar(1);
  ReadLidar(2);
  ReadLidar(3);
}

void ReadLidar(int lidarNum)
{
  while(lidarDevices[lidarNum].serial->available() > 0)
  {
    char c = lidarDevices[lidarNum].serial->read();
    if(c == HEADER) 
    {
      lidarFrameBuffer[0] = c;
      while(lidarDevices[lidarNum].serial->available() < 56);
      {
        lidarDevices[lidarNum].serial->readBytes(&lidarFrameBuffer[1], 56);
        memcpy(&lidarFrame, lidarFrameBuffer, sizeof(lidarFrame));
      }
      
      uint8_t crc = CalCRC8((uint8_t*)&lidarFrame, sizeof(lidarFrame) - 1);
      
      if(crc == lidarFrame.crc8)
      {
        float start_angle = ((float)lidarFrame.start_angle / 100);
        float end_angle   = ((float)lidarFrame.end_angle / 100);
        float angle_diff  = (end_angle - start_angle) / (POINT_PER_PACK - 1);
        
        for(int i = 0; i < POINT_PER_PACK; i++)
        {
          PositionStructDef point;
          float angle = (start_angle + (i * angle_diff));
          float angleAdjustedForLidarRotation = angle + lidarDevices[lidarNum].angle_offset;
          float radians = angleAdjustedForLidarRotation * deg2rad;
          float adjustedRadian = radians - (PI/2);    // This is adjusting to match polar coordinates

          // // Ignore message if the angle is between 180 and 270 degrees
          // if(angle > 180 && angle < 270)
          // {
          //   continue;
          // }

          point.x = -(cosf(adjustedRadian)) * lidarFrame.point[i].distance;
          point.y =  (sinf(adjustedRadian)) * lidarFrame.point[i].distance;
          point.intensity = lidarFrame.point[i].intensity;
          point.id = lidarNum;

          // Output the point data
          // Serial.print(lidarNum);
          // Serial.print(", ");
          // Serial.print(start_angle + i * angle_diff);
          // Serial.print(", ");
          // Serial.print(lidarFrame.point[i].distance);
          // Serial.print(", ");
          // Serial.print(lidarFrame.point[i].intensity);
          // Serial.println();
          
          if(lidarFrame.point[i].distance > MIN_DISTANCE && lidarFrame.point[i].distance < MAX_DISTANCE && lidarFrame.point[i].intensity > MIN_INTENSITY)
          {            
            Serial.print(point.id);
            Serial.print(",");
            Serial.print(angle);
            Serial.print(",");
            Serial.print(lidarFrame.point[i].distance);
            Serial.print(",");
            Serial.print(point.x);
            Serial.print(",");
            Serial.print(point.y);
            Serial.print(",");
            Serial.print(point.intensity);
            Serial.println();
          }
        }
      }
      else
      {
        // Serial.println("CRC Error");
      }
    }
  }
}
