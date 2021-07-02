#ifndef ESP32_H_
#define ESP32_H_



#define SENSORDATA_UPDATE_TIMER_DELAY  (5000)
#define SENSORDATA_UPDATE_TIMER         (0)
#define SENSORDATA_MAX_COUNT            (50)

#define DEVICE_TASK_SIZE            (2048)
#define DEVICE_TASK_PRIORITY        (configMAX_PRIORITIES - 2)
#define DEVICE_QUEUE_MAX_SIZE        (10)

#define UART1_TASK_SIZE            (2048)
#define UART1_TASK_PRIORITY        (configMAX_PRIORITIES)
#define UART2_TASK_SIZE            (2048)
#define UART2_TASK_PRIORITY        (configMAX_PRIORITIES)


typedef enum 
{
  SYSTEM_SUCCESS,
  SYSTEM_FAILURE
}System_Status;


typedef enum 
{
  DEVICE_SENSORDATAUPDATE_MSG
}Device_MSgType;

typedef struct
{
  Device_MSgType eMsgType;
  int32_t i32SensorData[SENSORDATA_MAX_COUNT][2];
}Device_MsgBuf;


typedef struct
{
  int32_t i32SensorDataFromDevB[SENSORDATA_MAX_COUNT][2];
  int32_t i32SensorDataFromDevB[SENSORDATA_MAX_COUNT][2];
  int32_t i32CombinedSensorData[SENSORDATA_MAX_COUNT][2];
}ESP32_CommonData;





#endif
