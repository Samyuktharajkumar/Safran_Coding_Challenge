#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
#include "ESP32.h"


/*Global Variables*/
TimerHandle_t sensorDataTimer;
QueueHandle_t DeviceQueueHandle_glb;
QueueHandle_t uart_queue1;
QueueHandle_t uart_queue2;

const uart_port_t uart_num1 = UART_NUM_1;
const uart_port_t uart_num2 = UART_NUM_2;
static ESP32_CommonData sCommonData;

/*Functions*/
static void CommDev_Task( void *pvParameters );
static void UART1_Task( void *pvParameters );
static void UART2_Task( void *pvParameters );
static System_Status CommDev_Start();
static void SensorData_TimerFunc();
static System_Status Device_Handle(int32_t * i32SensorData);


void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

 
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
    .rx_flow_ctrl_thresh = 122,
};
// Configure UART parameters
ESP_ERROR_CHECK(uart_param_config(uart_num1, &uart_config));
ESP_ERROR_CHECK(uart_param_config(uart_num2, &uart_config));

/*Use UART1 and UART2 default pins*/
ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

const int uart_buffer_size = (1024 * 2);

// Install UART driver using an event queue here
ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, uart_buffer_size,uart_buffer_size, 10, &uart_queue1, 0))
ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size,uart_buffer_size, 10, &uart_queue2, 0));
  
  CommDev_Start();
  
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

/*********************************************************************************************
   Function Name :  CommDev_Start
   Description   :  Starts the task ,timer and queue needed 
   Inputs        :  none
   Outputs       :  SYSTEM_SUCCESS on success ,SYSTEM_FAILURE on failure
   Author        :  Samyuktha R 
   Modified      :  -
   ******************************************************************************************/

System_Status CommDev_Start()
{
    BaseType_t retVal;

     /*Timer Creation*/
    sensorDataTimer = xTimerCreate("SensorDataUpdate Timer", pdMS_TO_TICKS(SENSORDATA_UPDATE_TIMER_DELAY), pdTRUE,
      ( void * )SENSORDATA_UPDATE_TIMER, (TimerCallbackFunction_t) SensorData_TimerFunc);

    if ( sensorDataTimer == NULL )
    {
      return SYSTEM_FAILURE;
    }
   
    /*Queue Creation*/
    DeviceQueueHandle_glb = xQueueCreate( DEVICE_QUEUE_MAX_SIZE, sizeof(Device_MsgBuf));
    if( DeviceQueueHandle_glb == NULL )
    {
      return SYSTEM_FAILURE;
    }

    /*Task Creation*/
    retVal = xTaskCreatePinnedToCore(Device_Task,"TaskDevice",DEVICE_TASK_SIZE,NULL,DEVICE_TASK_PRIORITY,ARDUINO_RUNNING_CORE);
    if(retVal != pdPASS)
    {
      return SYSTEM_FAILURE;
    }

    return SYSTEM_SUCCESS;
}

/*********************************************************************************************
   Function Name :  UART2_Task
   Description   :  Runs to collect data from sensors connected to dev A
   Inputs        :  none
   Outputs       :  none
   Author        :  Samyuktha R 
   Modified      :  -
   ******************************************************************************************/
void UART1_Task(void *pvParameters)
{
   while(1)
   {
      /*reading data from sensor using UART1*/
      uart_read_bytes(uart_num1, sCommonData.i32SensorDataFromDevA,(sizeof(int32_t)*SENSORDATA_MAX_COUNT*2), 100);
      vTaskDelay(67);//1 sec delay
   }
}

/*********************************************************************************************
   Function Name :  UART2_Task
   Description   :  Runs to collect data from dev B 
   Inputs        :  none
   Outputs       :  none
   Author        :  Samyuktha R 
   Modified      :  -
   ******************************************************************************************/
void UART2_Task(void *pvParameters)
{
   while(1)
   {
      /*reading data from sensor using UART1*/
      uart_read_bytes(uart_num2, sCommonData.i32SensorDataFromDevB,(sizeof(int32_t)*SENSORDATA_MAX_COUNT*2), 100);
      vTaskDelay(67);//1 sec delay
   }
}

/*********************************************************************************************
   Function Name :  Device_Task
   Description   :  Runs the task to send out the combined the dev A and dev B data to dev B
   Inputs        :  none
   Outputs       :  none
   Author        :  Samyuktha R 
   Modified      :  -
   ******************************************************************************************/
void Device_Task(void *pvParameters)  
{
  (void) pvParameters;
  Device_MsgBuf sDeviceMsgBuf;
  xTimerStart(sensorDataTimer, TICKS_TO_WAIT );

   
while (1)
   {

    uint8_t u8MsgRcv = xQueueReceive( DeviceQueueHandle_glb, &(sDeviceMsgBuf),portMAX_DELAY) ;
    if (u8MsgRcv == SYSTEM_FAILURE )
    {
      //Serial.println(" %s %d msg not received  \r\n",__func__,__LINE__);
    }
    else
    {
      switch(sDeviceMsgBuf.eMsgType)
      {
        case DEVICE_SENSORDATAUPDATE_MSG:
        {
          Device_Handle();
          break;
        }
        default:
        {
          //default case
        }
      }
   }
}

/***************************************************************************************
   Function Name :  SensorData_TimerFunc
   Description   :  Timer called for every 5 secs to update the combined table 
   Inputs        :  none
   Outputs       :  none
   Author        :  Samyuktha R 
   Modified      :  -
   ******************************************************************************************/
void SensorData_TimerFunc(void)
 {
   Device_MsgBuf  sDevMsgBuf;

   sDevMsgBuf.eMsgType = DEVICE_SENSORDATAUPDATE_MSG;
   
   if((xQueueSend(DeviceQueueHandle_glb ,&(sSensorDataMsgBuf),portMAX_DELAY ) != pdPASS))
   {
    //Serial.println(" %s %d msg not sent  \r\n",__func__,__LINE__);
   }
 }
 
/***************************************************************************************
   Function Name :  Device_Handle
   Description   :  Combines the Dev A and Dev B sensor values and a copy of combined 
                    values to devB
   Inputs        :  none
   Outputs       :  SYTEM SUCCESS
   Author        :  Samyuktha R 
   Modified      :  -
   ******************************************************************************************/

System_Status Device_Handle()
{

  int8_t i8Cnt = 0;

  for(int8_t i8LoopVar = 0;i8LoopVar < SENSORDATA_MAX_COUNT;i8LoopVar++)
  {
    if((sCommonData.i32SensorDataFromDevA[i8LoopVar][0]) == sCommonData.i32SensorDataFromDevB[i8LoopVar][0])&&(sCommonData.i32SensorDataFromDevB[i8LoopVar][0] != 0))
    {
      sCommonData.i32CombinedSensorData[i8Cnt][0] = sCommonData.i32SensorDataFromDevB[i8LoopVar][0];
      sCommonData.i32CombinedSensorData[i8Cnt][1] = sCommonData.i32SensorDataFromDevB[i8LoopVar][1];
      i8Cnt++;
    }
    else if(sCommonData.i32SensorDataFromDevA[i8LoopVar][0]) != 0)
    {
      sCommonData.i32CombinedSensorData[i8Cnt][0] = sCommonData.i32SensorDataFromDevA[i8LoopVar][0];
      sCommonData.i32CombinedSensorData[i8Cnt][1] = sCommonData.i32SensorDataFromDevA[i8LoopVar][1]);
      i8Cnt++;
    }
    else if(sCommonData.i32SensorDataFromDevB[i8LoopVar][0] != 0)
    {
      sCommonData.i32CombinedSensorData[i8Cnt][0] = sCommonData.i32SensorDataFromDevB[i8LoopVar][0];
      sCommonData.i32CombinedSensorData[i8Cnt][1] = sCommonData.i32SensorDataFromDevB[i8LoopVar][1];
      i8Cnt++; 
    }
      
  }

  /*sending data to dev B*/
  uart_write_bytes(uart_num2,sCommonData.i32CombinedSensorData , (sizeof(int32_t)*SENSORDATA_MAX_COUNT*2));
  return SYSTEM_SUCCESS;
}


   
