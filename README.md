# Safran_Coding_Challenge
Code to establish the connection between 2 ESP32 SoCs and to synchronize the data

DESCRIPTION:
Let device A and device B be connected through UART.And the sensors are also connected with dev A through UART.This code will be running on the device A.

This application is written over FreeRTOS.There will be 3 tasks running in the dev A.

1.UART1_Task - To collect the data from sensors connected to dev A using UART1.(polls for the data with 1 sec delay)

2.UART2_Task - To collect the data from dev B using UART2.(polls for the data with 1 sec delay)

3.Device_Task - To send the combined data to dev B through UART 2.

For every 5 secs(timer is used) the data from sensors in dev A (UART1_Task) and the data from dev B(UART2_Task) will be combined and stored in the global variable sCommonData.i32CombinedSensorData and a copy of it will be send to the dev B.

TIME TAKEN:
Time taken to complete this task is 2.5 hrs for coding and other 2 hrs for understanding the ESP32 datasheet and to get familiar with the IDE to be used for ESP32

MODIFICATIONS TO BE DONE:
This can be improvised by dynamically allocating the memory for the table storing the memory instead of giving the fixed value of 50.
