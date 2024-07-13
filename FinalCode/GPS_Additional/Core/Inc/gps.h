#ifndef __GPS_H
#define __GPS_H

#include "stm32f1xx_hal.h"
// #include "stm32f1xx_hal_uart.h"
// #include "usart.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;

#define GPS_DEBUG 0
#define GPS_USART &huart3
#define GPSBUFSIZE 128 // GPS buffer size

typedef struct
{
    // calculated values
    float dec_longitude;
    float dec_latitude;
    float altitude_ft;
    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;
    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d; //track angle
    float magnetic; //Magnetic Variation
    char mag;
    int date;
    // checksum

    // GLL
    char gll_status;
    // VTG - Course over ground, ground speed
    float course_t; // ground speed true
    char course_t_unit;
    float course_m; // magnetic
    char course_m_unit;
    char speed_k_unit;
    float speed_km; // speek km/hr
    char speed_km_unit;
    // self define
    int hours;
    int minutes;
    int seconds;
    int year;
    int month;
    int day;
} GPS_t;

extern GPS_t GPS; // Extern declaration

void GPS_print(char *data);
void GPS_Init();
void GSP_USBPrint(char *data);
void GPS_print_val(char *data, int value);
void GPS_UART_CallBack();
int GPS_validate(char *nmeastr);
void GPS_parse(char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);
void convertUTCTime(float utcTime, int *hours, int *minutes, int *seconds);
void convertDate(int date, int *year, int *month, int *day);

#endif // __GPS_H
