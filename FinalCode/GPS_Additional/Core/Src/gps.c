#include <gps.h>
#include <stdio.h>
#include <string.h>

// the function needed

uint8_t rx_data = 0;
uint8_t rx_buffer[GPSBUFSIZE];
uint8_t rx_index = 0;

// struct GPS_t GPS; // actual definition of the GPS_t object will be placed in the main.c file.

void GPS_Init() {
	HAL_UART_Receive_IT(GPS_USART, &rx_data, 1);
}

void GPS_UART_CallBack() {
	if (rx_data != '\n' && rx_index < sizeof(rx_buffer)) {
		rx_buffer[rx_index++] = rx_data;
	} else {

#if (GPS_DEBUG == 1)
		GPS_print((char*)rx_buffer);
		#endif

		if (GPS_validate((char*) rx_buffer))
			GPS_parse((char*) rx_buffer);
		rx_index = 0;
		memset(rx_buffer, 0, sizeof(rx_buffer));
	}
	HAL_UART_Receive_IT(GPS_USART, &rx_data, 1);
}

int GPS_validate(char *nmeastr) {
	char check[3];
	char checkcalcstr[3];
	int i;
	int calculated_check;

	i = 0;
	calculated_check = 0;

	// check to ensure that the string starts with a $
	if (nmeastr[i] == '$')
		i++;
	else
		return 0;

	//No NULL reached, 75 char largest possible NMEA message, no '*' reached
	while ((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 75)) {
		calculated_check ^= nmeastr[i]; // calculate the checksum
		i++;
	}

	if (i >= 75) {
		return 0; // the string was too long so return an error
	}

	if (nmeastr[i] == '*') {
		check[0] = nmeastr[i + 1];    //put hex chars in check string
		check[1] = nmeastr[i + 2];
		check[2] = 0;
	} else
		return 0;    // no checksum separator found there for invalid

	sprintf(checkcalcstr, "%02X", calculated_check);
	return ((checkcalcstr[0] == check[0]) && (checkcalcstr[1] == check[1])) ?
			1 : 0;
}

void GPS_parse(char *GPSstrParse) {
	if (!strncmp(GPSstrParse, "$GPGGA", 6)) {
		if (sscanf(GPSstrParse, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c",
				&GPS.utc_time, &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude,
				&GPS.ew, &GPS.lock, &GPS.satelites, &GPS.hdop,
				&GPS.msl_altitude, &GPS.msl_units) >= 1) {
			GPS.dec_latitude = GPS_nmea_to_dec(GPS.nmea_latitude, GPS.ns);
			GPS.dec_longitude = GPS_nmea_to_dec(GPS.nmea_longitude, GPS.ew);
			convertUTCTime(GPS.utc_time, &GPS.hours, &GPS.minutes,
					&GPS.seconds);

			return;
		}
	} else if (!strncmp(GPSstrParse, "$GPRMC", 6)) {
		if (sscanf(GPSstrParse, "$GPRMC,%f,%c,%f,%c,%f,%c,%f,%f,%d,%f,%c", &GPS.utc_time, &GPS.rmc_status, &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude,
				&GPS.ew, &GPS.speed_k, &GPS.course_d, &GPS.date, &GPS.magnetic, &GPS.mag) >= 1) {
			GPS.dec_latitude = GPS_nmea_to_dec(GPS.nmea_latitude, GPS.ns);
			GPS.dec_longitude = GPS_nmea_to_dec(GPS.nmea_longitude, GPS.ew);
			convertUTCTime(GPS.utc_time, &GPS.hours, &GPS.minutes,
					&GPS.seconds);
			convertDate(GPS.date, &GPS.year, &GPS.month, &GPS.day);
			return;
		}

	} else if (!strncmp(GPSstrParse, "$GPGLL", 6)) {
		if (sscanf(GPSstrParse, "$GPGLL,%f,%c,%f,%c,%f,%c", &GPS.nmea_latitude,
				&GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.utc_time,
				&GPS.gll_status) >= 1) {
			GPS.dec_latitude = GPS_nmea_to_dec(GPS.nmea_latitude, GPS.ns);
			GPS.dec_longitude = GPS_nmea_to_dec(GPS.nmea_longitude, GPS.ew);
			convertUTCTime(GPS.utc_time, &GPS.hours, &GPS.minutes,
					&GPS.seconds);
			return;
		}
	} else if (!strncmp(GPSstrParse, "$GPVTG", 6)) {
		if (sscanf(GPSstrParse, "$GPVTG,%f,%c,%f,%c,%f,%c,%f,%c", &GPS.course_t,
				&GPS.course_t_unit, &GPS.course_m, &GPS.course_m_unit,
				&GPS.speed_k, &GPS.speed_k_unit, &GPS.speed_km,
				&GPS.speed_km_unit) >= 1)
			return;
	}
}

float GPS_nmea_to_dec(float deg_coord, char nsew) {
	int degree = (int) (deg_coord / 100);
	float minutes = deg_coord - degree * 100;
	float dec_deg = minutes / 60;
	float decimal = degree + dec_deg;
	if (nsew == 'S' || nsew == 'W') { // return negative
		decimal *= -1;
	}
	return decimal;
}

void convertUTCTime(float utcTime, int *hours, int *minutes, int *seconds) {
	int totalSeconds = (int) utcTime;
	*seconds = totalSeconds % 100;
	*minutes = (totalSeconds / 100) % 100;
	*hours = (totalSeconds / 10000) + 8;

}

void convertDate(int date, int *year, int *month, int *day) {
	*year = date % 100;
	*month = (date / 100) % 100;
	*day = date / 10000;
}

