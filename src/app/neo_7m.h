#ifndef NEO_7M_H
#define NEO_7M_H

#include <stdint.h>


/********* Error Codes *********/
typedef int32_t neo_7m_error_t;
#define NEO_7M_OK             0
#define NEO_7M_DMA_ERROR    (-1)
#define NEO_7M_MUTEX_ERROR  (-2)
#define NEO_7M_PARSE_ERROR  (-3)

#define MIN_NMEA_SENTENCE_LENGTH 10 // Minimum length for a valid NMEA sentence (e.g. $GPRMC,0.00,V,,,,,,,)
#define SPEED_KNOTS_TO_KMH 1.852f

/************Length of a GPS data line******** */
#define GPS_LINE_BUF 128

/************ GPS Data Structure ************/
typedef struct {
    double latitude; // Latitude in decimal degrees
    double longitude; // Longitude in decimal degrees
    double altitude; // Altitude in meters
    double speed; // Speed in km/h
    uint8_t num_satellites; // Number of satellites in view
    char direction; // 'N', 'S', 'E', 'W'
} gps_data_t;

/* Global instance of GPS data */
extern gps_data_t g_neo_7m_data;

/** Public API */
neo_7m_error_t neo_7m_init(void);
neo_7m_error_t read_neo_7m_data(gps_data_t* data, uint32_t timeout);
void neo_7m_task(void* params);

#endif // NEO_7M_H