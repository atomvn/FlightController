/**
 * @file neo_7m.c
 * @brief Neo7m GPS Module Driver Implementation
 *
 * @details
 * Implements initialization and deinitialization of the Neo7m GPS module using UART3 and DMA.
 * Provides API for reading GPS data with thread safety.
 * 
 * @author Hao Nguyen
 * @version 1.0
 * @date 2026
 */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/dma.h>

#include "driver/dma.h"
#include "driver/uart.h"
#include "neo_7m.h"

#define NEO_7M_DEBUG_ENABLED 1

/* Global instance of GPS data */
gps_data_t g_neo_7m_data = {0};

/* Buffer to store a single line of GPS data */
// static char gps_line[GPS_LINE_BUF]; 

static SemaphoreHandle_t gps_data_mutex = NULL;

/** @brief Initialize Neo7m GPS module 
 * @return Error code indicating success or type of failure
*/
neo_7m_error_t neo_7m_init(void) {
    if (g_gps_dma_state.initialized) {
        return NEO_7M_OK; // Already initialized, not an error
    }

    dma_error_t dma_err = gps_dma_init();
    if (dma_err != DMA_OK) {
        return NEO_7M_DMA_ERROR; // DMA initialization failed
    }
    
    if (gps_data_mutex == NULL) {
        gps_data_mutex = xSemaphoreCreateMutex();
        if (gps_data_mutex == NULL) {
            return NEO_7M_MUTEX_ERROR; // Failed to create mutex
        }
    }
    #if NEO_7M_DEBUG_ENABLED
    uart_printf("[OK] Neo7m GPS module initialized successfully\n");
    #endif
    return NEO_7M_OK;

}

/** @brief Get the current position in the GPS DMA buffer
 * @return Current position index in the GPS DMA buffer
*/
static uint16_t gps_dma_get_pos(void) {
    uart_printf("[DEBUG] GPS DMA buffer position: %u\n", GPS_DMA_BUF_SIZE - dma_get_number_of_data(GPS_DMA_CONTROLLER, GPS_DMA_CHANNEL));
    return GPS_DMA_BUF_SIZE - dma_get_number_of_data(GPS_DMA_CONTROLLER, GPS_DMA_CHANNEL);
}

/** @brief Convert NMEA coordinate to decimal degrees
 * @details The way to convert NMEA coordinates is to separate the degrees and minutes, convert the minutes to degrees, and then combine them. 
 * The direction character determines the sign of the result.
 * @param[in] raw Raw NMEA coordinate (e.g. 4916.45 for 49 degrees 16.45 minutes)
 * @param[in] dir Direction character ('N', 'S', 'E', 'W')
 * @return Coordinate in decimal degrees, negative for South and West
*/
static double nmea_to_decimal(double raw, char dir)
{
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);

    double decimal = degrees + (minutes / 60.0);

    if (dir == 'S' || dir == 'W')
        decimal = -decimal;

    return decimal;
}

/** @brief Convert string to double
 * @details A simple implementation of atof that handles optional leading '-' and decimal points. 
 * It does not handle scientific notation or other edge cases, but is sufficient for parsing GPS data.
 * @param[in] str Input string representing a floating-point number
 * @return Converted double value
*/
static double my_atof(const char* str) {
    double result = 0.0;
    double fraction = 1.0;
    int sign = 1;

    if (*str == '-') {
        sign = -1;
        str++;
    }

    while (*str) {
        if (*str == '.') {
            str++;
            break;
        }
        result = result * 10.0 + (*str - '0');
        str++;
    }

    while (*str) {
        fraction /= 10.0;
        result += (*str - '0') * fraction;
        str++;
    }

    return sign * result;
}

/** @brief Convert string to integer
 * @details A simple implementation of atoi that handles optional leading '-'. 
 * It does not handle edge cases like overflow, but is sufficient for parsing GPS data.
 * @param[in] str Input string representing an integer
 * @return Converted integer value
*/
static int16_t my_atoi(const char* str) {
    int16_t result = 0;
    int sign = 1;

    if (*str == '-') {
        sign = -1;
        str++;
    }

    while (*str) {
        result = result * 10 + (*str - '0');
        str++;
    }

    return sign * result;
}

/** @brief Check if a string starts with a given prefix
 * @param[in] str Input string to check
 * @param[in] prefix Prefix to look for
 * @return true if str starts with prefix, false otherwise
*/
static bool starts_with(const char* str, const char* prefix) {
    while (*prefix) {
        if (*prefix++ != *str++) {
            return false;
        }
    }
    return true;
}

static uint16_t my_strlen(const char* str) {
    uint16_t len = 0;
    while (str[len]) {
        len++;
    }
    return len;
}

static neo_7m_error_t parse_nmea_sentence(const char* sentence) {
    if (sentence == NULL || my_strlen(sentence) < MIN_NMEA_SENTENCE_LENGTH || sentence[0] != '$') {
        uart_printf("[ERROR] Invalid NMEA sentence: %s\n", sentence ? sentence : "NULL");
        return NEO_7M_PARSE_ERROR; // Invalid sentence
    }

    /* Parse the sentence */
    uint8_t field = 0; // Field index

    if (starts_with(sentence, "$GPGGA") || starts_with(sentence, "$GNGGA")) {
        // GGA sentence contains: Time, Latitude, N/S, Longitude, E/W, Fix Quality, Number of Satellites, HDOP, Altitude, etc.
        char lat_dir = 'N';
        char lon_dir = 'E';
        double raw_lat = 0.0;
        double raw_lon = 0.0;
        const char *p = sentence;

        /* Parse the sentence */
        while (*p) {
            if (*p == ',') {
                field++;
                p++;
                continue;
            }
            switch (field) {
                case 2: // Latitude
                    raw_lat = my_atof(p);
                    break;
                case 3: // Latitude direction N/S
                    lat_dir = *p;
                    break;
                case 4: // Longitude
                    raw_lon = my_atof(p);
                    break;
                case 5: // Longitude direction E/W
                    lon_dir = *p;
                    break;
                case 6: // Fix quality, we can use this to determine if the data is valid
                    if (*p == '0') {
                        return NEO_7M_PARSE_ERROR; // No fix, data invalid
                    }
                    break;
                case 7: // Number of satellites, convert to integer
                    g_neo_7m_data.num_satellites = (uint8_t)my_atoi(p);
                    break;
                case 9: // Altitude in meters, convert to double
                    g_neo_7m_data.altitude = my_atof(p);
                    break;
                default:
                    break;
            }
            while (*p && *p != ',') p++; // Move to next field
        }
        // Convert raw NMEA coordinates to decimal degrees
        g_neo_7m_data.latitude = nmea_to_decimal(raw_lat, lat_dir);
        g_neo_7m_data.longitude = nmea_to_decimal(raw_lon, lon_dir);

    } else if (starts_with(sentence, "$GPRMC") || starts_with(sentence, "$GNRMC")) {
        // RMC sentence contains: Time, Status, Latitude, N/S, Longitude, E/W, Speed in knots, Course, Date, etc.
        char status = 'V';
        char lat_dir = 'N';
        char lon_dir = 'E';
        double raw_lat = 0.0;
        double raw_lon = 0.0;
        double speed_knots = 0.0;
        const char *p = sentence;

        /* Parse the sentence */
        while (*p) {
            if (*p == ',') {
                field++;
                p++;
                continue;
            }
            switch (field) {
                case 2: // Status
                    status = *p;
                    if (status != 'A') {
                        uart_printf("[ERROR] Invalid NMEA sentence: %s\n", sentence ? sentence : "NULL");
                        return NEO_7M_PARSE_ERROR; // Data invalid
                    }
                    break;
                case 3: // Latitude
                    raw_lat = my_atof(p);
                    break;
                case 4: // Latitude direction N/S
                    lat_dir = *p;
                    break;
                case 5: // Longitude
                    raw_lon = my_atof(p);
                    break;
                case 6: // Longitude direction E/W
                    lon_dir = *p;
                    break;
                case 7: // Speed in knots, convert to km/h
                    speed_knots = my_atof(p);
                    g_neo_7m_data.speed = speed_knots * SPEED_KNOTS_TO_KMH;
                    break;
                case 8:
                    if (my_strlen(p) > 0) {
                        double course = my_atof(p);
                        // Get direction based on course angle (0-360 degrees)
                        if      (course >= 315 || course < 45)   g_neo_7m_data.direction = 'N';
                        else if (course < 135)                   g_neo_7m_data.direction = 'E';
                        else if (course < 225)                   g_neo_7m_data.direction = 'S';
                        else                                     g_neo_7m_data.direction = 'W';
                    }
                    break;
                default:
                    break;
            }
            while (*p && *p != ',') p++; // Move to next field
        }
    }

    return NEO_7M_OK;
}

// static const char* copy_data_to_line_buffer(void) {
//     uint16_t pos = gps_dma_get_pos();
//     uint16_t start = pos;
//     uint16_t idx = 0;
//     char gps_data_line[GPS_DMA_BUF_SIZE];

//     while (idx < GPS_LINE_BUF - 1) {
//         char c = g_gps_dma_state.gps_dma_buf[start];
//         gps_data_line[idx++] = c;
//         if (c == '\n') {
//             break; // End of line
//         }
//         start = (start + 1) % GPS_DMA_BUF_SIZE; // Wrap around
//     }
//     gps_data_line[idx] = '\0'; // Null-terminate the string
//     uart_printf("[DEBUG] Copied GPS line: %s\n", gps_data_line);
//     return gps_data_line;
// }

static char* copy_data_to_line_buffer(void)
{
    // For debugging: print the entire GPS DMA buffer content
    for (int i = 0; i < GPS_DMA_BUF_SIZE; i++) {
        uart_printf("%c", g_gps_dma_state.gps_dma_buf[i]);
    }
    uart_printf("\n");

    uint16_t current_pos = gps_dma_get_pos();   // Current position in the circular buffer
    char gps_line[GPS_LINE_BUF];         // Buffer to store the copied line

    uint16_t start = current_pos;
    bool found_dollar = false;

    for (int i = 0; i < GPS_DMA_BUF_SIZE; i++) {   // Search backwards for the last '$' character
        if (g_gps_dma_state.gps_dma_buf[start] == '$') {
            found_dollar = true;
            break;
        }
        if (start == 0) start = GPS_DMA_BUF_SIZE - 1;
        else start--;
    }

    if (!found_dollar) {
        gps_line[0] = '\0';
        uart_printf("[ERROR] No valid NMEA sentence start ('$') found in GPS DMA buffer\n");
        return NULL; // No valid start found, return NULL
    }

    // Copy characters from the found '$' position until we reach a newline or the end of the buffer
    uint16_t idx = 0;
    uint16_t p = start;

    while (idx < GPS_LINE_BUF - 1) {
        char c = g_gps_dma_state.gps_dma_buf[p];
        gps_line[idx++] = c;

        if (c == '\n' || c == '\r') {
            break;
        }

        p = (p + 1) % GPS_DMA_BUF_SIZE;

        // If we loop back to the start position, it means we have read the entire buffer without finding a newline, so we stop to avoid an infinite loop
        if (p == start) break;
    }

    gps_line[idx] = '\0';

    // Remove trailing carriage return if present
    if (idx > 0 && gps_line[idx-1] == '\r')
        gps_line[idx-1] = '\0';

    #if NEO_7M_DEBUG_ENABLED
    uart_printf("[DEBUG] Copied GPS line (%u bytes): %s\n", idx, gps_line);
    #endif
    return gps_line;
}

void neo_7m_task(void* params) {
    neo_7m_error_t err = neo_7m_init();
    if (err != NEO_7M_OK) {
        uart_printf("[ERROR] Failed to initialize Neo7m GPS module: %d\n", err);
        // while(1);
    }
    
    while (1) {
        xSemaphoreTake(gps_data_mutex, portMAX_DELAY);
        // const char* gps_data_line = copy_data_to_line_buffer();
        // if (gps_data_line == NULL) {
        //     xSemaphoreGive(gps_data_mutex);
        //     vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before trying again
        //     continue;
        // }
        // parse_nmea_sentence(gps_data_line);
        // #if NEO_7M_DEBUG_ENABLED
        // uart_printf("[GPS] Lat: %f, Lon: %f, Alt: %f m, Speed: %f km/h, Sats: %d, Dir: %c\n",
        //             g_neo_7m_data.latitude, g_neo_7m_data.longitude, g_neo_7m_data.altitude, g_neo_7m_data.speed, g_neo_7m_data.num_satellites, g_neo_7m_data.direction);
        // #endif

        // Print the entire GPS DMA buffer for debugging
        uart_printf("[DEBUG] GPS DMA buffer content:");
        for (int i = 0; i < GPS_DMA_BUF_SIZE; i++) {
            uart_printf("%c", g_gps_dma_state.gps_dma_buf[i]);
        }
        uart_printf("\n");
        xSemaphoreGive(gps_data_mutex);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every 1000ms
    }
}