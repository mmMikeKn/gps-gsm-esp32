#ifndef _GPS_H
#define _GPS_H

#include "esp_types.h"

#define GPS_MAX_SATELLITES_IN_USE (12)
#define GPS_MAX_SATELLITES_IN_VIEW (16)

// UART_NUM_1, 16-RX, 17-TX
// UART_NUM_2, 18-RX, 19-TX
#define GPS_UART_NUM UART_NUM_2
#define GPS_RXD  (18)  
#define GPS_TXD  (19) 
#define TX_GPS_BUF_SIZE (0)
#define RX_GPS_BUF_SIZE (2048)

#define NMEA_PARSER_RUNTIME_BUFFER_SIZE (RX_GPS_BUF_SIZE / 2)

// - Latitude, Longitude, Altitude;
// - Number of satellites in use, fix status (no fix, GPS, DGPS), UTC time;
#define CONFIG_NMEA_STATEMENT_GGA (true)

// - Position/Vertical/Horizontal dilution of precision;
// - Fix mode (no fix, 2D, 3D fix);
// - IDs of satellites in use;
#define CONFIG_NMEA_STATEMENT_GSA (true)

// Number of satellites in view;
// Optional details of each satellite in view;
#define CONFIG_NMEA_STATEMENT_GSV (true)

// - Validity of GPS signal;
//  - Ground speed (knots) and course over ground (degrees);
// - Magnetic variation;
// - UTC date;
#define CONFIG_NMEA_STATEMENT_RMC (true)

// - Latitude, Longitude;
// - UTC time;
#define CONFIG_NMEA_STATEMENT_GLL (true)

// - Ground speed (knots, km/h) and course over ground (degrees);
// - Magnetic variation;
#define CONFIG_NMEA_STATEMENT_VTG (true)


/**
 * @brief GPS fix type
 *
 */
typedef enum {
    GPS_FIX_INVALID, /*!< Not fixed */
    GPS_FIX_GPS,     /*!< GPS */
    GPS_FIX_DGPS,    /*!< Differential GPS */
} gps_fix_t;

/**
 * @brief GPS fix mode
 *
 */
typedef enum {
    GPS_MODE_INVALID = 1, /*!< Not fixed */
    GPS_MODE_2D,          /*!< 2D GPS */
    GPS_MODE_3D           /*!< 3D GPS */
} gps_fix_mode_t;

/**
 * @brief GPS satellite information
 *
 */
typedef struct {
    uint8_t num;       /*!< Satellite number */
    uint8_t elevation; /*!< Satellite elevation */
    uint16_t azimuth;  /*!< Satellite azimuth */
    uint8_t snr;       /*!< Satellite signal noise ratio */
} gps_satellite_t;

/**
 * @brief GPS time
 *
 */
typedef struct {
    uint8_t hour;      /*!< Hour */
    uint8_t minute;    /*!< Minute */
    uint8_t second;    /*!< Second */
    uint16_t thousand; /*!< Thousand */
} gps_time_t;


/**
 * @brief GPS date
 *
 */
typedef struct {
    uint8_t day;   /*!< Day (start from 1) */
    uint8_t month; /*!< Month (start from 1) */
    uint16_t year; /*!< Year (start from 2000) */
} gps_date_t;

/**
 * @brief NMEA Statement
 *
 */
typedef enum {
    STATEMENT_UNKNOWN = 0, /*!< Unknown statement */
    STATEMENT_GGA,         /*!< GGA */
    STATEMENT_GSA,         /*!< GSA */
    STATEMENT_RMC,         /*!< RMC */
    STATEMENT_GSV,         /*!< GSV */
    STATEMENT_GLL,         /*!< GLL */
    STATEMENT_VTG          /*!< VTG */
} nmea_statement_t;

/**
 * @brief GPS object
 *
 */
#define GPS_LAST_VALID_MAX 5
typedef struct {
    float latitude;                                                /*!< Latitude (degrees) */
    float longitude;                                               /*!< Longitude (degrees) */
    float altitude;                                                /*!< Altitude (meters) */
    gps_time_t tim;                                                /*!< time in UTC */
    gps_date_t date;                                               /*!< Fix date */
    float speed;                                                   /*!< Ground speed, unit: m/s */
} gps_crd_t;

typedef struct {
    gps_crd_t cur_crd;
    gps_crd_t last_valid_crd[GPS_LAST_VALID_MAX];
    int last_valid_crd_num;

    gps_fix_t fix;                                                 /*!< Fix status */
    uint8_t sats_in_use;                                           /*!< Number of satellites in use */
    gps_fix_mode_t fix_mode;                                       /*!< Fix mode */
    uint8_t sats_id_in_use[GPS_MAX_SATELLITES_IN_USE];             /*!< ID list of satellite in use */
    float dop_h;                                                   /*!< Horizontal dilution of precision */
    float dop_p;                                                   /*!< Position dilution of precision  */
    float dop_v;                                                   /*!< Vertical dilution of precision  */
    uint8_t sats_in_view;                                          /*!< Number of satellites in view */
    gps_satellite_t sats_desc_in_view[GPS_MAX_SATELLITES_IN_VIEW]; /*!< Information of satellites in view */
    bool valid;                                                    /*!< GPS validity */
    float cog;                                                     /*!< Course over ground */
    float variation;                                               /*!< Magnetic variation */
} gps_state_t;

//===============================
extern gps_state_t gps_state;
extern void start_gps();

#endif
