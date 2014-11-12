//
// Dallas DS3234 SPI Real Time Clock Driver
// Copyright (c) 2008 by Daniel Corley
//

/**************************************************************************************************************/
// Address Map & Bit definitions
/**************************************************************************************************************/

// RTC Registers and Bit Masks
#define RTC_WRITE_FLAG              0x80

#define RTC_BASE_ADDR               0x00

// Seconds (00-59)
#define RTC_SECONDS_ADDR            0x00

// Minutes (00-59)
#define RTC_MINUTES_ADDR            0x01

// Hours (1-12 + !AM/PM -or- 00-23)
#define RTC_HOURS_ADDR              0x02

// Day of the Week (1-7)
#define RTC_DAY_ADDR                0x03

// Day of the Month (01-31)
#define RTC_DATE_ADDR               0x04

// Month/Century (01-12 + Century)
#define RTC_MONTH_ADDR              0x05

// Year (00-99)
#define RTC_YEAR_ADDR               0x06

// Alarm 1 Seconds (00-59)
#define RTC_ALM1_SEC_ADDR           0x07

// Alarm 1 Minutes (00-59)
#define RTC_ALM1_MIN_ADDR           0x08

// Alarms 1 Hours (1-12 + !AM/PM -or- 00-23)
#define RTC_ALM1_HOURS_ADDR         0x09

// Alarm 1 Day/Date (1-7/01-31)
#define RTC_ALM1_DAYDATE_ADDR       0x0A

// Alarm 2 Minutes (00-59)
#define RTC_ALM2_MIN_ADDR           0x0B

// Alarms 1 Hours (1-12 + !AM/PM -or- 00-23)
#define RTC_ALM2_HOURS_ADDR         0x0C

// Alarm 2 Day/Date (1-7/01-31)
#define RTC_ALM2_DAYDATE_ADDR       0x0D

// Control
#define RTC_CONTROL_ADDR            0x0E

// Control / Status
#define RTC_STATUS_ADDR             0x0F

// Crystal Aging Offset (MSB is sign bit)
#define RTC_XTALAGE_ADDR            0x10

// Temperature MSB (Read Only) (MSB is sign bit)
#define RTC_TEMP_MSB_ADDR           0x11

// Temperature LSB (Read Only)
#define RTC_TEMP_LSB_ADDR           0x12

// Disable Temp Conversions
#define RTC_DTC_ADDR                0x13

// SRAM Address Register
#define RTC_SRAM_ADDR               0x18

// SRAM Data Register
#define RTC_SRAM_DATA               0x19

/**************************************************************************************************************/
// Memory Structures
// Note: in the following structure declarations, bitfields are assigned starting
// from the low-order bit.  Some compilers assign bitfields starting from the
// high-order bit.
/**************************************************************************************************************/

typedef struct 
{
    // Seconds
    uint8_t seconds1  : 4;
    uint8_t seconds10 : 3;
    uint8_t           : 1;
} ds3234seconds_t;

typedef struct 
{
    // Minutes
    uint8_t minutes1  : 4;
    uint8_t minutes10 : 3;
    uint8_t           : 1;
} ds3234minutes_t;

typedef struct 
{
    // Hours
    uint8_t hours1    : 4; 
    uint8_t hours10   : 2;  // In 12-hour mode, MSB of this field indicates 0 = AM, 1 = PM
    uint8_t mode12    : 1;  // 12-hour mode (1) or 24-hour mode (0)
    uint8_t           : 1;
} ds3234hours_t;

typedef struct 
{
    // Day of the week
    uint8_t day       : 3;
    uint8_t           : 5;
} ds3234day_t;

typedef struct 
{
    // Day of the month
    uint8_t date1     : 4;
    uint8_t date10    : 2;
    uint8_t           : 2;
} ds3234date_t;

typedef struct 
{
    // Month & Century
    uint8_t month1    : 4;
    uint8_t month10   : 1;
    uint8_t           : 2;
    uint8_t century   : 1;
} ds3234month_t;

typedef struct 
{
    // Year
    uint8_t year1     : 4;
    uint8_t year10    : 4;
} ds3234year_t;

typedef struct 
{
    // Alarm Seconds (Alarm 1 only)
    uint8_t seconds1  : 4;
    uint8_t seconds10 : 3;
    uint8_t m1        : 1;
} ds3234almsec_t;

typedef struct 
{
    // Alarm Minutes
    uint8_t minutes1  : 4;
    uint8_t minutes10 : 3;
    uint8_t m2        : 1;
} ds3234almmin_t;

typedef struct 
{
    // Alarm Hours
    uint8_t hours1    : 4; 
    uint8_t hours10   : 2;  // In 12-hour mode, MSB of this field indicates 0 = AM, 1 = PM
    uint8_t mode12    : 1;  // 12-hour mode (1) or 24-hour mode (0)
    uint8_t m3        : 1;
} ds3234almhrs_t;

typedef struct 
{
    // Alarm Day/Date
    uint8_t date1     : 4;
    uint8_t date10    : 2;
    uint8_t daydate   : 1;  // Date (0) or Day (1)
    uint8_t m4        : 1;
} ds3234almday_t;

typedef struct 
{
    // Control
    uint8_t a1ie      : 1;
    uint8_t a2ie      : 1;    
    uint8_t intcn     : 1;    
    uint8_t rs        : 2;
    uint8_t conv      : 1;
    uint8_t bbsqw     : 1;
    uint8_t eosc      : 1;  // Active low
} ds3234control_t;

typedef struct 
{
    // Control / Status
    uint8_t a1f       : 1;
    uint8_t a2f       : 1;    
    uint8_t bsy       : 1;    
    uint8_t en32khz   : 1;
    uint8_t crate     : 2;
    uint8_t bb32khz   : 1;
    uint8_t osf       : 1;
} ds3234status_t;

typedef struct 
{
    // Disable Temp Conversions
    uint8_t bb_td     : 1;
    uint8_t           : 7;    
} ds3234dtc_t;

// The Time Portion of the DS3234 memory map
typedef struct 
{
    // Current Time
    ds3234seconds_t     seconds;                // 00h
    ds3234minutes_t     minutes;                // 01h
    ds3234hours_t       hours;                  // 02h
} ds3234time_t;

// The Date Portion of the DS3234 memory map (day is omitted)
typedef struct 
{
    // Current Date
    ds3234date_t        date;                   // 04h
    ds3234month_t       month;                  // 05h
    ds3234year_t        year;                   // 06h
} ds3234fulldate_t;

// The Time & Date Portion of the DS3234 memory map
typedef struct 
{
    // Current Time
    ds3234seconds_t     seconds;                // 00h
    ds3234minutes_t     minutes;                // 01h
    ds3234hours_t       hours;                  // 02h
    ds3234day_t         day;                    // 03h
    ds3234date_t        date;                   // 04h
    ds3234month_t       month;                  // 05h
    ds3234year_t        year;                   // 06h
} ds3234timeanddate_t;

#define RTC_FULL_WRITE_LEN      16      // Size of RTC's writeable lower register space. Address pointer will wrap back to 00h once it reaches 13h.
#define RTC_FULL_READ_LEN       20

// The entire DS3234 memory map
typedef struct 
{
    // Current Time
    ds3234seconds_t     seconds;                // 00h
    ds3234minutes_t     minutes;                // 01h
    ds3234hours_t       hours;                  // 02h
    ds3234day_t         day;                    // 03h
    ds3234date_t        date;                   // 04h
    ds3234month_t       month;                  // 05h
    ds3234year_t        year;                   // 06h

    // Alarm 1
    ds3234almsec_t      alm1_seconds;           // 07h
    ds3234almmin_t      alm1_minutes;           // 08h
    ds3234almhrs_t      alm1_hours;             // 09h
    ds3234almday_t      alm1_day;               // 0Ah

    // Alarm 2
    ds3234almmin_t      alm2_minutes;           // 0Bh
    ds3234almhrs_t      alm2_hours;             // 0Ch
    ds3234almday_t      alm2_day;               // 0Dh

    // Control & Status
    ds3234control_t     control;                // 0Eh
    ds3234status_t      status;                 // 0Fh

    // Crystal and Temperature Data
    int8_t              xtal_age_offset;        // 10h
    int8_t              temp_msb;               // 11h
    uint8_t             temp_lsb;               // 12h
    ds3234dtc_t         dtc;                    // 13h

    // Reserved
    uint8_t             reserved1;              // 14h
    uint8_t             reserved2;              // 15h
    uint8_t             reserved3;              // 16h
    uint8_t             reserved4;              // 17h

    // SRAM Address & Data Registers
    uint8_t             sram_addr;              // 18h
    uint8_t             sram_data;              // 19h
} ds3234Reg_t;


/**************************************************************************************************************/
//
// Access Function Prototypes
//
/**************************************************************************************************************/
// Status
bool isTimeValid (void);

// RTC I/O
void rtcSelect(bool select);
void rtcWrite (uint8_t addr, uint8_t *data, uint8_t len);
void rtcRead (uint8_t addr, uint8_t *data, uint8_t len);

// Initialization
void initRtc (void);

// User Functions
void showDateTime (void);
void setTime (char *command);
void setDate (char *command);
void setDay (char *command);

void showTemperature (void);

// Set Functions
void incrementHours (void);
void incrementMinutes (void);
void incrementSeconds (void);

void incrementMonths (void);
void incrementDays (void);
void incrementYears (void);

