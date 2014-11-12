//
// Dallas DS3234 SPI Real Time Clock Driver
// Copyright (c) 2008 by Daniel Corley
//

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "ds3234.h"
#include "NumiCore.h"
#include "delay_x.h"

/**************************************************************************************************************/
//
// DS3234 Private Variables
//
/**************************************************************************************************************/
static bool timeValid = false;

/**************************************************************************************************************/
//
// DS3234 Global Variables
//
/**************************************************************************************************************/
volatile ds3234timeanddate_t timeAndDate;

/**************************************************************************************************************/
//
// DS3234 Status Indication Functions
//
/**************************************************************************************************************/
bool isTimeValid (void)
{
    return timeValid;
}

/**************************************************************************************************************/
//
// DS3234 Register Access Functions
//
/**************************************************************************************************************/

void rtcSelect(bool select)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (select)
        {
            // Set SS_RTC pin.
            PORTC |= _BV(PC3);
        }
        else
        {
            // Clear SS_RTC pin.
            PORTC &= ~(_BV(PC3));
        }
    }
}

// Write specified number of bytes from the data source to the specified device address.
void rtcWrite (uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t i = 0;
    
    // First, select the device.
    rtcSelect(0);

    // Set the SPI interface for MSB-first operation. (Clear the DORD bit)
    SPCR &= ~(_BV(DORD));

    // Need at least 400 ns delay before clocking any data.
    _delay_ns(400);

    // Send the address of the register to be written to.
    // Note, the RTC auto-increments the address pointer for multi-byte writes.
    spiTranscieve(addr | RTC_WRITE_FLAG);

    // Send all the data to be written.
    for (i = 0; i < len; i++)
    {
        spiTranscieve(data[i]);
    }

    // Need at least 100 ns delay before de-selecting CS line.
    _delay_ns(100);

    // Now de-select the device.
    rtcSelect(1);
}

// Read specified number of bytes into the data location from the specified device address.
void rtcRead (uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t i = 0;
    
    // First, select the device.
    rtcSelect(0);

    // Set the SPI interface for MSB-first operation. (Clear the DORD bit)
    SPCR &= ~(_BV(DORD));

    // Need at least 400 ns delay before clocking any data.
    _delay_ns(400);

    // Send the address of the register to be read.
    // Note, the RTC auto-increments the address pointer for multi-byte reads.
    spiTranscieve(addr);

    // Now clock out dummy data and store what comes back.
    for (i = 0; i < len; i++)
    {
        data[i] = spiTranscieve(0);
    }

    // Need at least 100 ns delay before de-selecting CS line.
    _delay_ns(100);

    // Now de-select the device.
    rtcSelect(1);
}

/**************************************************************************************************************/
//
// DS3234 Initialization Functions
//
/**************************************************************************************************************/

void initRtc (void)
{
    // Check the Oscillator Stop Flag to see if power has been lost. 
    // If so, set registers according to how we want it configured.
    ds3234status_t status = { 0 };

    rtcRead(RTC_STATUS_ADDR, (uint8_t *)&status, 1);

    if (status.osf)
    {  
        // Oscillator has stopped some time in the past. That means the RTC needs to be reconfigured, 
        // and the current time is invalid.
        ds3234Reg_t rtc;
        memset(&rtc, 0, sizeof(rtc));

        timeValid = false;

        // Clear the OSF flag, disable 32KHz output in Vcc and battery mode.
        rtc.status.a1f      = 0;        // Clear the alarm flags
        rtc.status.a2f      = 0;        // 
        rtc.status.bsy      = 0;        // Clear the temp conversion BUSY flag
        rtc.status.en32khz  = 0;        // Disable the 32 KHz output pin when Vcc is present
        rtc.status.crate    = 0;        // Sample temperature every 64 seconds
        rtc.status.bb32khz  = 0;        // Disable the 32 KHz output pin when Vcc is absent
        rtc.status.osf      = 0;        // Clear the OSF flag
        
        // Enable the oscillator, set Square Wave output to 1 Hz, disable everything else.
        rtc.control.a1ie    = 0;        // Disable alarm interrupts
        rtc.control.a2ie    = 0;        // 
        rtc.control.intcn   = 0;        // Square wave output on INT/SQW pin.
        rtc.control.rs      = 0;        // 1 Hz square wave output
        rtc.control.conv    = 0;        // Don't do a temp convert
        rtc.control.bbsqw   = 0;        // Don't generate square wave in battery-backed mode
        rtc.control.eosc    = 0;        // Start the oscillator

        // Set the Default time to 12:00:00 AM, Tuesday, 07/01/2008
        rtc.seconds.seconds1    = 0;
        rtc.seconds.seconds10   = 0;
        rtc.minutes.minutes1    = 0;
        rtc.minutes.minutes10   = 0;
        rtc.hours.hours1        = 2;
        rtc.hours.hours10       = 1;    // 12 (AM)
        rtc.hours.mode12        = 1;    // 12-hour mode.
        rtc.day.day             = 2;    // Tuesday
        rtc.date.date1          = 1;    // 1st
        rtc.date.date10         = 0;
        rtc.month.month1        = 7;    // July
        rtc.month.month10       = 0;
        rtc.month.century       = 0;
        rtc.year.year1          = 8;    // 2008
        rtc.year.year10         = 0;

        // Set alarms to 12-hour mode.
        rtc.alm1_hours.mode12 = 1;
        rtc.alm2_hours.mode12 = 1;
        
        // Write the data into the lowest 16 bytes of the RTC register space. 
        // (We don't need to write to registers any higher than that.)
        rtcWrite(RTC_BASE_ADDR, (uint8_t *)&rtc, RTC_FULL_WRITE_LEN);
    }
    else
    {
        // Otherwise, indicate the time is valid. (normal startup routines will read the time)
        // Also be sure to re-set the main control and status bits in case anything fishy
        // has happened. Just leave the time registers alone.

        // In our case, writing a zero to each register will suffice. (see above)
        uint8_t data = 0;

        rtcWrite(RTC_STATUS_ADDR, (uint8_t *)&data, 1);
        rtcWrite(RTC_CONTROL_ADDR, (uint8_t *)&data, 1);        
        
        timeValid = true;
    }

    // Initialize the time & data global with what is currently in the RTC.
    rtcRead(RTC_BASE_ADDR, (uint8_t *)&timeAndDate, sizeof(timeAndDate));
}

/**************************************************************************************************************/
//
// DS3234 Utility Functions
//
/**************************************************************************************************************/

void showDateTime (void)
{   
    ds3234timeanddate_t timeAndDate;
    uint8_t monthIndex;

    static PROGMEM const char months[12][4] =
    {
        { "Jan" }, { "Feb" }, { "Mar" }, { "Apr" },
        { "May" }, { "Jun" }, { "Jul" }, { "Aug" },
        { "Sep" }, { "Oct" }, { "Nov" }, { "Dec" }
    };

    char month[ 4 ];

    // Read the time from the RTC into the local data structure.
    rtcRead(RTC_BASE_ADDR, (uint8_t *)&timeAndDate, sizeof(timeAndDate));

    // display time
    if ( timeAndDate.hours.mode12 == 0 )
    {
        // 24-hour mode
        printf_P(
            PSTR( "%d%d:%d%d:%d%d" ),
            timeAndDate.hours.hours10, timeAndDate.hours.hours1,
            timeAndDate.minutes.minutes10, timeAndDate.minutes.minutes1,
            timeAndDate.seconds.seconds10, timeAndDate.seconds.seconds1
        );
    }
    else
    {
        // 12-hour mode
        printf_P(
            PSTR( "%d%d:%d%d:%d%d %cM" ),
            timeAndDate.hours.hours10 & 0x01, timeAndDate.hours.hours1,
            timeAndDate.minutes.minutes10, timeAndDate.minutes.minutes1,
            timeAndDate.seconds.seconds10, timeAndDate.seconds.seconds1,
            ( ( timeAndDate.hours.hours10 & 0x2 ) == 0 ) ? 'A' : 'P'
        );
    } // end if

    // display date and day
    monthIndex = timeAndDate.month.month1 + ( ( timeAndDate.month.month10 == 0 ) ? 0 : 10 );

    if ( ( monthIndex >= 1 ) && ( monthIndex <= 12 ) )
    {
        strcpy_P( month, months[monthIndex-1] );
    }
    else
    {
        strcpy_P( month, PSTR( "???" ) );
    } // end if
    
    printf_P(
        PSTR( "   %d%d %s %d%d   Day %d\r\n" ),
        timeAndDate.date.date10, timeAndDate.date.date1,
        month,
        timeAndDate.year.year10, timeAndDate.year.year1,
        timeAndDate.day.day
    );    
} 

void setTime (char *command)
{
    PGM_P   usage = PSTR( "Usage: thh:mm:ss or thh:mm:ssa or thh:mm:ssp\r\n" );

    uint8_t hours[2], minutes[2], seconds[2];
    bool    mode24, pm;
    ds3234time_t time;    

    if ( ( ( strlen( command ) != 9 ) && ( strlen( command ) != 10 ) ) ||
        ( command[ 3 ] != ':' ) || ( command[ 6 ] != ':' ) )
    {
        puts_P( usage );
        return;
    } // end if

    mode24 = (strlen( command ) == 9);

    pm = false;
    if ( !mode24 )
    {
        // 12-hour mode, check for AM or PM
        if ( ( command[ 9 ] == 'p' ) || ( command[ 9 ] == 'P' ) )
        {
            pm = true;
        }
        else if ( ( command[ 9 ] != 'a' ) && ( command[ 9 ] != 'A' ) )
        {
            puts_P( usage );
            return;
        } // end if
    } // end if

    // Convert from ASCII to a number.
    hours[ 0 ] = command[ 1 ] - '0';
    hours[ 1 ] = command[ 2 ] - '0';

    minutes[ 0 ] = command[ 4 ] - '0';
    minutes[ 1 ] = command[ 5 ] - '0';

    seconds[ 0 ] = command[ 7 ] - '0';
    seconds[ 1 ] = command[ 8 ] - '0';

    // Make sure time given by user is valid.
    if ( ( hours[ 0 ] > ( mode24 ? 2 : 1 ) ) || ( hours[ 1 ] > 9 ) ||
        ( minutes[ 0 ] > 5 ) || ( minutes[ 1 ] > 9 ) ||
        ( seconds[ 0 ] > 5 ) || ( seconds[ 1 ] > 9 ) ||
        ( ( 10 * hours[ 0 ] + hours[ 1 ] ) > ( mode24 ? 23 : 12 ) ) ||
        ( !mode24 && ( hours[ 0 ] == 0 ) && ( hours[ 1 ] == 0 ) ) )
    {
        puts_P( usage );
        return;
    } // end if

    // Write time out to RTC chip.
    time.seconds.seconds1   = seconds[1];
    time.seconds.seconds10  = seconds[0];

    time.minutes.minutes1   = minutes[1];
    time.minutes.minutes10  = minutes[0];

    time.hours.hours1       = hours[1];
    time.hours.hours10      = mode24 ? hours[0] : (pm ? (hours[0] | 2) : hours[0]);
    time.hours.mode12       = mode24 ? 0 : 1;   

    rtcWrite(RTC_BASE_ADDR, (uint8_t *)&time, sizeof(time));
} // end setTime

void setDate (char *command)
{
    PGM_P   usage = PSTR( "Usage: dyy/mm/dd\r\n" );
    uint8_t years[2], months[2], days[2];
    ds3234fulldate_t date;

    if ( ( strlen( command ) != 9 ) ||
        ( command[ 3 ] != '/' ) || ( command[ 6 ] != '/' ) )
    {
        puts_P( usage );
        return;
    } // end if

    // Convert from ASCII to numbers.
    years[ 0 ] = command[ 1 ] - '0';
    years[ 1 ] = command[ 2 ] - '0';

    months[ 0 ] = command[ 4 ] - '0';
    months[ 1 ] = command[ 5 ] - '0';

    days[ 0 ] = command[ 7 ] - '0';
    days[ 1 ] = command[ 8 ] - '0';

    // Check the numbers for consistency & range.
    if ( (years[0] > 9) || (years[1] > 9) ||
        (months[0] > 1 ) ||
        ((months[0] == 0) && (months[1] > 9)) ||
        ((months[0] == 1 ) && (months[1] > 2)) ||
        (days[0] > 3) || (days[1] > 9))
    {
        puts_P( usage );
        return;
    } // end if

    // Check day-of-month for consistency.
    // Note: this code assumes any year could be a leap year.
    switch ( months[1] + ((months[0] == 0) ? 0 : 10) )
    {
        case 2:     // February
            // Don't allow days greater than 29.
            if ((days[0] == 3) || ((days[0] == 2) && (days[1] > 9)))
            {
                puts_P( usage );
                return;
            } // end if
        break;

        case 4:     // April
        case 6:     // June
        case 9:     // September
        case 11:    // November
            // Don't allow days greater than 30.
            if ( (days[0] == 3) && (days[ 1 ] > 0) )
            {
                puts_P( usage );
                return;
            } // end if
        break;

        default:
            // Otherwise don't allow days greater than 31.
            if ( (days[0] == 3) && (days[1] > 1) )
            {
                puts_P( usage );
                return;
            } // end if
        break;
    }

    // Write the date out to the RTC.
    date.date.date1     = days[1];
    date.date.date10    = days[0];
    date.month.month1   = months[1];
    date.month.month10  = months[0];
    date.year.year1     = years[1];
    date.year.year10    = years[0];

    rtcWrite(RTC_DATE_ADDR, (uint8_t *)&date, sizeof(date));
} // end setDate

void setDay (char *command)
{
    PGM_P   usage = PSTR( "Usage: yd\r\n" );
    uint8_t day;
    ds3234day_t weekday;

    if ( strlen(command) != 2 )
    {
        puts_P(usage);
        return;
    } // end if

    // Convert from ASCII to a number.
    day = command[1] - '0';

    // Range check
    if ( ( day < 1 ) || ( day > 7 ) )
    {
        puts_P( usage );
        return;
    } // end if

    // Write the day-of-the-week out to the RTC.
    weekday.day = day;

    rtcWrite(RTC_DAY_ADDR, (uint8_t *)&weekday, sizeof(weekday));
} // end setDay

void showTemperature (void)
{
    uint8_t lsb, msb;
    int32_t temperature;
    
    // Read the temperature (in Celsius, 0.25 degree resolution):
    rtcRead(RTC_TEMP_MSB_ADDR, &msb, 1);
    rtcRead(RTC_TEMP_LSB_ADDR, &lsb, 1);

    temperature = 256*(int8_t)msb + lsb;

    // Do a numeric conversion.
    temperature = (25*temperature)/64;

    printf_P( PSTR("Temp: %d.%02d C\r\n"), (temperature/100), (abs(temperature) % 100));
}

// This function increments the hours register in the RTC and accounts for roll-over appropriately.
void incrementHours (void)
{
    ds3234hours_t hourData;
    uint8_t hour = 0;

    // Get the current value.
    rtcRead(RTC_HOURS_ADDR, (uint8_t *)&hourData, 1);

    // Account for roll over.
    if (hourData.mode12)
    {
        // Account for AM/PM
        bool pmBit = (hourData.hours10) & 0x02;

        // Get the decimal hour value. Mask off AM/PM bit.
        hour = (((hourData.hours10) & 0x01) ? 10 : 0) + hourData.hours1;

        // Increment
        hour++;

        // Test for roll-over.
        if (hour > 12)
        {
            // Roll hours over to 1.
            hourData.hours1 = 1;
            hourData.hours10 = 0;

            // Toggle AM/PM indicator
            hourData.hours10 |= pmBit ? 0 : 0x02;                
        }
        else
        {
            // Otherwise, write the incremented hour value back into hourData.
            hourData.hours1 = hour % 10;
            hourData.hours10 = (pmBit ? ((hour/10) | 2) : (hour/10));            
        }
    }
    else
    {
        // Get the decimal hour value.
        hour = 10*hourData.hours10 + hourData.hours1;

        // Increment
        hour++;
        
        if (hour > 23)
        {            
            // Roll the hours over to 0.
            hourData.hours1 = 0;
            hourData.hours10 = 0;
        }
    }

    // Write the register back into the RTC.
    rtcWrite(RTC_HOURS_ADDR, (uint8_t *)&hourData, 1);
}

// This function increments the minutes register in the RTC and accounts for roll-over appropriately.
void incrementMinutes (void)
{
    ds3234minutes_t minuteData;
    uint8_t minutes = 0;

    // Get the current value.
    rtcRead(RTC_MINUTES_ADDR, (uint8_t *)&minuteData, 1);

    // Calculate the decimal value.
    minutes = 10*minuteData.minutes10 + minuteData.minutes1;

    // Increment the minutes.
    minutes++;

    // Range check.
    if (minutes > 59)
    {
        minutes = 0;
    }

    // Recode
    minuteData.minutes1 = minutes % 10;
    minuteData.minutes10 = minutes / 10;

    // Write the register back into the RTC.
    rtcWrite(RTC_MINUTES_ADDR, (uint8_t *)&minuteData, 1);
}

// This function increments the seconds register in the RTC and accounts for roll-over appropriately.
void incrementSeconds (void)
{
    ds3234seconds_t secondData;
    uint8_t seconds = 0;

    // Get the current value.
    rtcRead(RTC_SECONDS_ADDR, (uint8_t *)&secondData, 1);

    // Calculate the decimal value.
    seconds = 10*secondData.seconds10 + secondData.seconds1;

    // Increment the seconds.
    seconds++;

    // Range check.
    if (seconds > 59)
    {
        seconds = 0;
    }

    // Recode
    secondData.seconds1 = seconds % 10;
    secondData.seconds10 = seconds / 10;

    // Write the register back into the RTC.
    rtcWrite(RTC_SECONDS_ADDR, (uint8_t *)&secondData, 1);
}

// This function increments the month register in the RTC and accounts for roll-over appropriately.
void incrementMonths (void)
{
    ds3234month_t monthData;
    uint8_t month = 0;

    // Get the current value.
    rtcRead(RTC_MONTH_ADDR, (uint8_t *)&monthData, 1);

    // Calculate the decimal value.
    month = 10*monthData.month10 + monthData.month1;

    // Increment the month.
    month++;

    // Range check.
    if (month > 12)
    {
        month = 1;
    }

    // Recode
    monthData.month1 = month % 10;
    monthData.month10 = month / 10;

    // Write the register back into the RTC.
    rtcWrite(RTC_MONTH_ADDR, (uint8_t *)&monthData, 1);    
}

// This function increments the month register in the RTC and accounts for roll-over appropriately. (per month)
void incrementDays (void)
{
    ds3234date_t dateData;
    ds3234month_t monthData;
    uint8_t date = 0;
    uint8_t month = 0;

    // Get the current value(s).
    rtcRead(RTC_DATE_ADDR, (uint8_t *)&dateData, 1);
    rtcRead(RTC_MONTH_ADDR, (uint8_t *)&monthData, 1);

    // Increment the date based on the current month. 
    // Simply assume any year could be a leap year.
    date = 10*dateData.date10 + dateData.date1;
    month = 10*monthData.month10 + monthData.month1;

    // Increment date.
    date++;

    // Check day-of-month for consistency.
    // Note: this code assumes any year could be a leap year.
    switch ( month )
    {
        case 2:     // February
            // Don't allow days greater than 29.
            if (date > 29)
            {
                date = 1;
            }
        break;
 
        case 4:     // April
        case 6:     // June
        case 9:     // September
        case 11:    // November
            // Don't allow days greater than 30.
            if (date > 30)
            {
                date = 1;
            }
        break;
 
        default:
            // Otherwise don't allow days greater than 31.
            if (date > 31)
            {
                date = 1;
            }
        break;
    }

    // Recode the date into the data byte.
    dateData.date1 = date % 10;
    dateData.date10 = date / 10;

    // Write the register back into the RTC.
    rtcWrite(RTC_DATE_ADDR, (uint8_t *)&dateData, 1);    
}

// This function increments the year register in the RTC and accounts for roll-over appropriately.
void incrementYears (void)
{
    ds3234year_t yearData;
    uint8_t year = 0;

    // Get the current value.
    rtcRead(RTC_YEAR_ADDR, (uint8_t *)&yearData, 1);

    // Calculate the decimal value.
    year = 10*yearData.year10 + yearData.year1;

    // Increment
    year++;

    // Range check
    if (year > 99)
    {
        year = 0;
    }

    // Recode
    yearData.year1 = year % 10;
    yearData.year10 = year / 10;

    // Write the register back into the RTC.
    rtcWrite(RTC_YEAR_ADDR, (uint8_t *)&yearData, 1);    
}

