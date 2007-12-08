/** 
\file    test_time_conversion.c
\brief   unit tests for time_conversion.c/.h
\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
\since   2007-11-26

\b "LICENSE INFORMATION" \n
Copyright (c) 2007, refer to 'author' doxygen tags \n
All rights reserved. \n

Redistribution and use in source and binary forms, with or without
modification, are permitted provided the following conditions are met: \n

- Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer. \n
- Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution. \n
- The name(s) of the contributor(s) may not be used to endorse or promote 
  products derived from this software without specific prior written 
  permission. \n

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS ``AS IS'' AND ANY EXPRESS 
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.
*/
#include <stdio.h>
#include "Basic.h"     // CUnit/Basic.h
#include "time_conversion.h"
#include "constants.h"


int init_suite_TIMECONV(void)
{
  return 0;
}

int clean_suite_TIMECONV(void)
{
  return 0;
}


void test_TIMECONV_GetJulianDateFromGPSTime(void)
{
  unsigned short gpsWeek;
  double gpsTow;
  unsigned char utcOffset;
  double julianDate;
  BOOL result;

  const double julian_date_start_of_gps_time = (2444244.5);  // [days]

  gpsWeek = 0;
  gpsTow = 0.0;
  utcOffset = 0;
  result = TIMECONV_GetJulianDateFromGPSTime(
    gpsWeek,
    gpsTow,
    utcOffset,
    &julianDate );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( julianDate, julian_date_start_of_gps_time, 0.0001 );

  gpsWeek = 1;
  gpsTow = 0.0;
  utcOffset = 0;
  result = TIMECONV_GetJulianDateFromGPSTime(
    gpsWeek,
    gpsTow,
    utcOffset,
    &julianDate );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( julianDate - 7, julian_date_start_of_gps_time, 0.0001 );

  gpsWeek = 1024;
  gpsTow = 302400.0;
  utcOffset = 0;
  result = TIMECONV_GetJulianDateFromGPSTime(
    gpsWeek,
    gpsTow,
    utcOffset,
    &julianDate );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( julianDate - 7*1024 - 3.5, julian_date_start_of_gps_time, 0.0001 );
}


void test_TIMECONV_GetJulianDateFromUTCTime(void)
{
  unsigned short utc_year;      // Universal Time Coordinated  [year]
  unsigned char  utc_month;     // Universal Time Coordinated  [1-12 months] 
  unsigned char  utc_day;       // Universal Time Coordinated  [1-31 days]
  unsigned char  utc_hour;      // Universal Time Coordinated  [hours]
  unsigned char  utc_minute;    // Universal Time Coordinated  [minutes]
  float          utc_seconds;   // Universal Time Coordinated  [s]
  double         julian_date;   // Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
  BOOL result;

  const double julian_date_start_of_gps_time = (2444244.5);  // [days]

  utc_year = 1980;
  utc_month = 1;
  utc_day = 6;
  utc_hour = 0;
  utc_minute = 0;
  utc_seconds = 0.0;
  
  result = TIMECONV_GetJulianDateFromUTCTime(
    utc_year,
    utc_month,
    utc_day,
    utc_hour,
    utc_minute,
    utc_seconds,
    &julian_date );
  CU_ASSERT_FATAL( result );

  CU_ASSERT_DOUBLE_EQUAL( julian_date, julian_date_start_of_gps_time, 0.0001 );

  utc_year = 1981;
  result = TIMECONV_GetJulianDateFromUTCTime(
    utc_year,
    utc_month,
    utc_day,
    utc_hour,
    utc_minute,
    utc_seconds,
    &julian_date );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( julian_date - 366, julian_date_start_of_gps_time, 0.0001 ); // leap year
}

void test_TIMECONV_GetGPSTimeFromJulianDate(void)
{
  double          julian_date; // Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
  unsigned char   utc_offset;  // Integer seconds that GPS is ahead of UTC time, always positive [s]
  unsigned short  gps_week;    // GPS week (0-1024+)            [week]
  double          gps_tow;     // GPS time of week [s]
  BOOL result;

  const double julian_date_start_of_gps_time = (2444244.5);  // [days]
 
  julian_date = julian_date_start_of_gps_time;
  utc_offset = 0;
  result = TIMECONV_GetGPSTimeFromJulianDate(
    julian_date,
    utc_offset,
    &gps_week,
    &gps_tow );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( gps_week == 0 );
  CU_ASSERT_DOUBLE_EQUAL( gps_tow, 0.0, 1e-08 );

  julian_date = julian_date_start_of_gps_time + 7*1024 + 3.5;
  utc_offset = 13;
  result = TIMECONV_GetGPSTimeFromJulianDate(
    julian_date,
    utc_offset,
    &gps_week,
    &gps_tow );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( gps_week == 1024 );
  CU_ASSERT_DOUBLE_EQUAL( gps_tow, 302413.0, 1e-08 );
}

void test_TIMECONV_GetUTCTimeFromJulianDate(void)
{
  unsigned short utc_year;      // Universal Time Coordinated  [year]
  unsigned char  utc_month;     // Universal Time Coordinated  [1-12 months] 
  unsigned char  utc_day;       // Universal Time Coordinated  [1-31 days]
  unsigned char  utc_hour;      // Universal Time Coordinated  [hours]
  unsigned char  utc_minute;    // Universal Time Coordinated  [minutes]
  float          utc_seconds;   // Universal Time Coordinated  [s]
  double         julian_date;   // Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
  BOOL result;

  const double julian_date_start_of_gps_time = (2444244.5);  // [days]
 
  julian_date = julian_date_start_of_gps_time;

  result = TIMECONV_GetUTCTimeFromJulianDate(
    julian_date,
    &utc_year,
    &utc_month,
    &utc_day,
    &utc_hour,
    &utc_minute,
    &utc_seconds );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_year == 1980 );
  CU_ASSERT( utc_month == 1 );
  CU_ASSERT( utc_day == 6 );
  CU_ASSERT( utc_hour == 0 );
  CU_ASSERT( utc_minute == 0 );
  CU_ASSERT_DOUBLE_EQUAL( utc_seconds, 0.0, 1e-08  );


  julian_date = julian_date_start_of_gps_time + 366;
  result = TIMECONV_GetUTCTimeFromJulianDate(
    julian_date,
    &utc_year,
    &utc_month,
    &utc_day,
    &utc_hour,
    &utc_minute,
    &utc_seconds );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_year == 1981 );
  CU_ASSERT( utc_month == 1 );
  CU_ASSERT( utc_day == 6 );
  CU_ASSERT( utc_hour == 0 );
  CU_ASSERT( utc_minute == 0 );
  CU_ASSERT_DOUBLE_EQUAL( utc_seconds, 0.0, 1e-08  );
}


void test_TIMECONV_GetGPSTimeFromUTCTime(void)
{
  unsigned short utc_year;      // Universal Time Coordinated  [year]
  unsigned char  utc_month;     // Universal Time Coordinated  [1-12 months] 
  unsigned char  utc_day;       // Universal Time Coordinated  [1-31 days]
  unsigned char  utc_hour;      // Universal Time Coordinated  [hours]
  unsigned char  utc_minute;    // Universal Time Coordinated  [minutes]
  float          utc_seconds;   // Universal Time Coordinated  [s]
  unsigned short gps_week;      // GPS week (0-1024+)            [week]
  double         gps_tow;       // GPS time of week [s]
  BOOL result;

  utc_year = 1980;
  utc_month = 1;
  utc_day = 6;
  utc_hour = 0;
  utc_minute = 0;
  utc_seconds = 0.0;
  result = TIMECONV_GetGPSTimeFromUTCTime(
    utc_year,
    utc_month, 
    utc_day, 
    utc_hour, 
    utc_minute, 
    utc_seconds, 
    &gps_week, 
    &gps_tow );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( gps_week == 0 );
  CU_ASSERT_DOUBLE_EQUAL( gps_tow, 0.0, 1e-08 );

  utc_year = 1999;
  utc_month = 8;
  utc_day = 22; // GPS week rollover
  utc_hour = 0;
  utc_minute = 0;
  utc_seconds = 0.0;
  result = TIMECONV_GetGPSTimeFromUTCTime(
    utc_year,
    utc_month, 
    utc_day, 
    utc_hour, 
    utc_minute, 
    utc_seconds, 
    &gps_week, 
    &gps_tow );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( gps_week == 1024 );
  CU_ASSERT_DOUBLE_EQUAL( gps_tow, 13.0, 1e-08 ); // 13.0 second UTC_offset at this point
}

void test_TIMECONV_GetUTCTimeFromGPSTime(void)
{
  unsigned short utc_year;      // Universal Time Coordinated  [year]
  unsigned char  utc_month;     // Universal Time Coordinated  [1-12 months] 
  unsigned char  utc_day;       // Universal Time Coordinated  [1-31 days]
  unsigned char  utc_hour;      // Universal Time Coordinated  [hours]
  unsigned char  utc_minute;    // Universal Time Coordinated  [minutes]
  float          utc_seconds;   // Universal Time Coordinated  [s]
  unsigned short gps_week;      // GPS week (0-1024+)            [week]
  double         gps_tow;       // GPS time of week [s]
  BOOL result;

  gps_week = 0;
  gps_tow = 0.0;
  result = TIMECONV_GetUTCTimeFromGPSTime(
    gps_week, 
    gps_tow,
    &utc_year,
    &utc_month, 
    &utc_day, 
    &utc_hour, 
    &utc_minute, 
    &utc_seconds );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_year == 1980 );
  CU_ASSERT( utc_month == 1 );
  CU_ASSERT( utc_day == 6 );
  CU_ASSERT( utc_hour == 0 );
  CU_ASSERT( utc_minute == 0 );
  CU_ASSERT_DOUBLE_EQUAL( utc_seconds, 0.0, 1e-08 );

  gps_week = 1024;
  gps_tow = 13.0;
  result = TIMECONV_GetUTCTimeFromGPSTime(
    gps_week, 
    gps_tow,
    &utc_year,
    &utc_month, 
    &utc_day, 
    &utc_hour, 
    &utc_minute, 
    &utc_seconds );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_year == 1999 );
  CU_ASSERT( utc_month == 8 );
  CU_ASSERT( utc_day == 22 );
  CU_ASSERT( utc_hour == 0 );
  CU_ASSERT( utc_minute == 0 );
  CU_ASSERT_DOUBLE_EQUAL( utc_seconds, 0.0, 1e-08 );
}


void test_TIMECONV_DetermineUTCOffset(void)
{
  double julian_date;
  unsigned char utc_offset;
  BOOL result;

  ///  6 jan 1980    0,    Jan 06, 1980, 00:00:00.0,    2444244.5000 \n
  ///  1 jul 1981    1,    Jul 01, 1981, 00:00:00.0,    2444786.5000 \n
  ///  1 jul 1982    2,    Jul 01, 1982, 00:00:00.0,    2445151.5000 \n
  ///  1 jul 1983    3,    Jul 01, 1983, 00:00:00.0,    2445516.5000 \n
  ///  1 jul 1985    4,    Jul 01, 1985, 00:00:00.0,    2446247.5000 \n
  ///  1 jan 1988    5,    Jan 01, 1988, 00:00:00.0,    2447161.5000 \n
  ///  1 jan 1990    6,    Jan 01, 1990, 00:00:00.0,    2447892.5000 \n
  ///  1 jan 1991    7,    Jan 01, 1991, 00:00:00.0,    2448257.5000 \n
  ///  1 jul 1992    8,    Jul 01, 1992, 00:00:00.0,    2448804.5000 \n
  ///  1 jul 1993    9,    Jul 01, 1993, 00:00:00.0,    2449169.5000 \n
  ///  1 jul 1994    10,   Jul 01, 1994, 00:00:00.0,    2449534.5000 \n
  ///  1 jan 1996    11,   Jan 01, 1996, 00:00:00.0,    2450083.5000 \n
  ///  1 jul 1997    12,   Jul 01, 1997, 00:00:00.0,    2450630.5000 \n
  ///  1 jan 1999    13,   Jan 01, 1999, 00:00:00.0,    2451179.5000 \n

  julian_date = 2444244.5000 + 1;
  result = TIMECONV_DetermineUTCOffset(
    julian_date,
    &utc_offset );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_offset == 0 );

  julian_date = 2444786.5 + 1;
  result = TIMECONV_DetermineUTCOffset(
    julian_date,
    &utc_offset );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_offset == 1 );

  julian_date = 2445151.5 + 1;
  result = TIMECONV_DetermineUTCOffset(
    julian_date,
    &utc_offset );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_offset == 2 );

  julian_date = 2445516.5 + 1;
  result = TIMECONV_DetermineUTCOffset(
    julian_date,
    &utc_offset );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_offset == 3 );

  julian_date = 2446247.5 + 1;
  result = TIMECONV_DetermineUTCOffset(
    julian_date,
    &utc_offset );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_offset == 4 );

  julian_date = 2447161.5 + 1;
  result = TIMECONV_DetermineUTCOffset(
    julian_date,
    &utc_offset );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_offset == 5 );

  julian_date = 2447892.5 + 1;
  result = TIMECONV_DetermineUTCOffset(
    julian_date,
    &utc_offset );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_offset == 6 );

  julian_date = 2448257.5 + 1;
  result = TIMECONV_DetermineUTCOffset(
    julian_date,
    &utc_offset );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_offset == 7 );

  julian_date = 2448804.5 + 1;
  result = TIMECONV_DetermineUTCOffset(
    julian_date,
    &utc_offset );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_offset == 8 );

  julian_date = 2449169.5 + 1;
  result = TIMECONV_DetermineUTCOffset(
    julian_date,
    &utc_offset );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_offset == 9 );

  julian_date = 2449534.5 + 1;
  result = TIMECONV_DetermineUTCOffset(
    julian_date,
    &utc_offset );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_offset == 10 );

  julian_date = 2450083.5 + 1;
  result = TIMECONV_DetermineUTCOffset(
    julian_date,
    &utc_offset );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_offset == 11 );

  julian_date = 2450630.5 + 1;
  result = TIMECONV_DetermineUTCOffset(
    julian_date,
    &utc_offset );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_offset == 12 );

  julian_date = 2451179.5 + 1;
  result = TIMECONV_DetermineUTCOffset(
    julian_date,
    &utc_offset );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc_offset == 13 );

}


void test_TIMECONV_GetNumberOfDaysInMonth(void)
{
  unsigned char days_in_month; // Days in the specified month   [1-28|29|30|31 days]
  BOOL result;
  
  result = TIMECONV_GetNumberOfDaysInMonth( 1980, 1, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 31 );
  result = TIMECONV_GetNumberOfDaysInMonth( 1980, 2, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 29 );
  result = TIMECONV_GetNumberOfDaysInMonth( 1980, 3, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 31 );
  result = TIMECONV_GetNumberOfDaysInMonth( 1980, 4, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 30 );
  result = TIMECONV_GetNumberOfDaysInMonth( 1980, 5, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 31 );
  result = TIMECONV_GetNumberOfDaysInMonth( 1980, 6, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 30 );
  result = TIMECONV_GetNumberOfDaysInMonth( 1980, 7, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 31 );
  result = TIMECONV_GetNumberOfDaysInMonth( 1980, 8, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 31 );
  result = TIMECONV_GetNumberOfDaysInMonth( 1980, 9, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 30 );
  result = TIMECONV_GetNumberOfDaysInMonth( 1980, 10, &days_in_month ); CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 31 );
  result = TIMECONV_GetNumberOfDaysInMonth( 1980, 11, &days_in_month ); CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 30 );
  result = TIMECONV_GetNumberOfDaysInMonth( 1980, 12, &days_in_month ); CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 31 );
  
  result = TIMECONV_GetNumberOfDaysInMonth( 2006, 1, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 31 );
  result = TIMECONV_GetNumberOfDaysInMonth( 2006, 2, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 28 );
  result = TIMECONV_GetNumberOfDaysInMonth( 2006, 3, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 31 );
  result = TIMECONV_GetNumberOfDaysInMonth( 2006, 4, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 30 );
  result = TIMECONV_GetNumberOfDaysInMonth( 2006, 5, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 31 );
  result = TIMECONV_GetNumberOfDaysInMonth( 2006, 6, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 30 );
  result = TIMECONV_GetNumberOfDaysInMonth( 2006, 7, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 31 );
  result = TIMECONV_GetNumberOfDaysInMonth( 2006, 8, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 31 );
  result = TIMECONV_GetNumberOfDaysInMonth( 2006, 9, &days_in_month );  CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 30 );
  result = TIMECONV_GetNumberOfDaysInMonth( 2006, 10, &days_in_month ); CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 31 );
  result = TIMECONV_GetNumberOfDaysInMonth( 2006, 11, &days_in_month ); CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 30 );
  result = TIMECONV_GetNumberOfDaysInMonth( 2006, 12, &days_in_month ); CU_ASSERT_FATAL( result ); CU_ASSERT( days_in_month == 31 );

}

void test_TIMECONV_IsALeapYear(void)
{
  CU_ASSERT( TIMECONV_IsALeapYear(1980) == 1 );
  CU_ASSERT( TIMECONV_IsALeapYear(1981) == 0 );
  CU_ASSERT( TIMECONV_IsALeapYear(2000) == 1 );
  CU_ASSERT( TIMECONV_IsALeapYear(2006) == 0 );
}


void test_TIMECONV_GetDayOfYear(void)
{
  unsigned short dayofyear;
  BOOL result;

  result = TIMECONV_GetDayOfYear(
    1980,
    12,
    31,
    &dayofyear );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( dayofyear == 366 );

  result = TIMECONV_GetDayOfYear(
    2006,
    12,
    31,
    &dayofyear );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( dayofyear == 365 );
}


void test_TIMECONV_GetGPSTimeFromYearAndDayOfYear(void)
{
  BOOL result = 0;

  unsigned short year = 1999;
  unsigned short dayofyear = 31;
  unsigned short gps_week = 0;
  double gps_tow = 0;
 
  result = TIMECONV_GetGPSTimeFromYearAndDayOfYear(
    year,
    dayofyear,
    &gps_week,
    &gps_tow );

  CU_ASSERT( gps_week == 995 ) ;
  CU_ASSERT_DOUBLE_EQUAL( gps_tow, 0, 1e-03 ) ;

  result = TIMECONV_GetGPSTimeFromYearAndDayOfYear(
    2000,
    60,
    &gps_week,
    &gps_tow );

  CU_ASSERT( gps_week == 1051 ) ;
  CU_ASSERT_DOUBLE_EQUAL( gps_tow, 172800, 1e-03 ) ;
}


