/**
\file    unit_testing_main.c
\brief   GNSS core 'c' function library: unit testing main().
\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
\since   2007-11-29

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
#include <stdlib.h>
#include <string.h>
#include "Basic.h"  // CUnit/Basic.h
#include "time_conversion.h"
#include "test_geodesy.h"
#include "test_time_conversion.h"
#include "test_novatel.h"
#include "test_yuma.h"
#include "test_ionosphere.h"
#include "test_rinex.h"


/** \brief The function where all suites and tests are added. */
int AddTests();

/** \brief Print the time the test occurred to stdout. */
static void print_system_time(void);

int main(int argc, char* argv[])
{
  CU_BasicRunMode mode = CU_BRM_VERBOSE;
  CU_ErrorAction error_action = CUEA_IGNORE;
  int i;

  setvbuf(stdout, NULL, _IONBF, 0);

  for (i=1 ; i<argc ; i++) 
  {
    if (!strcmp("-i", argv[i])) 
    {
      error_action = CUEA_IGNORE;
    }
    else if (!strcmp("-f", argv[i])) 
    {
      error_action = CUEA_FAIL;
    }
    else if (!strcmp("-A", argv[i])) 
    {
      error_action = CUEA_ABORT;
    }
    else if (!strcmp("-s", argv[i])) 
    {
      mode = CU_BRM_SILENT;
    }
    else if (!strcmp("-n", argv[i])) 
    {
      mode = CU_BRM_NORMAL;
    }
    else if (!strcmp("-v", argv[i])) 
    {
      mode = CU_BRM_VERBOSE;
    }
    /*
    else if (!strcmp("-e", argv[i])) 
    {
      print_example_results();
      return 0;
    }
    */
    else 
    {
      printf("\nUsage:  eGNSS_Test [options]\n\n"
               "Options:   -i   ignore framework errors [default].\n"
               "           -f   fail on framework error.\n"
               "           -A   abort on framework error.\n\n"
               "           -s   silent mode - no output to screen.\n"
               "           -n   normal mode - standard output to screen.\n"
               "           -v   verbose mode - max output to screen [default].\n\n"
             /*"           -e   print expected test results and exit.\n"*/
               "           -h   print this message and exit.\n\n");
      return 0;
    }
  }

  // Output system time and date.
  print_system_time();

  if (CU_initialize_registry()) 
  {
    printf("\nInitialization of Test Registry failed.");
  }
  else 
  {
    if( AddTests() == CUE_SUCCESS )
    {
      CU_basic_set_mode(mode);
      CU_set_error_action(error_action);
      printf("\nTests completed with return value %d.\n", CU_basic_run_tests());
    }
    CU_cleanup_registry();
  }

  return 0;
}



/** 
The AddTests() function for setting up the tests. 
*/
int AddTests()
{
  CU_pSuite pSuite = NULL;



  /* add a suite to the registry */
  pSuite = CU_add_suite("GEODESY", init_suite_GEODESY, clean_suite_GEODESY);
  if (NULL == pSuite)   
    return CU_get_error();
  
  /* add the tests to the suite */
  if( CU_add_test(pSuite, "GEODESY_GetReferenceEllipseParameters()", test_GEODESY_GetReferenceEllipseParameters) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "test_GEODESY_ConvertCoordinates()", test_GEODESY_ConvertCoordinates) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "GEODESY_ComputeNorthingEastingVertical()", test_GEODESY_ComputeNorthingEastingVertical) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "GEODESY_ComputePositionDifference()", test_GEODESY_ComputePositionDifference) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "GEODESY_ComputeMeridianRadiusOfCurvature()", test_GEODESY_ComputeMeridianRadiusOfCurvature) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "GEODESY_ComputePrimeVerticalRadiusOfCurvature()", test_GEODESY_ComputePrimeVerticalRadiusOfCurvature) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "GEODESY_ComputeMeridianArcBetweenTwoLatitudes()", test_GEODESY_ComputeMeridianArcBetweenTwoLatitudes) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "GEODESY_ComputeParallelArcBetweenTwoLongitudes()", test_GEODESY_ComputeParallelArcBetweenTwoLongitudes) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "GEODESY_RotateVectorFromLocalGeodeticFrameToEarthFixedFrame()", test_GEODESY_RotateVectorFromLocalGeodeticFrameToEarthFixedFrame) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "GEODESY_RotateVectorFromEarthFixedFrameToLocalGeodeticFrame()", test_GEODESY_RotateVectorFromEarthFixedFrameToLocalGeodeticFrame) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "GEODESY_ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame()", test_GEODESY_ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame) == NULL )
    return CU_get_error();




  /* add a suite to the registry */
  pSuite = CU_add_suite("TIMECONV", init_suite_GEODESY, clean_suite_GEODESY);
  if (NULL == pSuite)   
    return CU_get_error();
  
  /* add the tests to the suite */
  if( CU_add_test(pSuite, "TIMECONV_GetJulianDateFromGPSTime()", test_TIMECONV_GetJulianDateFromGPSTime) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "TIMECONV_GetJulianDateFromUTCTime()", test_TIMECONV_GetJulianDateFromUTCTime) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "TIMECONV_GetGPSTimeFromJulianDate()", test_TIMECONV_GetGPSTimeFromJulianDate) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "TIMECONV_GetUTCTimeFromJulianDate()", test_TIMECONV_GetUTCTimeFromJulianDate) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "TIMECONV_GetGPSTimeFromUTCTime()", test_TIMECONV_GetGPSTimeFromUTCTime) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "TIMECONV_GetUTCTimeFromGPSTime()", test_TIMECONV_GetUTCTimeFromGPSTime) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "TIMECONV_DetermineUTCOffset()", test_TIMECONV_DetermineUTCOffset) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "TIMECONV_GetNumberOfDaysInMonth()", test_TIMECONV_GetNumberOfDaysInMonth) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "TIMECONV_IsALeapYear()", test_TIMECONV_IsALeapYear) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "TIMECONV_GetDayOfYear()", test_TIMECONV_GetDayOfYear) == NULL )
    return CU_get_error();


  /* add a suite to the registry */
  pSuite = CU_add_suite("NOVATELOEM4", init_suite_GEODESY, clean_suite_GEODESY);
  if (NULL == pSuite)   
    return CU_get_error();

  /* add the tests to the suite */
  if( CU_add_test(pSuite, "NOVATELOEM4_FindNextMessageInFile()", test_NOVATELOEM4_FindNextMessageInFile) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "NOVATELOEM4_DecodeRANGEB()", test_NOVATELOEM4_DecodeRANGEB) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "NOVATELOEM4_DecodeRANGECMPB()", test_NOVATELOEM4_DecodeRANGECMPB) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "NOVATELOEM4_DecodeRAWEPHEMB()", test_NOVATELOEM4_DecodeRAWEPHEMB) == NULL )
    return CU_get_error();



  /* add a suite to the registry */
  pSuite = CU_add_suite("YUMA", init_suite_YUMA, clean_suite_YUMA);
  if (NULL == pSuite)   
    return CU_get_error();

  /* add the tests to the suite */
  if( CU_add_test(pSuite, "YUMA_ReadAlmanacDataFromFile()", test_YUMA_ReadAlmanacDataFromFile) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "YUMA_WriteAlmanacDataToFile()", test_YUMA_WriteAlmanacDataToFile) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "YUMA_WriteSingleAlmanacElementToBuffer()", test_YUMA_WriteSingleAlmanacElementToBuffer) == NULL )
    return CU_get_error();

  /* add a suite to the registry */
  pSuite = CU_add_suite("IONOSPHERE", init_suite_YUMA, clean_suite_YUMA);
  if (NULL == pSuite)   
    return CU_get_error();

  /* add the tests to the suite */
  if( CU_add_test(pSuite, "IONOSPHERE_GetL1KlobucharCorrection()", test_IONOSPHERE_GetL1KlobucharCorrection) == NULL )
    return CU_get_error();

  /* add a suite to the registry */
  pSuite = CU_add_suite("RINEX", init_suite_RINEX, clean_suite_RINEX);
  if (NULL == pSuite)   
    return CU_get_error();

  /* add the tests to the suite */
  if( CU_add_test(pSuite, "RINEX_GetHeader()", test_RINEX_GetHeader) == NULL )
    return CU_get_error();
  if( CU_add_test(pSuite, "RINEX_DecodeHeader_ObservationFile()", test_RINEX_DecodeHeader_ObservationFile) == NULL )
    return CU_get_error();
  
  return CUE_SUCCESS;
}



void print_system_time(void)
{
  BOOL result;
  unsigned short     utc_year;     // Universal Time Coordinated    [year]
  unsigned char      utc_month;    // Universal Time Coordinated    [1-12 months] 
  unsigned char      utc_day;      // Universal Time Coordinated    [1-31 days]
  unsigned char      utc_hour;     // Universal Time Coordinated    [hours]
  unsigned char      utc_minute;   // Universal Time Coordinated    [minutes]
  float              utc_seconds;  // Universal Time Coordinated    [s]
  unsigned char      utc_offset;   // Integer seconds that GPS is ahead of UTC time, always positive             [s], obtained from a look up table
  double             julian_date;  // Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
  unsigned short     gps_week;     // GPS week (0-1024+)            [week]
  double             gps_tow;      // GPS time of week (0-604800.0) [s]
  
  result = TIMECONV_GetSystemTime(
    &utc_year,   
    &utc_month,  
    &utc_day,    
    &utc_hour,   
    &utc_minute,  
    &utc_seconds, 
    &utc_offset,  
    &julian_date, 
    &gps_week,    
    &gps_tow          
    );

  if( result == 0 )
    return;

  printf( "\nDate YYYY-MM-DD, Time HH:MM:SS, of test (UTC).\n");
  printf( "     %4d-%02d-%02d,      %02d:%02d:%02.0f\n", 
    utc_year,
    utc_month,
    utc_day,
    utc_hour,
    utc_minute,
    utc_seconds );
  return;
}
