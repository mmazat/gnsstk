/** 
\file    test_rinex.c
\brief   unit tests for test_rinex.c/.h
\author  Glenn D. MacGougan (GDM)
\date    2007-12-03
\since   2007-12-03

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
#include "rinex.h"
#include "time_conversion.h"
#include "constants.h"


typedef struct 
{
  unsigned short year;
  unsigned char month;
  unsigned char day;
  unsigned char hour;
  unsigned char minute;
  float seconds;
} struct_utc;


int init_suite_RINEX(void)
{
  return 0;
}

int clean_suite_RINEX(void)
{
  return 0;
}



void test_RINEX_GetHeader(void)
{
  BOOL result;
  char buffer[16384];
  unsigned buffer_size = 0;
  double version = 0.0;
  RINEX_enumFileType file_type = RINEX_FILE_TYPE_UNKNOWN;
  
  result = RINEX_GetHeader( 
    "aira0010.07o",
    //"rinex.07o",
    buffer,
    16384,
    &buffer_size,
    &version,
    &file_type
    );

  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( version, 2.1, 1e-02 );
  CU_ASSERT( file_type == RINEX_FILE_TYPE_OBS );
}



void test_RINEX_DecodeHeader_ObservationFile(void)
{
  BOOL result;
  char buffer[16384];
  unsigned buffer_size = 0;
  double version = 0.0;
  RINEX_enumFileType file_type = RINEX_FILE_TYPE_UNKNOWN;
  RINEX_structDecodedHeader header;

  memset( &header, 0, sizeof(RINEX_structDecodedHeader) );
  
  result = RINEX_GetHeader( 
    "aira0010.07o",
    //"rinex.07o",
    buffer,
    16384,
    &buffer_size,
    &version,
    &file_type
    );

  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( version, 2.1, 1e-02 );
  CU_ASSERT_FATAL( file_type == RINEX_FILE_TYPE_OBS );

  
  result = RINEX_DecodeHeader_ObservationFile(
    buffer,
    buffer_size,  
    &header 
    );

  CU_ASSERT_DOUBLE_EQUAL( header.version, 2.1, 1E-02 );
  CU_ASSERT( header.type == RINEX_FILE_TYPE_OBS );
  CU_ASSERT( strcmp( header.marker_name, "AIRA" ) == 0 );
  CU_ASSERT_DOUBLE_EQUAL( header.x, -3530185.4892, 1e-04  );
  CU_ASSERT_DOUBLE_EQUAL( header.y, 4118797.3370, 1e-04  );
  CU_ASSERT_DOUBLE_EQUAL( header.z, 3344036.9313, 1e-04 );
  CU_ASSERT_DOUBLE_EQUAL( header.antenna_delta_h, 0.0, 1e-04 );
  CU_ASSERT_DOUBLE_EQUAL( header.antenna_ecc_e, 0.0, 1e-04 );
  CU_ASSERT_DOUBLE_EQUAL( header.antenna_ecc_n, 0.0, 1e-04 );
  CU_ASSERT_FATAL( header.nr_obs_types == 6 );
  CU_ASSERT( header.obs_types[0] == RINEX_OBS_TYPE_L1 );
  CU_ASSERT( header.obs_types[1] == RINEX_OBS_TYPE_L2 );
  CU_ASSERT( header.obs_types[2] == RINEX_OBS_TYPE_C1 );
  CU_ASSERT( header.obs_types[3] == RINEX_OBS_TYPE_P2 );
  CU_ASSERT( header.obs_types[4] == RINEX_OBS_TYPE_S1 );
  CU_ASSERT( header.obs_types[5] == RINEX_OBS_TYPE_S2 );
  CU_ASSERT( header.default_wavefactor_L1 == RINEX_WAVELENTH_FACTOR_FULL_AMB );
  CU_ASSERT( header.default_wavefactor_L2 == RINEX_WAVELENTH_FACTOR_FULL_AMB );
  CU_ASSERT( header.time_of_first_obs.year == 2007 );
  CU_ASSERT( header.time_of_first_obs.month == 1 );
  CU_ASSERT( header.time_of_first_obs.day == 1 );
  CU_ASSERT( header.time_of_first_obs.hour == 0 );
  CU_ASSERT( header.time_of_first_obs.minute == 0 );
  CU_ASSERT( header.time_of_first_obs.seconds == 0.0 );
  CU_ASSERT( header.time_of_first_obs.time_system == RINEX_TIME_SYSTEM_GPS );


  result = RINEX_GetHeader( 
    "www.ngs.noaa.gov_CORS_rinex210.txt",    
    buffer,
    16384,
    &buffer_size,
    &version,
    &file_type
    );

  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( version, 2.1, 1e-02 );
  CU_ASSERT_FATAL( file_type == RINEX_FILE_TYPE_OBS );

  
  result = RINEX_DecodeHeader_ObservationFile(
    buffer,
    buffer_size,  
    &header 
    );

  CU_ASSERT_DOUBLE_EQUAL( header.version, 2.1, 1E-02 );
  CU_ASSERT( header.type == RINEX_FILE_TYPE_OBS );
  CU_ASSERT( strcmp( header.marker_name, "A 9080" ) == 0 );
  CU_ASSERT_DOUBLE_EQUAL( header.x, 4375274.0, 1e-04  );
  CU_ASSERT_DOUBLE_EQUAL( header.y, 587466.0, 1e-04  );
  CU_ASSERT_DOUBLE_EQUAL( header.z, 4589095.0, 1e-04 );
  CU_ASSERT_DOUBLE_EQUAL( header.antenna_delta_h, 0.9030, 1e-04 );
  CU_ASSERT_DOUBLE_EQUAL( header.antenna_ecc_e, 0.0, 1e-04 );
  CU_ASSERT_DOUBLE_EQUAL( header.antenna_ecc_n, 0.0, 1e-04 );
  CU_ASSERT_FATAL( header.nr_obs_types == 4 );
  CU_ASSERT( header.obs_types[0] == RINEX_OBS_TYPE_P1 );
  CU_ASSERT( header.obs_types[1] == RINEX_OBS_TYPE_L1 );
  CU_ASSERT( header.obs_types[2] == RINEX_OBS_TYPE_L2 );
  CU_ASSERT( header.obs_types[3] == RINEX_OBS_TYPE_P2 );
  CU_ASSERT( header.default_wavefactor_L1 == RINEX_WAVELENTH_FACTOR_FULL_AMB );
  CU_ASSERT( header.default_wavefactor_L2 == RINEX_WAVELENTH_FACTOR_FULL_AMB );
  CU_ASSERT( header.time_of_first_obs.year == 2001 );
  CU_ASSERT( header.time_of_first_obs.month == 3 );
  CU_ASSERT( header.time_of_first_obs.day == 24 );
  CU_ASSERT( header.time_of_first_obs.hour == 13 );
  CU_ASSERT( header.time_of_first_obs.minute == 10 );
  CU_ASSERT( header.time_of_first_obs.seconds == 36.0 );
  CU_ASSERT( header.time_of_first_obs.time_system == RINEX_TIME_SYSTEM_GPS );
  
}


void test_RINEX_GetNextObservationSet(void)
{
  BOOL result;
  char buffer[16384];
  unsigned buffer_size = 0;
  double version = 0.0;
  RINEX_enumFileType file_type = RINEX_FILE_TYPE_UNKNOWN;
  RINEX_structDecodedHeader header;
  unsigned i = 0;
  double gps_tow = 0;
  unsigned short gps_week = 0;

  FILE* fid = NULL;
  BOOL wasEndOfFileReached;  // Has the end of the file been reached (output).
  BOOL wasObservationFound;  // Was a valid observation found (output).
  unsigned filePosition=0;   // The file position for the start of the 
  unsigned nrObs = 0;
  char linebuf[8192];

  GNSS_structMeasurement obsArray[24];

  struct_utc utc;
  
  result = RINEX_GetHeader( 
    "aira0010.07o",
    buffer,
    16384,
    &buffer_size,
    &version,
    &file_type
    );

  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( version, 2.1, 1e-02 );
  CU_ASSERT_FATAL( file_type == RINEX_FILE_TYPE_OBS );

  
  result = RINEX_DecodeHeader_ObservationFile(
    buffer,
    buffer_size,  
    &header 
    );

  CU_ASSERT_DOUBLE_EQUAL( header.version, 2.1, 1E-02 );
  CU_ASSERT( header.type == RINEX_FILE_TYPE_OBS );
  CU_ASSERT( strcmp( header.marker_name, "AIRA" ) == 0 );
  CU_ASSERT_DOUBLE_EQUAL( header.x, -3530185.4892, 1e-04  );
  CU_ASSERT_DOUBLE_EQUAL( header.y, 4118797.3370, 1e-04  );
  CU_ASSERT_DOUBLE_EQUAL( header.z, 3344036.9313, 1e-04 );
  CU_ASSERT_DOUBLE_EQUAL( header.antenna_delta_h, 0.0, 1e-04 );
  CU_ASSERT_DOUBLE_EQUAL( header.antenna_ecc_e, 0.0, 1e-04 );
  CU_ASSERT_DOUBLE_EQUAL( header.antenna_ecc_n, 0.0, 1e-04 );
  CU_ASSERT_FATAL( header.nr_obs_types == 6 );
  CU_ASSERT( header.obs_types[0] == RINEX_OBS_TYPE_L1 );
  CU_ASSERT( header.obs_types[1] == RINEX_OBS_TYPE_L2 );
  CU_ASSERT( header.obs_types[2] == RINEX_OBS_TYPE_C1 );
  CU_ASSERT( header.obs_types[3] == RINEX_OBS_TYPE_P2 );
  CU_ASSERT( header.obs_types[4] == RINEX_OBS_TYPE_S1 );
  CU_ASSERT( header.obs_types[5] == RINEX_OBS_TYPE_S2 );
  CU_ASSERT( header.default_wavefactor_L1 == RINEX_WAVELENTH_FACTOR_FULL_AMB );
  CU_ASSERT( header.default_wavefactor_L2 == RINEX_WAVELENTH_FACTOR_FULL_AMB );
  CU_ASSERT( header.time_of_first_obs.year == 2007 );
  CU_ASSERT( header.time_of_first_obs.month == 1 );
  CU_ASSERT( header.time_of_first_obs.day == 1 );
  CU_ASSERT( header.time_of_first_obs.hour == 0 );
  CU_ASSERT( header.time_of_first_obs.minute == 0 );
  CU_ASSERT( header.time_of_first_obs.seconds == 0.0 );
  CU_ASSERT( header.time_of_first_obs.time_system == RINEX_TIME_SYSTEM_GPS );
  
  { 
    // test "aira0010.07o"
    unsigned total_nr_epochs = 0;

    fid = fopen( "aira0010.07o", "r" );
    CU_ASSERT_FATAL( fid != NULL );

    while( !feof(fid) )
    {
      if( fgets( linebuf, 8192, fid ) == NULL )
        break;
      if( strstr( linebuf, "END OF HEADER" ) != NULL )
        break;
    }

    if( feof(fid) || ferror(fid) != 0 )
      return;
   
    result = RINEX_GetNextObservationSet(
      fid,
      &header,
      &wasEndOfFileReached,
      &wasObservationFound,
      &filePosition,
      obsArray,
      24,
      &nrObs,
      &gps_week,
      &gps_tow
      );    
    CU_ASSERT_FATAL( result );
    CU_ASSERT_FATAL( nrObs == 14 ); // L1, and L2 for 7 satellites.
    total_nr_epochs++;

    for( i = 0; i < 14; i++ )    
    {
      CU_ASSERT( obsArray[i].system == GNSS_GPS );
      CU_ASSERT( obsArray[i].flags.isActive );
      CU_ASSERT( obsArray[i].flags.isCodeLocked );
      CU_ASSERT( obsArray[i].flags.isPhaseLocked );
      CU_ASSERT( obsArray[i].flags.isParityValid );
      CU_ASSERT( obsArray[i].flags.isPsrValid );
      CU_ASSERT( obsArray[i].flags.isAdrValid );
      CU_ASSERT( obsArray[i].flags.isDopplerValid == FALSE );
    }

    CU_ASSERT( obsArray[0].id == 13 );
    CU_ASSERT( obsArray[0].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[0].codeType == GNSS_CACode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[0].psr, 22845039.656, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[0].adr, -7244061.926, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[0].cno, 43.750, 1E-03 );

    CU_ASSERT( obsArray[1].id == 13 );
    CU_ASSERT( obsArray[1].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[1].codeType == GNSS_PCode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[1].psr, 22845035.492 , 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[1].adr, -5627744.060, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[1].cno, 30.750, 1E-03 );

    CU_ASSERT( obsArray[2].id == 31 );
    CU_ASSERT( obsArray[2].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[2].codeType == GNSS_CACode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[2].psr, 23633146.703, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[2].adr, -2365861.250, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[2].cno, 39.750, 1E-03 );

    CU_ASSERT( obsArray[3].id == 31 );
    CU_ASSERT( obsArray[3].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[3].codeType == GNSS_PCode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[3].psr, 23633143.383, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[3].adr, -2855938.538, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[3].cno, 28.500, 1E-03 );

  
    CU_ASSERT( obsArray[4].id == 25 );
    CU_ASSERT( obsArray[4].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[4].codeType == GNSS_CACode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[4].psr, 22637559.453, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[4].adr, -7887817.801, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[4].cno, 47.000, 1E-03 );

    CU_ASSERT( obsArray[5].id == 25 );
    CU_ASSERT( obsArray[5].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[5].codeType == GNSS_PCode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[5].psr, 22637555.531, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[5].adr, -6613691.417, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[5].cno, 35.000, 1E-03 );

  
    CU_ASSERT( obsArray[6].id == 23 );
    CU_ASSERT( obsArray[6].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[6].codeType == GNSS_CACode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[6].psr, 22399110.750, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[6].adr, -10666567.867, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[6].cno, 47.000, 1E-03 );

    CU_ASSERT( obsArray[7].id == 23 );
    CU_ASSERT( obsArray[7].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[7].codeType == GNSS_PCode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[7].psr, 22399104.246, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[7].adr, -9429599.906, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[7].cno, 37.250, 1E-03 );

  
    CU_ASSERT( obsArray[8].id == 27 );
    CU_ASSERT( obsArray[8].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[8].codeType == GNSS_CACode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[8].psr, 24956414.750, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[8].adr, -1712409.000, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[8].cno, 36.750, 1E-03 );

    CU_ASSERT( obsArray[9].id == 27 );
    CU_ASSERT( obsArray[9].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[9].codeType == GNSS_PCode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[9].psr, 24956410.289, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[9].adr, -1190201.485, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[9].cno, 19.500, 1E-03 );

  
    CU_ASSERT( obsArray[10].id == 19 );
    CU_ASSERT( obsArray[10].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[10].codeType == GNSS_CACode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[10].psr, 20845187.570, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[10].adr, -14268620.926, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[10].cno, 51.000, 1E-03 );

    CU_ASSERT( obsArray[11].id == 19 );
    CU_ASSERT( obsArray[11].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[11].codeType == GNSS_PCode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[11].psr, 20845181.938, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[11].adr, -12449166.320, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[11].cno, 46.750, 1E-03 );


    CU_ASSERT( obsArray[12].id == 16 );
    CU_ASSERT( obsArray[12].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[12].codeType == GNSS_CACode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[12].psr, 21448404.508, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[12].adr, -15023921.164, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[12].cno, 51.500, 1E-03 );

    CU_ASSERT( obsArray[13].id == 16 );
    CU_ASSERT( obsArray[13].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[13].codeType == GNSS_PCode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[13].psr, 21448399.809, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[13].adr, -12087101.065, 1E-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[13].cno, 43.250, 1E-03 );

    while( wasObservationFound && !wasEndOfFileReached )
    {
      RINEX_GetNextObservationSet(
        fid,
        &header,
        &wasEndOfFileReached,
        &wasObservationFound,
        &filePosition,
        obsArray,
        24,
        &nrObs,
        &gps_week,
        &gps_tow      
        );
      if( wasObservationFound )
        total_nr_epochs++;

      if( total_nr_epochs == 2880 ) 
      {
        // the last record
        i = 0;
        CU_ASSERT( obsArray[i].id == 13 );
        CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
        CU_ASSERT( obsArray[i].codeType == GNSS_CACode );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 22849570.617, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -7211395.695, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 44.000, 1E-03 );
        i++;
        CU_ASSERT( obsArray[i].id == 13 );
        CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
        CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 22849566.484, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -5610987.454, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 31.000, 1E-03 );
        i++;
        CU_ASSERT( obsArray[i].id == 25 );
        CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
        CU_ASSERT( obsArray[i].codeType == GNSS_CACode );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 22756497.375, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -6098094.969, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 45.000, 1E-03 );
        i++;
        CU_ASSERT( obsArray[i].id == 25 );
        CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
        CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 22756494.617, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -5877136.896, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 34.000, 1E-03 );
        i++;
        CU_ASSERT( obsArray[i].id == 23 );
        CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
        CU_ASSERT( obsArray[i].codeType == GNSS_CACode );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 22475769.211, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -10284321.602, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 45.500, 1E-03 );
        i++;
        CU_ASSERT( obsArray[i].id == 23 );
        CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
        CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 22475762.820, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -8789458.208, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 34.250, 1E-03 );
        i++;
        CU_ASSERT( obsArray[i].id == 31 );
        CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
        CU_ASSERT( obsArray[i].codeType == GNSS_CACode );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 23767808.828, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -1142317.953, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 34.750, 1E-03 );
        i++;
        CU_ASSERT( obsArray[i].id == 31 );
        CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
        CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 23767805.035, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -1850546.594, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 21.500, 1E-03 );
        i++;
        CU_ASSERT( obsArray[i].id == 27 );
        CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
        CU_ASSERT( obsArray[i].codeType == GNSS_CACode );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 24827754.609, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -2521640.309, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 36.000, 1E-03 );
        i++;
        CU_ASSERT( obsArray[i].id == 27 );
        CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
        CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 24827751.324, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -1849906.420, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 19.000, 1E-03 );
        i++;
        CU_ASSERT( obsArray[i].id == 19 );
        CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
        CU_ASSERT( obsArray[i].codeType == GNSS_CACode );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20784715.734, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -14615047.402, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 50.750, 1E-03 );
        i++;
        CU_ASSERT( obsArray[i].id == 19 );
        CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
        CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20784710.168, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -12203215.200, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 47.000, 1E-03 );
        i++;
        CU_ASSERT( obsArray[i].id == 16 );
        CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
        CU_ASSERT( obsArray[i].codeType == GNSS_CACode );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 21495162.945, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -14601064.457, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 51.000, 1E-03 );
        i++;
        CU_ASSERT( obsArray[i].id == 16 );
        CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
        CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 21495158.898, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -11756509.901, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 42.750, 1E-03 );
      }
    }

    CU_ASSERT( total_nr_epochs == 2880 );
    
    fclose(fid);    
  }


  result = RINEX_GetHeader( 
    "www.ngs.noaa.gov_CORS_rinex210.txt",    
    buffer,
    16384,
    &buffer_size,
    &version,
    &file_type
    );

  CU_ASSERT_FATAL( result );
  CU_ASSERT_DOUBLE_EQUAL( version, 2.1, 1e-02 );
  CU_ASSERT_FATAL( file_type == RINEX_FILE_TYPE_OBS );

  
  result = RINEX_DecodeHeader_ObservationFile(
    buffer,
    buffer_size,  
    &header 
    );

  CU_ASSERT_DOUBLE_EQUAL( header.version, 2.1, 1E-02 );
  CU_ASSERT( header.type == RINEX_FILE_TYPE_OBS );
  CU_ASSERT( strcmp( header.marker_name, "A 9080" ) == 0 );
  CU_ASSERT_DOUBLE_EQUAL( header.x, 4375274.0, 1e-04  );
  CU_ASSERT_DOUBLE_EQUAL( header.y, 587466.0, 1e-04  );
  CU_ASSERT_DOUBLE_EQUAL( header.z, 4589095.0, 1e-04 );
  CU_ASSERT_DOUBLE_EQUAL( header.antenna_delta_h, 0.9030, 1e-04 );
  CU_ASSERT_DOUBLE_EQUAL( header.antenna_ecc_e, 0.0, 1e-04 );
  CU_ASSERT_DOUBLE_EQUAL( header.antenna_ecc_n, 0.0, 1e-04 );
  CU_ASSERT_FATAL( header.nr_obs_types == 4 );
  CU_ASSERT( header.obs_types[0] == RINEX_OBS_TYPE_P1 );
  CU_ASSERT( header.obs_types[1] == RINEX_OBS_TYPE_L1 );
  CU_ASSERT( header.obs_types[2] == RINEX_OBS_TYPE_L2 );
  CU_ASSERT( header.obs_types[3] == RINEX_OBS_TYPE_P2 );
  CU_ASSERT( header.default_wavefactor_L1 == RINEX_WAVELENTH_FACTOR_FULL_AMB );
  CU_ASSERT( header.default_wavefactor_L2 == RINEX_WAVELENTH_FACTOR_FULL_AMB );
  CU_ASSERT( header.time_of_first_obs.year == 2001 );
  CU_ASSERT( header.time_of_first_obs.month == 3 );
  CU_ASSERT( header.time_of_first_obs.day == 24 );
  CU_ASSERT( header.time_of_first_obs.hour == 13 );
  CU_ASSERT( header.time_of_first_obs.minute == 10 );
  CU_ASSERT( header.time_of_first_obs.seconds == 36.0 );
  CU_ASSERT( header.time_of_first_obs.time_system == RINEX_TIME_SYSTEM_GPS );

  { 
    fid = fopen( "www.ngs.noaa.gov_CORS_rinex210.txt", "r" );
    CU_ASSERT_FATAL( fid != NULL );

    while( !feof(fid) )
    {
      if( fgets( linebuf, 8192, fid ) == NULL )
        break;
      if( strstr( linebuf, "END OF HEADER" ) != NULL )
        break;
    }

    if( feof(fid) || ferror(fid) != 0 )
      return;
   
    // Get the first observation set.
    result = RINEX_GetNextObservationSet(
      fid,
      &header,
      &wasEndOfFileReached,
      &wasObservationFound,
      &filePosition,
      obsArray,
      24,
      &nrObs,
      &gps_week,
      &gps_tow      
      );
    CU_ASSERT_FATAL( result );

    
    CU_ASSERT_FATAL( nrObs == 6 ); // L1, and L2 for 3 satellites.

    for( i = 0; i < nrObs; i++ )
    {
      CU_ASSERT( obsArray[i].system == GNSS_GPS );
      CU_ASSERT( obsArray[i].codeType == GNSS_PCode );      
    }
    i = 0;
    CU_ASSERT( obsArray[i].id == 12 );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 23629347.915, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr,.300, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 44.5, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 12 );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 23629364.158, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr,-0.353, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 32.0, 1e-01 );

    i++;
    CU_ASSERT( obsArray[i].id == 9 );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20891534.648, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -.120, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 50.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 9 );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20891541.292, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -.358, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 32.0, 1e-01 );

    i++;
    CU_ASSERT( obsArray[i].id == 6 );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20607600.189, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -.430, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 50.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 6 );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20607605.848, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, .394, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 32.0, 1e-01 );

    result = TIMECONV_GetUTCTimeFromGPSTime( 
      obsArray[0].week, 
      obsArray[0].tow+23629347.915/LIGHTSPEED, 
      &utc.year,
      &utc.month,
      &utc.day,
      &utc.hour,
      &utc.minute,
      &utc.seconds
      );
    utc.seconds += 13; // no utc offset in RINEX time.
    CU_ASSERT_FATAL( result );
    CU_ASSERT( utc.year == 2001 );
    CU_ASSERT( utc.month == 3 );
    CU_ASSERT( utc.day == 24 );
    CU_ASSERT( utc.hour == 13 );
    CU_ASSERT( utc.minute == 10 );
    CU_ASSERT_DOUBLE_EQUAL( utc.seconds, 36.0, 1e-01 );

    result = TIMECONV_GetUTCTimeFromGPSTime( 
      obsArray[5].week, 
      obsArray[5].tow+20607605.848/LIGHTSPEED + 13,  // + 13 since there is no utc offset in RINEX time.
      &utc.year,
      &utc.month,
      &utc.day,
      &utc.hour,
      &utc.minute,
      &utc.seconds
      );
    CU_ASSERT_FATAL( result );
    CU_ASSERT( utc.year == 2001 );
    CU_ASSERT( utc.month == 3 );
    CU_ASSERT( utc.day == 24 );
    CU_ASSERT( utc.hour == 13 );
    CU_ASSERT( utc.minute == 10 );
    CU_ASSERT_DOUBLE_EQUAL( utc.seconds, 36.0, 1e-01 );


    // Get the next observation set.
    result = RINEX_GetNextObservationSet(
      fid,
      &header,
      &wasEndOfFileReached,
      &wasObservationFound,
      &filePosition,
      obsArray,
      24,
      &nrObs,
      &gps_week,
      &gps_tow      
      );
    CU_ASSERT_FATAL( result );
    
    CU_ASSERT_FATAL( nrObs == 8 ); // L1, and L2 for 3 satellites and two L1 for GLONASS.

    i = 0;
    CU_ASSERT( obsArray[i].id == 12 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 23619095.450, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -53875.632, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 44.5, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 12 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 23619112.008, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -41981.375, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 32.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 9 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20886075.667, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -28688.027, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 50.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 9 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20886082.101, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -22354.535, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 32.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 6 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20611072.689, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 18247.789, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 50.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 6 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20611078.410, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 14219.770, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 32.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 21 );
    CU_ASSERT( obsArray[i].system == GNSS_GLONASS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 21345678.576, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 12345.567, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 28.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 22 );
    CU_ASSERT( obsArray[i].system == GNSS_GLONASS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 22123456.789, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 23456.789, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 28.0, 1e-01 );


    // Get the next observation set.
    result = RINEX_GetNextObservationSet(
      fid,
      &header,
      &wasEndOfFileReached,
      &wasObservationFound,
      &filePosition,
      obsArray,
      24,
      &nrObs,
      &gps_week,
      &gps_tow      
      );
    CU_ASSERT_FATAL( result );
    
    CU_ASSERT_FATAL( nrObs == 8 ); // L1, and L2 for 3 satellites and two L1 for GLONASS.

    i = 0;
    CU_ASSERT( obsArray[i].id == 16 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 21110991.756, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 16119.980, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 39.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 16 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 21110998.441, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 12560.510, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 32.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 12 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 23588424.398, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -215050.557, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 33.5, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 12 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 23588439.570, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -167571.734, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 32.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 9 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20869878.790, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -113803.187, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 44.5, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 9 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20869884.938, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -88677.926, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 32.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 6 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20621643.727, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 73797.462, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 39.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 6 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20621649.276, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 57505.177, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 32.0, 1e-01 );
    i++;
    

    // Get the next observation set.
    result = RINEX_GetNextObservationSet(
      fid,
      &header,
      &wasEndOfFileReached,
      &wasObservationFound,
      &filePosition,
      obsArray,
      24,
      &nrObs,
      &gps_week,
      &gps_tow      
      );
    CU_ASSERT_FATAL( result );

    // confirm that the marker name has changed
    CU_ASSERT( strcmp( header.marker_name, "A 9081" ) == 0 );

    // confirm that the antenna height has changed
    CU_ASSERT_DOUBLE_EQUAL( header.antenna_delta_h, 0.999, 1e-02 );
    
    CU_ASSERT_FATAL( nrObs == 8 ); // L1, and L2 for 3 satellites and two L1 for GLONASS.

    i = 0;
    CU_ASSERT( obsArray[i].id == 16 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 21112589.384, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 24515.877, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 33.5, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 16 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 21112596.187, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 19102.763, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 17.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 12 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 23578228.338, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -268624.234, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 39.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 12 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 23578244.398, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -209317.284, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 22.5, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 6 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20625218.088, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 92581.207, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 39.0, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 6 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20625223.795, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 72141.846, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 22.5, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 9 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20864539.693, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -141858.836, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 44.5, 1e-01 );
    i++;
    CU_ASSERT( obsArray[i].id == 9 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20864545.943, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -110539.435, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 28.0, 1e-01 );
    i++;


    // Get the next observation set.
    result = RINEX_GetNextObservationSet(
      fid,
      &header,
      &wasEndOfFileReached,
      &wasObservationFound,
      &filePosition,
      obsArray,
      24,
      &nrObs,
      &gps_week,
      &gps_tow      
      );
    CU_ASSERT_FATAL( result );

    CU_ASSERT_FATAL( nrObs == 8 ); // L1, and L2 for 3 satellites and two L1 for GLONASS.

    i = 0;
    CU_ASSERT( obsArray[i].id == 16 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 21124965.133, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 89551.302, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 33.5, 1e-01 );
    CU_ASSERT( obsArray[i].flags.isNoCycleSlipDetected == FALSE );
    i++;
    CU_ASSERT( obsArray[i].id == 16 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 21124972.275, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 69779.626, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 22.5, 1e-01 );
    CU_ASSERT( obsArray[i].flags.isNoCycleSlipDetected == FALSE );
    i++;
    CU_ASSERT( obsArray[i].id == 12 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 23507272.372, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -212616.150, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 39.0, 1e-01 );    
    i++;
    CU_ASSERT( obsArray[i].id == 12 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 23507288.421, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -165674.789, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 28, 1e-01 );    
    i++;
    CU_ASSERT( obsArray[i].id == 9 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20828010.354, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -333820.093, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 33.5, 1e-01 );    
    i++;
    CU_ASSERT( obsArray[i].id == 9 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20828017.129, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -260119.395, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 28.0, 1e-01 );    
    i++;
    CU_ASSERT( obsArray[i].id == 6 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20650944.902, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 227775.130, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 39.0, 1e-01 );    
    i++;
    CU_ASSERT( obsArray[i].id == 6 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20650950.363, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 177487.651, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 22.5, 1e-01 );    
    i++;    


    // Get the next observation set.
    result = RINEX_GetNextObservationSet(
      fid,
      &header,
      &wasEndOfFileReached,
      &wasObservationFound,
      &filePosition,
      obsArray,
      24,
      &nrObs,
      &gps_week,
      &gps_tow      
      );
    CU_ASSERT_FATAL( result );

    CU_ASSERT_FATAL( nrObs == 0 ); // Cycle slip observation records which are not processed yet.


    // Get the next observation set.
    result = RINEX_GetNextObservationSet(
      fid,
      &header,
      &wasEndOfFileReached,
      &wasObservationFound,
      &filePosition,
      obsArray,
      24,
      &nrObs,
      &gps_week,
      &gps_tow      
      );
    CU_ASSERT_FATAL( result );

    CU_ASSERT_FATAL( nrObs == 8 ); // L1 and L2 for 4 GPS satellites.

    result = TIMECONV_GetUTCTimeFromGPSTime( 
      obsArray[i].week, 
      obsArray[0].tow+obsArray[0].psr/LIGHTSPEED + 13, // +13 since there is no utc offset in RINEX time.
      &utc.year,
      &utc.month,
      &utc.day,
      &utc.hour,
      &utc.minute,
      &utc.seconds );
    CU_ASSERT_FATAL( result );
    CU_ASSERT( utc.year == 2001 );
    CU_ASSERT( utc.month == 3 );
    CU_ASSERT( utc.day == 24 );
    CU_ASSERT( utc.hour == 13 );
    CU_ASSERT( utc.minute == 14 );
    CU_ASSERT_DOUBLE_EQUAL( utc.seconds, 48.0, 1e-01 );

    i=0;
    CU_ASSERT( obsArray[i].id == 16 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 21128884.159, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 110143.144, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 39.0, 1e-01 );    
    i++;    
    CU_ASSERT( obsArray[i].id == 16 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 21128890.7764, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 85825.18545, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 28.0, 1e-01 );    
    i++;    
    CU_ASSERT( obsArray[i].id == 12 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 23487131.045, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -318463.297, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 39.0, 1e-01 );    
    i++;    
    CU_ASSERT( obsArray[i].id == 12 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 23487146.149, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -248152.72824, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 22.5, 1e-01 );    
    i++;    
    CU_ASSERT( obsArray[i].id == 9 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20817844.743, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -387242.571, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 33.5, 1e-01 );    
    i++;    
    CU_ASSERT( obsArray[i].id == 9 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20817851.322, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, -301747.22925, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 28.0, 1e-01 );    
    i++;    
    CU_ASSERT( obsArray[i].id == 6 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20658519.895, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 267583.67817, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 39.0, 1e-01 );    
    CU_ASSERT( obsArray[i].flags.isNoCycleSlipDetected == FALSE );
    i++;    
    CU_ASSERT( obsArray[i].id == 6 );
    CU_ASSERT( obsArray[i].system == GNSS_GPS );
    CU_ASSERT( obsArray[i].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[i].codeType == GNSS_PCode );
    CU_ASSERT( obsArray[i].channel == i );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].psr, 20658525.869, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].adr, 208507.26234, 1e-03 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[i].cno, 22.5, 1e-01 );    
    i++;    

    fclose(fid);
  }   
}


void test_RINEX_DecodeGPSNavigationFile(void)
{
  BOOL result;
  GNSS_structKlobuchar iono_model;
  GPS_structEphemeris ephemeris_array[512];
  unsigned length_ephemeris_array = 0;
  unsigned i = 0;

  struct_utc utc;

  result = RINEX_DecodeGPSNavigationFile(
    "aira0010.07n",
    &iono_model,
    ephemeris_array,
    512,
    &length_ephemeris_array
    );

  CU_ASSERT_FATAL( result );
  CU_ASSERT_FATAL( length_ephemeris_array == 333 );

  CU_ASSERT( iono_model.isValid == TRUE );
  CU_ASSERT( iono_model.week == 1408 )
  CU_ASSERT( iono_model.tow == 233472 ); 

  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha0, 7.4506E-09, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha1, -1.4901E-08, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha2, -5.9605E-08, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha3, 1.1921E-07, 1e-13 );

  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta0, 9.0112E+04, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta1, -6.5536E+04, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta2, -1.3107E+05, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta3, 4.5875E+05, 1e-13 );

  // examine the first record
  /*
  13 07  1  1  0  0  0.0 1.281178556383E-04 3.979039320257E-12 0.000000000000E+00
    2.390000000000E+02-1.053750000000E+02 3.802301140610E-09 1.191964944844E+00
   -5.604699254036E-06 2.938966266811E-03 2.777203917503E-06 5.153674734116E+03
    8.640000000000E+04-9.499490261078E-08-1.452396007335E+00 1.117587089539E-08
    9.933681219067E-01 3.428125000000E+02 1.254085410171E+00-7.978189664470E-09
   -3.142988092009E-10 1.000000000000E+00 1.408000000000E+03 0.000000000000E+00
    2.000000000000E+00 0.000000000000E+00-1.117587089539E-08 2.390000000000E+02
    8.149800000000D+04 4.000000000000D+00
  */

  result = TIMECONV_GetUTCTimeFromGPSTime(
    ephemeris_array[0].week,
    ephemeris_array[0].toe+14, // no utc offset in RINEX time.
    &utc.year,
    &utc.month,
    &utc.day,
    &utc.hour,
    &utc.minute,
    &utc.seconds 
    );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc.year == 2007 );
  CU_ASSERT( utc.month == 1 );
  CU_ASSERT( utc.day == 1 );
  CU_ASSERT( utc.hour == 0 );
  CU_ASSERT( utc.minute == 0 );
  CU_ASSERT_DOUBLE_EQUAL( utc.seconds, 0.0, 1e-2 );

  CU_ASSERT( ephemeris_array[0].prn == 13 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].af0, 1.281178556383E-04, 1e-15 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].af1, 3.979039320257E-12, 1e-23 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].af2, 0.000000000000E+00, 1e-10 );

  CU_ASSERT( ephemeris_array[0].iode == 239 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].crs, -1.053750000000E+02, 1e-07 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].delta_n, 3.802301140610E-09, 1e-20 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].m0, 1.191964944844E+00, 1e-12 );

  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].cuc, -5.604699254036E-06, 1e-17 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].ecc, 2.938966266811E-03, 1e-15 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].cus, 2.777203917503E-06, 1e-17 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].sqrta, 5.153674734116E+03, 1e-09 );

  CU_ASSERT( ephemeris_array[0].toe == 86400 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].cic, -9.499490261078E-08, 1e-19 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].omega0, -1.452396007335E+00, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].cis, 1.117587089539E-08, 1e-19 );

  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].i0, 9.933681219067E-01, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].crc, 3.428125000000E+02, 1e-10 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].w, 1.254085410171E+00, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].omegadot, -7.978189664470E-09, 1e-20 );

  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].idot, -3.142988092009E-10, 1e-21 );
  CU_ASSERT( ephemeris_array[0].code_on_L2 == 1 );
  CU_ASSERT( ephemeris_array[0].week == 1408 );
  CU_ASSERT( ephemeris_array[0].L2_P_data_flag == 0 );

  CU_ASSERT( ephemeris_array[i].ura == 0 );
  CU_ASSERT( ephemeris_array[0].health == 0 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[0].tgd, -1.117587089539E-08, 1e-19 );
  CU_ASSERT( ephemeris_array[0].iodc == 239 );

  CU_ASSERT( ephemeris_array[0].fit_interval_flag == 0 ); // four hours   

  // Test the very last record
  /*
  27 07  1  2  0  0  0.0 8.633267134428E-05 2.046363078989E-12 0.000000000000E+00
    1.630000000000E+02-1.581562500000E+02 3.835874284874E-09-3.001539225745E+00
   -8.240342140198E-06 2.041919447947E-02 9.946525096893E-06 5.153800029755E+03
    1.728000000000E+05 1.918524503708E-07-4.983656852855E-01-1.359730958939E-07
    9.620745223570E-01 1.831562500000E+02-1.862446808648E+00-7.336019791637E-09
    8.643217391802E-11 1.000000000000E+00 1.408000000000E+03 0.000000000000E+00
    2.800000000000E+00 0.000000000000E+00-4.190951585770E-09 4.190000000000E+02
    1.720680000000E+05 4.000000000000E+00
  */

  i = length_ephemeris_array-1;
  result = TIMECONV_GetUTCTimeFromGPSTime(
    ephemeris_array[i].week,
    ephemeris_array[i].toe+14, // no utc offset in RINEX time.
    &utc.year,
    &utc.month,
    &utc.day,
    &utc.hour,
    &utc.minute,
    &utc.seconds 
    );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc.year == 2007 );
  CU_ASSERT( utc.month == 1 );
  CU_ASSERT( utc.day == 2 );
  CU_ASSERT( utc.hour == 0 );
  CU_ASSERT( utc.minute == 0 );
  CU_ASSERT_DOUBLE_EQUAL( utc.seconds, 0.0, 1e-2 );

  CU_ASSERT( ephemeris_array[i].prn == 27 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].af0, 8.633267134428E-05, 1e-15 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].af1, 2.046363078989E-12, 1e-23 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].af2, 0.000000000000E+00, 1e-10 );

   
  CU_ASSERT( ephemeris_array[i].iode == 163 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].crs, -1.581562500000E+02, 1e-07 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].delta_n, 3.835874284874E-09, 1e-20 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].m0, -3.001539225745E+00, 1e-12 );

  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].cuc, -8.240342140198E-06, 1e-17 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].ecc, 2.041919447947E-02, 1e-15 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].cus, 9.946525096893E-06, 1e-17 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].sqrta, 5.153800029755E+03, 1e-09 );

  CU_ASSERT( ephemeris_array[i].toe == 172800 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].cic, 1.918524503708E-07, 1e-19 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].omega0, -4.983656852855E-01, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].cis, -1.359730958939E-07, 1e-19 );

  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].i0, 9.620745223570E-01, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].crc, 1.831562500000E+02, 1e-10 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].w, -1.862446808648, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].omegadot, -7.336019791637E-09, 1e-20 );

  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].idot, 8.643217391802E-11, 1e-21 );
  CU_ASSERT( ephemeris_array[i].code_on_L2 == 1 );
  CU_ASSERT( ephemeris_array[i].week == 1408 );
  CU_ASSERT( ephemeris_array[i].L2_P_data_flag == 0 );

  CU_ASSERT( ephemeris_array[i].ura == 1 );
  CU_ASSERT( ephemeris_array[i].health == 0 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].tgd, -4.190951585770E-09, 1e-19 );
  CU_ASSERT( ephemeris_array[i].iodc == 419 );

  CU_ASSERT( ephemeris_array[i].fit_interval_flag == 0 ); // four hours   



  result = RINEX_DecodeGPSNavigationFile(
    "AIUB2450.99n",
    &iono_model,
    ephemeris_array,
    512,
    &length_ephemeris_array
    );

  CU_ASSERT_FATAL( result );
  CU_ASSERT_FATAL( length_ephemeris_array == 2 );

  /*
  .1676E-07   .2235E-07  -.1192E-06  -.1192E-06          ION ALPHA
  .1208E+06   .1310E+06  -.1310E+06  -.1966E+06          ION BETA
  */
  CU_ASSERT( iono_model.isValid == TRUE );
  CU_ASSERT( iono_model.week == 1025 )
  CU_ASSERT( iono_model.tow == 552960 ); 

  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha0, .1676E-07, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha1, .2235E-07, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha2, -.1192E-06, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha3, -.1192E-06, 1e-13 );

  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta0, .1208E+06, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta1, .1310E+06, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta2, -.1310E+06, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta3, -.1966E+06, 1e-13 );

  /*
  6 99  9  2 17 51 44.0 -.839701388031E-03 -.165982783074E-10  .000000000000E+00
     .910000000000E+02  .934062500000E+02  .116040547840E-08  .162092304801E+00
     .484101474285E-05  .626740418375E-02  .652112066746E-05  .515365489006E+04
     .409904000000E+06 -.242143869400E-07  .329237003460E+00 -.596046447754E-07d
     .111541663136E+01  .326593750000E+03  .206958726335E+01 -.638312302555E-08d
     .307155651409E-09  .000000000000E+00  .102500000000E+04  .000000000000E+00d
     .000000000000E+00  .000000000000E+00  .000000000000E+00  .910000000000E+02
     .406800000000E+06  .000000000000E+00
     */

  i = 0;
  result = TIMECONV_GetUTCTimeFromGPSTime(
    ephemeris_array[i].week,
    ephemeris_array[i].toe+13, // no utc offset in RINEX time.
    &utc.year,
    &utc.month,
    &utc.day,
    &utc.hour,
    &utc.minute,
    &utc.seconds 
    );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc.year == 1999 );
  CU_ASSERT( utc.month == 9 );
  CU_ASSERT( utc.day == 2 );
  CU_ASSERT( utc.hour == 17 );
  CU_ASSERT( utc.minute == 51 );
  CU_ASSERT_DOUBLE_EQUAL( utc.seconds, 44.0, 1e-2 );

  CU_ASSERT( ephemeris_array[i].prn == 6 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].af0, -.839701388031E-03, 1e-15 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].af1, -.165982783074E-10, 1e-23 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].af2, 0.000000000000E+00, 1e-10 );

  CU_ASSERT( ephemeris_array[i].iode == 91 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].crs, .934062500000E+02, 1e-07 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].delta_n, .116040547840E-08, 1e-20 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].m0, .162092304801E+00, 1e-12 );

  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].cuc, .484101474285E-05, 1e-17 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].ecc, .626740418375E-02, 1e-15 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].cus, .652112066746E-05, 1e-17 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].sqrta, .515365489006E+04, 1e-09 );
     
  CU_ASSERT( ephemeris_array[i].toe == 409904 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].cic, -.242143869400E-07, 1e-19 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].omega0, .329237003460E+00, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].cis, -.596046447754E-07, 1e-19 );

  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].i0, .111541663136E+01, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].crc, .326593750000E+03, 1e-10 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].w, .206958726335E+01, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].omegadot, -.638312302555E-08, 1e-20 );

  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].idot, .307155651409E-09, 1e-21 );
  CU_ASSERT( ephemeris_array[i].code_on_L2 == 0 );
  CU_ASSERT( ephemeris_array[i].week == 1025 );
  CU_ASSERT( ephemeris_array[i].L2_P_data_flag == 0 );
  
  CU_ASSERT( ephemeris_array[i].ura == 0 );
  CU_ASSERT( ephemeris_array[i].health == 0 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].tgd, 0.0, 1e-12 );
  CU_ASSERT( ephemeris_array[i].iodc == 91 );
  
  CU_ASSERT( ephemeris_array[i].fit_interval_flag == 0 ); // four hours   

  /*
  13 99  9  2 19  0  0.0  .490025617182E-03  .204636307899E-11  .000000000000E+00
     .133000000000E+03 -.963125000000E+02  .146970407622E-08  .292961152146E+01
    -.498816370964E-05  .200239347760E-02  .928156077862E-05  .515328476143E+04
     .414000000000E+06 -.279396772385E-07  .243031939942E+01 -.558793544769E-07
     .110192796930E+01  .271187500000E+03 -.232757915425E+01 -.619632953057E-08
    -.785747015231E-11  .000000000000E+00  .102500000000E+04  .000000000000E+00
     .000000000000E+00  .000000000000E+00  .000000000000E+00  .389000000000E+03
     .410400000000E+06  .000000000000E+00
     */

  i = 1;
  result = TIMECONV_GetUTCTimeFromGPSTime(
    ephemeris_array[i].week,
    ephemeris_array[i].toe+13.00001, // no utc offset in RINEX time.
    &utc.year,
    &utc.month,
    &utc.day,
    &utc.hour,
    &utc.minute,
    &utc.seconds 
    );
  CU_ASSERT_FATAL( result );
  CU_ASSERT( utc.year == 1999 );
  CU_ASSERT( utc.month == 9 );
  CU_ASSERT( utc.day == 2 );
  CU_ASSERT( utc.hour == 19 );
  CU_ASSERT( utc.minute == 0 );
  CU_ASSERT_DOUBLE_EQUAL( utc.seconds, 0.0, 1e-2 );

  
  CU_ASSERT( ephemeris_array[i].prn == 13 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].af0, .490025617182E-03, 1e-15 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].af1, .204636307899E-11, 1e-23 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].af2, 0.000000000000E+00, 1e-10 );

  CU_ASSERT( ephemeris_array[i].iode == 133 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].crs, -.963125000000E+02, 1e-07 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].delta_n, .146970407622E-08, 1e-20 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].m0, .292961152146E+01, 1e-12 );

  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].cuc, -.498816370964E-05, 1e-17 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].ecc, .200239347760E-02, 1e-15 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].cus, .928156077862E-05, 1e-17 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].sqrta, .515328476143E+04, 1e-09 );
     
  CU_ASSERT( ephemeris_array[i].toe == 414000 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].cic, -.279396772385E-07, 1e-19 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].omega0, .243031939942E+01, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].cis, -.558793544769E-07, 1e-19 );

  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].i0, .110192796930E+01, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].crc, .271187500000E+03, 1e-10 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].w, -.232757915425E+01, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].omegadot, -.619632953057E-08, 1e-20 );

  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].idot, -.785747015231E-11, 1e-21 );
  CU_ASSERT( ephemeris_array[i].code_on_L2 == 0 );
  CU_ASSERT( ephemeris_array[i].week == 1025 );
  CU_ASSERT( ephemeris_array[i].L2_P_data_flag == 0 );
  
  CU_ASSERT( ephemeris_array[i].ura == 0 );
  CU_ASSERT( ephemeris_array[i].health == 0 );
  CU_ASSERT_DOUBLE_EQUAL( ephemeris_array[i].tgd, 0.0, 1e-12 );
  CU_ASSERT( ephemeris_array[i].iodc == 389 );
  
  CU_ASSERT( ephemeris_array[i].fit_interval_flag == 0 ); // four hours   
}




void test_RINEX_GetKlobucharIonoParametersFromNavFile(void)
{
  BOOL result;
  GNSS_structKlobuchar iono_model;

  result = RINEX_GetKlobucharIonoParametersFromNavFile( "aira0010.07n", &iono_model );

  CU_ASSERT_FATAL( result );

  CU_ASSERT( iono_model.isValid == TRUE );
  CU_ASSERT( iono_model.week == 1408 )
  CU_ASSERT( iono_model.tow == 233472 ); 

  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha0, 7.4506E-09, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha1, -1.4901E-08, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha2, -5.9605E-08, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha3, 1.1921E-07, 1e-13 );

  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta0, 9.0112E+04, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta1, -6.5536E+04, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta2, -1.3107E+05, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta3, 4.5875E+05, 1e-13 );

  result = RINEX_GetKlobucharIonoParametersFromNavFile( "AIUB2450.99n", &iono_model );

  CU_ASSERT_FATAL( result );

  /*
  .1676E-07   .2235E-07  -.1192E-06  -.1192E-06          ION ALPHA
  .1208E+06   .1310E+06  -.1310E+06  -.1966E+06          ION BETA
  */
  CU_ASSERT( iono_model.isValid == TRUE );
  CU_ASSERT( iono_model.week == 1025 )
  CU_ASSERT( iono_model.tow == 552960 ); 

  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha0, .1676E-07, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha1, .2235E-07, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha2, -.1192E-06, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.alpha3, -.1192E-06, 1e-13 );

  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta0, .1208E+06, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta1, .1310E+06, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta2, -.1310E+06, 1e-13 );
  CU_ASSERT_DOUBLE_EQUAL( iono_model.beta3, -.1966E+06, 1e-13 );
}



