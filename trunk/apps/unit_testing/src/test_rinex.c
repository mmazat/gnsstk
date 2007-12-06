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

  FILE* fid = NULL;
  BOOL wasEndOfFileReached;  // Has the end of the file been reached (output).
  BOOL wasObservationFound;  // Was a valid observation found (output).
  unsigned filePosition=0;   // The file position for the start of the 
  unsigned nrObs = 0;
  char linebuf[8192];

  GNSS_structMeasurement obsArray[24];

  struct 
  {
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    float seconds;
  } utc;


  

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
      &nrObs
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
        &nrObs
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
      &nrObs
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
    CU_ASSERT_FATAL( result );
    CU_ASSERT( utc.year == 2001 );
    CU_ASSERT( utc.month == 3 );
    CU_ASSERT( utc.day == 24 );
    CU_ASSERT( utc.hour == 13 );
    CU_ASSERT( utc.minute == 10 );
    CU_ASSERT_DOUBLE_EQUAL( utc.seconds, 36.0, 1e-01 );

    result = TIMECONV_GetUTCTimeFromGPSTime( 
      obsArray[5].week, 
      obsArray[5].tow+20607605.848/LIGHTSPEED, 
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
      &nrObs
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
      &nrObs
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
      &nrObs
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
      &nrObs
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
      &nrObs
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
      &nrObs
      );
    CU_ASSERT_FATAL( result );

    CU_ASSERT_FATAL( nrObs == 8 ); // L1 and L2 for 4 GPS satellites.

    result = TIMECONV_GetUTCTimeFromGPSTime( 
      obsArray[i].week, 
      obsArray[0].tow+obsArray[0].psr/LIGHTSPEED, 
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

