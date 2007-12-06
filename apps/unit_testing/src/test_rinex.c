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
  CU_ASSERT( header.obs_types[2] == RINEX_OBS_TYPE_P2 );
  CU_ASSERT( header.obs_types[3] == RINEX_OBS_TYPE_S2 );
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
  
  if(0)
  { 
    // test "aira0010.07o"

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
    
    CU_ASSERT( nrObs == 14 ); // L1, and L2 for 7 satellites.

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
    CU_ASSERT_DOUBLE_EQUAL( obsArray[0].psr, 22845039.6564, 1E-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[0].adr, -7244061.92649, 1e-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[0].cno, 43.7504, 1e-04 );

    CU_ASSERT( obsArray[1].id == 13 );
    CU_ASSERT( obsArray[1].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[1].codeType == GNSS_PCode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[1].psr, 22845035.4924 , 1E-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[1].adr, -5627744.06047, 1e-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[1].cno, 30.7504, 1e-04 );

    CU_ASSERT( obsArray[2].id == 31 );
    CU_ASSERT( obsArray[2].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[2].codeType == GNSS_CACode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[2].psr, 23633146.7034, 1E-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[2].adr, -2365861.25049, 1e-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[2].cno, 39.7504, 1e-04 );

    CU_ASSERT( obsArray[3].id == 31 );
    CU_ASSERT( obsArray[3].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[3].codeType == GNSS_PCode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[3].psr, 23633143.3834, 1E-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[3].adr, -2855938.53847, 1e-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[3].cno, 28.5004, 1e-04 );

  
    CU_ASSERT( obsArray[4].id == 25 );
    CU_ASSERT( obsArray[4].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[4].codeType == GNSS_CACode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[4].psr, 22637559.4534, 1E-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[4].adr, -7887817.80149, 1e-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[4].cno, 47.0004, 1e-04 );

    CU_ASSERT( obsArray[5].id == 25 );
    CU_ASSERT( obsArray[5].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[5].codeType == GNSS_PCode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[5].psr, 22637555.5314, 1E-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[5].adr, -6613691.41747, 1e-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[5].cno, 35.0004, 1e-04 );

  
    CU_ASSERT( obsArray[6].id == 23 );
    CU_ASSERT( obsArray[6].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[6].codeType == GNSS_CACode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[6].psr, 22399110.7504, 1E-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[6].adr, -10666567.86749, 1e-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[6].cno, 47.0004, 1e-04 );

    CU_ASSERT( obsArray[7].id == 23 );
    CU_ASSERT( obsArray[7].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[7].codeType == GNSS_PCode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[7].psr, 22399104.2464, 1E-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[7].adr, -9429599.90647, 1e-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[7].cno, 37.2504, 1e-04 );

  
    CU_ASSERT( obsArray[8].id == 27 );
    CU_ASSERT( obsArray[8].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[8].codeType == GNSS_CACode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[8].psr, 24956414.7504, 1E-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[8].adr, -1712409.00049, 1e-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[8].cno, 36.7504, 1e-04 );

    CU_ASSERT( obsArray[9].id == 27 );
    CU_ASSERT( obsArray[9].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[9].codeType == GNSS_PCode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[9].psr, 24956410.2894, 1E-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[9].adr, -1190201.48546, 1e-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[9].cno, 19.5004, 1e-04 );

  
    CU_ASSERT( obsArray[10].id == 19 );
    CU_ASSERT( obsArray[10].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[10].codeType == GNSS_CACode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[10].psr, 20845187.5704, 1E-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[10].adr, -14268620.92649, 1e-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[10].cno, 51.0004, 1e-04 );

    CU_ASSERT( obsArray[11].id == 19 );
    CU_ASSERT( obsArray[11].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[11].codeType == GNSS_PCode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[11].psr, 20845181.9384, 1E-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[11].adr, -12449166.32048, 1e-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[11].cno, 46.7504, 1e-04 );


    CU_ASSERT( obsArray[12].id == 16 );
    CU_ASSERT( obsArray[12].freqType == GNSS_GPSL1 );
    CU_ASSERT( obsArray[12].codeType == GNSS_CACode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[12].psr, 21448404.5084, 1E-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[12].adr, -15023921.16449, 1e-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[12].cno, 51.5004, 1e-04 );

    CU_ASSERT( obsArray[13].id == 16 );
    CU_ASSERT( obsArray[13].freqType == GNSS_GPSL2 );
    CU_ASSERT( obsArray[13].codeType == GNSS_PCode );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[13].psr, 21448399.8094, 1E-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[13].adr, -12087101.06548, 1e-04 );
    CU_ASSERT_DOUBLE_EQUAL( obsArray[13].cno, 43.2504, 1e-04 );


    /*
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
    }
    */

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
  CU_ASSERT( header.obs_types[2] == RINEX_OBS_TYPE_P2 );
  CU_ASSERT( header.obs_types[3] == RINEX_OBS_TYPE_S2 );
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
    
    CU_ASSERT( nrObs == 8 ); // L1, and L2 for 3 satellites and L1 only for 2 satellites.
  }


    
}

