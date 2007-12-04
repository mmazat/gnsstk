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

  
  {
    FILE* fid = NULL;
    BOOL wasEndOfFileReached;  // Has the end of the file been reached (output).
    BOOL wasObservationFound;  // Was a valid observation found (output).
    unsigned filePosition=0;   // The file position for the start of the 
    unsigned nrObs = 0;
    char linebuf[8192];

    NOVATELOEM4_structBinaryHeader header;       // A pointer to a NovAtel OEM4 header information struct (output).
    NOVATELOEM4_structObservation  obsArray[24]; // A pointer to a user provided array of struct_NOVATELOEM4_RANGE (output).

    fid = fopen( "aira0010.07o", "r" );
    CU_ASSERT_FATAL( fid == NULL );

    while( !feof(fid) )
    {
      fgets( linbuf

    
    RINEX_GetNextObservationSet(
      fid,
      &header,
      &wasEndOfFileReached,
      &wasObservationFound,
      &filePosition,
      &header,  
      &obsArray,
      24,
      &nrObs
      );
  }
}

