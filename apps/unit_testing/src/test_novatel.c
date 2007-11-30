/** 
\file    test_novatel.c
\brief   unit tests for novatel.c/.h
\author  Glenn D. MacGougan (GDM)
\date    2007-11-26
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
#include "gps.h"
#include "novatel.h"
#include "constants.h"


int init_suite_NOVATELOEM4(void)
{
  return 0;
}

int clean_suite_NOVATELOEM4(void)
{
  return 0;
}


void test_NOVATELOEM4_FindNextMessageInFile(void)
{
  FILE* fid;
  BOOL result;
  unsigned char message[8192];
  BOOL wasMessageFound = FALSE;
  BOOL wasEndOfFileReached = FALSE;
  unsigned filePosition = 0;
  unsigned short messageLength = 0;
  unsigned short messageID = 0;
  unsigned numberBadCRC = 0;
  
  result = fopen_s( &fid, "crctest.bin", "rb" );
  CU_ASSERT_FATAL( result == 0 );

  if( result == 0 )
    return;

  // crctest.bin - has NovAtel OEM4 binary messages.
  // A valid ALMANAC messages, type 73.
  // An invalid BESTPOS message, type 42.
  // A strange length (100 ) but valid crc RAWGPSSUBFRAME message, type 25.
  // An invalid TRACKSTAT message, type 83.
  // A valid ALMANAC messages, type 73.  
  // A valid BESTPOS message, type 42
  // A strange length (100 ) but valid crc RAWGPSSUBFRAME message, type 25.
  // An invalid message with a large data length and cut off before the crc is present.
  // A valid but emply RANGE message, type 43. 

  result = NOVATELOEM4_FindNextMessageInFile(
    fid,
    message,
    8192, 
    &wasEndOfFileReached,
    &wasMessageFound,
    &filePosition,
    &messageLength,
    &messageID,
    &numberBadCRC );

  CU_ASSERT( !wasEndOfFileReached );
  CU_ASSERT( wasMessageFound );
  CU_ASSERT( messageID == 73 );
  CU_ASSERT( messageLength == 3508 );
  CU_ASSERT( numberBadCRC == 0 );

  result = NOVATELOEM4_FindNextMessageInFile(
    fid,
    message,
    8192, 
    &wasEndOfFileReached,
    &wasMessageFound,
    &filePosition,
    &messageLength,
    &messageID,
    &numberBadCRC );

  CU_ASSERT( !wasEndOfFileReached );
  CU_ASSERT( wasMessageFound );
  CU_ASSERT( messageID == 25 );
  CU_ASSERT( numberBadCRC == 1 );
  
  result = NOVATELOEM4_FindNextMessageInFile(
    fid,
    message,
    8192, 
    &wasEndOfFileReached,
    &wasMessageFound,
    &filePosition,
    &messageLength,
    &messageID,
    &numberBadCRC );

  CU_ASSERT( !wasEndOfFileReached );
  CU_ASSERT( wasMessageFound );
  CU_ASSERT( messageID == 73 );
  CU_ASSERT( messageLength == 3508 );  
  CU_ASSERT( numberBadCRC == 1 );

  result = NOVATELOEM4_FindNextMessageInFile(
    fid,
    message,
    8192, 
    &wasEndOfFileReached,
    &wasMessageFound,
    &filePosition,
    &messageLength,
    &messageID,
    &numberBadCRC );

  CU_ASSERT( !wasEndOfFileReached );
  CU_ASSERT( wasMessageFound );
  CU_ASSERT( messageID == 42 );
  CU_ASSERT( messageLength == 72+28+4 );  
  CU_ASSERT( numberBadCRC == 0 );

  result = NOVATELOEM4_FindNextMessageInFile(
    fid,
    message,
    8192, 
    &wasEndOfFileReached,
    &wasMessageFound,
    &filePosition,
    &messageLength,
    &messageID,
    &numberBadCRC );

  CU_ASSERT( !wasEndOfFileReached );
  CU_ASSERT( wasMessageFound );
  CU_ASSERT( messageID == 25 );
  CU_ASSERT( numberBadCRC == 0 );
  
  result = NOVATELOEM4_FindNextMessageInFile(
    fid,
    message,
    8192, 
    &wasEndOfFileReached,
    &wasMessageFound,
    &filePosition,
    &messageLength,
    &messageID,
    &numberBadCRC );

  CU_ASSERT( !wasEndOfFileReached );
  CU_ASSERT( wasMessageFound );
  CU_ASSERT( messageID == 43 );
  CU_ASSERT( numberBadCRC == 0 );
  CU_ASSERT( messageLength == 4+28+4 );  

  result = NOVATELOEM4_FindNextMessageInFile(
    fid,
    message,
    8192, 
    &wasEndOfFileReached,
    &wasMessageFound,
    &filePosition,
    &messageLength,
    &messageID,
    &numberBadCRC );

  CU_ASSERT( wasEndOfFileReached );
  CU_ASSERT( !wasMessageFound );
  
  if( fid != NULL )
  {
    result = fclose( fid );
    CU_ASSERT( result == 0 );
  }
}

void test_NOVATELOEM4_DecodeRANGEB(void)
{
  unsigned char rangeb[564] = { 
    0xaa,0x44,0x12,0x1c,0x2b,0x00,0x02,0x20,0x14,0x02,0x00,0x00,0x6a,0x78, 
    0x8a,0x04,0x78,0xbb,0xce,0x0c,0x08,0x00,0x04,0x00,0xa7,0xdd,0xa7,0x00,
    0x0c,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x31,0xa1,0x18,0xd7,0xeb,0x0a,
    0x77,0x41,0x92,0xa4,0x27,0x3e,0xb8,0x1b,0x6d,0x18,0x00,0x21,0x38,0xc1,
    0x03,0xae,0xec,0x3b,0x80,0xac,0x1b,0xc5,0x2a,0x68,0x36,0x42,0xa4,0x70,
    0x2a,0x42,0x24,0x3c,0x10,0x08,0x03,0x00,0x00,0x00,0xe4,0x13,0x12,0x64,
    0xec,0x0a,0x77,0x41,0xda,0x36,0xf1,0x40,0x06,0x5f,0x3d,0x93,0x6a,0xe7,
    0xbb,0xc0,0xf0,0xd8,0x35,0x3c,0x96,0x9b,0xf2,0xc4,0x37,0x27,0x09,0x42,
    0x52,0xb8,0x7e,0x40,0x2b,0x3c,0xb0,0x00,0x05,0x00,0x00,0x00,0xd8,0x63,
    0x24,0x86,0x32,0x40,0x77,0x41,0xe1,0x30,0x82,0x3e,0xb3,0x50,0xbe,0xc2,
    0x44,0xe7,0xf9,0x40,0x6a,0x3b,0xfb,0x3b,0x00,0x78,0x35,0x45,0x9b,0x53,
    0x36,0x42,0xf6,0x28,0xd4,0x41,0x84,0x3c,0x10,0x08,0x05,0x00,0x00,0x00,
    0xd1,0x8c,0x28,0x81,0x32,0x40,0x77,0x41,0x73,0x93,0x40,0x40,0x42,0x57,
    0xc7,0x96,0x6e,0x61,0xd5,0x40,0xa5,0xe2,0xc5,0x3c,0x79,0x67,0x0d,0x45,
    0xcb,0xc7,0x0a,0x42,0x14,0xae,0x1f,0x41,0x8b,0x3c,0x30,0x01,0x11,0x00,
    0x00,0x00,0x83,0x85,0x7c,0xd3,0x93,0x50,0x75,0x41,0xa7,0x22,0xc1,0x3d,
    0x6a,0x92,0xed,0xee,0xee,0xe6,0x36,0xc1,0x37,0x06,0xe0,0x3b,0x00,0x69,
    0xa9,0xc4,0x24,0x47,0x38,0x42,0xcd,0x4c,0x8d,0x42,0xa4,0x3c,0x10,0x08,
    0x11,0x00,0x00,0x00,0x24,0x77,0xd3,0xb4,0x93,0x50,0x75,0x41,0xb5,0x3d,
    0x70,0x3f,0xb2,0x56,0xfe,0xc6,0xe8,0x3a,0x55,0xc0,0xa5,0xea,0x81,0x3b,
    0x02,0x02,0x84,0xc4,0xc6,0x9e,0x16,0x42,0x00,0x00,0x00,0x42,0xab,0x3c,
    0xb0,0x00,0x18,0x00,0x00,0x00,0xcb,0x42,0x45,0x60,0xe4,0x0a,0x75,0x41,
    0x7e,0x79,0x0d,0x3e,0x8a,0xac,0xf3,0x97,0xce,0x46,0x35,0xc1,0x0c,0x77,
    0xe5,0x3b,0x00,0xfe,0xb9,0x44,0xa2,0x9f,0x38,0x42,0x9a,0x99,0x3c,0x42,
    0xc4,0x3c,0x10,0x08,0x18,0x00,0x00,0x00,0x30,0xdf,0xaf,0x58,0xe4,0x0a,
    0x75,0x41,0x9c,0xe5,0xa0,0x3f,0x40,0x1d,0xea,0xd2,0xc6,0x77,0xe5,0x40,
    0x6c,0xaf,0x57,0x3b,0xd3,0xed,0x90,0x44,0xd3,0x12,0x16,0x42,0x3d,0x0a,
    0xbf,0x41,0xcb,0x3c,0xb0,0x00,0x0e,0x00,0x00,0x00,0x1a,0xff,0x74,0xdf,
    0xb6,0x93,0x77,0x41,0xe9,0xe6,0x14,0x3e,0xeb,0x83,0x55,0xf8,0x02,0x44,
    0x35,0xc1,0x3a,0xea,0x01,0x3c,0x00,0xa1,0x96,0x44,0x47,0x07,0x35,0x42,
    0xb8,0x1e,0x4b,0x42,0xe4,0x3c,0x10,0x08,0x0e,0x00,0x00,0x00,0x6a,0xdb,
    0x85,0xd3,0xb6,0x93,0x77,0x41,0x46,0xed,0xa0,0x3f,0x79,0x67,0xa3,0x29,
    0x44,0x21,0xe3,0x40,0x31,0x70,0x54,0x3b,0x17,0xbf,0x6a,0x44,0x91,0x50,
    0x11,0x42,0x3d,0x0a,0xbf,0x41,0xeb,0x3c,0xb0,0x00,0x17,0x00,0x00,0x00,
    0x3e,0x36,0xb1,0x2e,0xab,0xe6,0x76,0x41,0xfb,0x75,0xce,0x3d,0xa8,0x48,
    0xae,0x50,0x5c,0xf8,0x31,0xc1,0xdb,0xf8,0xfd,0x3b,0x00,0xa5,0x44,0x45,
    0x73,0xcf,0x35,0x42,0xb8,0x9e,0x8f,0x42,0x64,0x3d,0x10,0x08,0x17,0x00,
    0x00,0x00,0x5f,0xa8,0xa2,0x08,0xab,0xe6,0x76,0x41,0xf0,0x94,0x65,0x3f,
    0x3b,0xa8,0xc4,0xf5,0xd1,0xc3,0xfc,0x40,0x47,0xab,0x99,0x3b,0xbf,0x3a,
    0x19,0x45,0x30,0x26,0x12,0x42,0x00,0x00,0x06,0x42,0x6b,0x3d,0xb0,0x00,
    0xe1,0xfa,0xa9,0x05 };

  /* The ASCII equivalent of above message
  #RANGEA,COM1,0,53.0,COARSESTEERING,1162,214875.000,00040008,dda7,167;
  12,
  3,0,24161981.444,0.164,-1581312.095,0.007,-2490.781,45.6,42.610,08103c24,
  3,0,24161990.254,7.538,-7143.416,0.011,-1940.862,34.3,3.980,00b03c2b,
  5,0,24380200.384,0.254,106100.298,0.008,2903.500,45.6,26.520,08103c84,
  5,0,24380200.072,3.009,21893.728,0.024,2262.467,34.7,9.980,01303c8b,
  17,0,22350141.218,0.094,-1500910.933,0.007,-1355.281,46.1,70.650,08103ca4,
  17,0,22350139.302,0.938,-84.920,0.004,-1056.063,37.7,32.000,00b03cab,
  24,0,22064710.017,0.138,-1394382.594,0.007,1487.938,46.2,47.150,08103cc4,
  24,0,22064709.543,1.257,43966.213,0.003,1159.432,37.5,23.880,00b03ccb,
  14,0,24722285.966,0.145,-1393666.970,0.008,1205.031,45.3,50.780,08103ce4,
  14,0,24722285.220,1.257,39178.130,0.003,938.986,36.3,23.880,00b03ceb,
  23,0,24013490.918,0.101,-1177692.315,0.008,3146.312,45.5,71.810,08103d64,
  23,0,24013488.540,0.897,117821.123,0.005,2451.672,36.5,33.500,00b03d6b*c43bd266
  */
  BOOL result;
  unsigned nrObs = 0;
  unsigned i = 0;

  NOVATELOEM4_structBinaryHeader header;
  NOVATELOEM4_structObservation obsArray[24];

  result = NOVATELOEM4_DecodeRANGEB(
    rangeb, 
    564,
    &header,
    obsArray,
    24,
    &nrObs );
  CU_ASSERT( result );
  CU_ASSERT( nrObs == 12 );

  CU_ASSERT( header.gpsWeek == 1162 );
  CU_ASSERT( header.gpsMilliSeconds == 214875000 );
  CU_ASSERT( header.eTimeStatus == NOVATELOEM4_TIMESTATUS_COARSESTEERING );
  CU_ASSERT( header.receiverStatus.isErrorIndicated == 0 );
  

  for( i = 0; i < nrObs; i++ )
    CU_ASSERT( obsArray[i].reserved == 0 );

  // 3,0,24161981.444,0.164,-1581312.095,0.007,-2490.781,45.6,42.610,08103c24,
  CU_ASSERT( obsArray[0].prn == 3 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[0].psr,    24161981.444, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[0].psrstd,        0.164, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[0].adr,    -1581312.095, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[0].adrstd,        0.007, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[0].doppler,   -2490.781, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[0].cno,            45.6, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[0].locktime,     42.610, 1E-03  );
  CU_ASSERT( obsArray[0].rawTrackingStatus == 0x08103c24 );
  CU_ASSERT( obsArray[0].trackingStatus.eTrackingState == NOVATELOEM4_L1PhaseLockLoop );
  CU_ASSERT( obsArray[0].trackingStatus.channelNumber == 1 );
  CU_ASSERT( obsArray[0].trackingStatus.isPhaseLocked == TRUE );
  CU_ASSERT( obsArray[0].trackingStatus.isParityKnown == TRUE );
  CU_ASSERT( obsArray[0].trackingStatus.isCodeLocked == TRUE );
  CU_ASSERT( obsArray[0].trackingStatus.eCorrelatorSpacing == NOVATELOEM4_OneChipSpacing );
  CU_ASSERT( obsArray[0].trackingStatus.eSatelliteSystem == NOVATELOEM4_GPSSystem );
  CU_ASSERT( obsArray[0].trackingStatus.isGrouped == TRUE );
  CU_ASSERT( obsArray[0].trackingStatus.eFrequency == NOVATELOEM4_L1 );
  CU_ASSERT( obsArray[0].trackingStatus.eCodeType == NOVATELOEM4_CACode );
  CU_ASSERT( obsArray[0].trackingStatus.isFECEnabled == FALSE );
  CU_ASSERT( obsArray[0].trackingStatus.isPrimaryL1Channel == TRUE );
  CU_ASSERT( obsArray[0].trackingStatus.isHalfCycleAdded == FALSE );
  CU_ASSERT( obsArray[0].trackingStatus.isForcedAssignment == FALSE );


  // 3,0,24161990.254,7.538,-7143.416,0.011,-1940.862,34.3,3.980,00b03c2b,
  CU_ASSERT( obsArray[1].prn == 3 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[1].psr,    24161990.254, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[1].psrstd,        7.538, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[1].adr,       -7143.416, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[1].adrstd,        0.011, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[1].doppler,   -1940.862, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[1].cno,            34.3, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[1].locktime,      3.980, 1E-03  );
  CU_ASSERT( obsArray[1].rawTrackingStatus == 0x00b03c2b );
  CU_ASSERT( obsArray[1].trackingStatus.eTrackingState == NOVATELOEM4_L2PhaseLockLoop );
  CU_ASSERT( obsArray[1].trackingStatus.channelNumber == 1 );
  CU_ASSERT( obsArray[1].trackingStatus.isPhaseLocked == TRUE );
  CU_ASSERT( obsArray[1].trackingStatus.isParityKnown == TRUE );
  CU_ASSERT( obsArray[1].trackingStatus.isCodeLocked == TRUE );
  CU_ASSERT( obsArray[1].trackingStatus.eCorrelatorSpacing == NOVATELOEM4_OneChipSpacing );
  CU_ASSERT( obsArray[1].trackingStatus.eSatelliteSystem == NOVATELOEM4_GPSSystem );
  CU_ASSERT( obsArray[1].trackingStatus.isGrouped == TRUE );
  CU_ASSERT( obsArray[1].trackingStatus.eFrequency == NOVATELOEM4_L2 );
  CU_ASSERT( obsArray[1].trackingStatus.eCodeType == NOVATELOEM4_PCode );
  CU_ASSERT( obsArray[1].trackingStatus.isFECEnabled == FALSE );
  CU_ASSERT( obsArray[1].trackingStatus.isPrimaryL1Channel == FALSE );
  CU_ASSERT( obsArray[1].trackingStatus.isHalfCycleAdded == FALSE );
  CU_ASSERT( obsArray[1].trackingStatus.isForcedAssignment == FALSE );

  //5,0,24380200.384,0.254,106100.298,0.008,2903.500,45.6,26.520,08103c84,
  CU_ASSERT( obsArray[2].prn == 5 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[2].psr,    24380200.384, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[2].psrstd,        0.254, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[2].adr,      106100.298, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[2].adrstd,        0.008, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[2].doppler,    2903.500, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[2].cno,            45.6, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[2].locktime,     26.520, 1E-03  );
  CU_ASSERT( obsArray[2].rawTrackingStatus == 0x08103c84 );

  //5,0,24380200.072,3.009,21893.728,0.024,2262.467,34.7,9.980,01303c8b,  
  CU_ASSERT( obsArray[3].prn == 5 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[3].psr,    24380200.072, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[3].psrstd,        3.009, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[3].adr,       21893.728, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[3].adrstd,        0.024, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[3].doppler,    2262.467, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[3].cno,            34.7, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[3].locktime,      9.980, 1E-03  );
  CU_ASSERT( obsArray[3].rawTrackingStatus == 0x01303c8b );

  //17,0,22350141.218,0.094,-1500910.933,0.007,-1355.281,46.1,70.650,08103ca4,
  CU_ASSERT( obsArray[4].prn == 17 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[4].psr,    22350141.218, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[4].psrstd,        0.094, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[4].adr,    -1500910.933, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[4].adrstd,        0.007, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[4].doppler,   -1355.281, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[4].cno,            46.1, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[4].locktime,     70.650, 1E-03  );
  CU_ASSERT( obsArray[4].rawTrackingStatus == 0x08103ca4 );

  //17,0,22350139.302,0.938,-84.920,0.004,-1056.063,37.7,32.000,00b03cab,
  CU_ASSERT( obsArray[5].prn == 17 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[5].psr,    22350139.302, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[5].psrstd,        0.938, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[5].adr,         -84.920, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[5].adrstd,        0.004, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[5].doppler,   -1056.063, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[5].cno,            37.7, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[5].locktime,     32.000, 1E-03  );
  CU_ASSERT( obsArray[5].rawTrackingStatus == 0x00b03cab );

  //24,0,22064710.017,0.138,-1394382.594,0.007,1487.938,46.2,47.150,08103cc4,
  CU_ASSERT( obsArray[6].prn == 24 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[6].psr,    22064710.017, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[6].psrstd,        0.138, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[6].adr,    -1394382.594, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[6].adrstd,        0.007, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[6].doppler,    1487.938, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[6].cno,            46.2, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[6].locktime,     47.150, 1E-03  );
  CU_ASSERT( obsArray[6].rawTrackingStatus == 0x08103cc4 );

  //24,0,22064709.543,1.257,43966.213,0.003,1159.432,37.5,23.880,00b03ccb,
  CU_ASSERT( obsArray[7].prn == 24 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[7].psr,    22064709.543, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[7].psrstd,        1.257, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[7].adr,       43966.213, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[7].adrstd,        0.003, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[7].doppler,    1159.432, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[7].cno,            37.5, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[7].locktime,     23.880, 1E-03  );
  CU_ASSERT( obsArray[7].rawTrackingStatus == 0x00b03ccb );

  //14,0,24722285.966,0.145,-1393666.970,0.008,1205.031,45.3,50.780,08103ce4,
  CU_ASSERT( obsArray[8].prn == 14 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[8].psr,    24722285.966, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[8].psrstd,        0.145, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[8].adr,    -1393666.970, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[8].adrstd,        0.008, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[8].doppler,    1205.031, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[8].cno,            45.3, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[8].locktime,     50.780, 1E-03  );
  CU_ASSERT( obsArray[8].rawTrackingStatus == 0x08103ce4 );

  //14,0,24722285.220,1.257,39178.130,0.003,938.986,36.3,23.880,00b03ceb,
  CU_ASSERT( obsArray[9].prn == 14 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[9].psr,    24722285.220, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[9].psrstd,        1.257, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[9].adr,       39178.130, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[9].adrstd,        0.003, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[9].doppler,     938.986, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[9].cno,            36.3, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[9].locktime,     23.880, 1E-03  );
  CU_ASSERT( obsArray[9].rawTrackingStatus == 0x00b03ceb );

  //23,0,24013490.918,0.101,-1177692.315,0.008,3146.312,45.5,71.810,08103d64,
  CU_ASSERT( obsArray[10].prn == 23 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[10].psr,    24013490.918, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[10].psrstd,        0.101, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[10].adr,    -1177692.315, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[10].adrstd,        0.008, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[10].doppler,    3146.312, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[10].cno,            45.5, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[10].locktime,     71.810, 1E-03  );
  CU_ASSERT( obsArray[10].rawTrackingStatus == 0x08103d64 );
  CU_ASSERT( obsArray[10].trackingStatus.eTrackingState == NOVATELOEM4_L1PhaseLockLoop );
  CU_ASSERT( obsArray[10].trackingStatus.channelNumber == 11 );
  CU_ASSERT( obsArray[10].trackingStatus.isPhaseLocked == TRUE );
  CU_ASSERT( obsArray[10].trackingStatus.isParityKnown == TRUE );
  CU_ASSERT( obsArray[10].trackingStatus.isCodeLocked == TRUE );
  CU_ASSERT( obsArray[10].trackingStatus.eCorrelatorSpacing == NOVATELOEM4_OneChipSpacing );
  CU_ASSERT( obsArray[10].trackingStatus.eSatelliteSystem == NOVATELOEM4_GPSSystem );
  CU_ASSERT( obsArray[10].trackingStatus.isGrouped == TRUE );
  CU_ASSERT( obsArray[10].trackingStatus.eFrequency == NOVATELOEM4_L1 );
  CU_ASSERT( obsArray[10].trackingStatus.eCodeType == NOVATELOEM4_CACode );
  CU_ASSERT( obsArray[10].trackingStatus.isFECEnabled == FALSE );
  CU_ASSERT( obsArray[10].trackingStatus.isPrimaryL1Channel == TRUE );
  CU_ASSERT( obsArray[10].trackingStatus.isHalfCycleAdded == FALSE );
  CU_ASSERT( obsArray[10].trackingStatus.isForcedAssignment == FALSE );


  //23,0,24013488.540,0.897,117821.123,0.005,2451.672,36.5,33.500,00b03d6b
  CU_ASSERT( obsArray[11].prn == 23 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[11].psr,    24013488.540, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[11].psrstd,        0.897, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[11].adr,      117821.123, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[11].adrstd,        0.005, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[11].doppler,    2451.672, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[11].cno,            36.5, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[11].locktime,     33.500, 1E-03  );
  CU_ASSERT( obsArray[11].rawTrackingStatus == 0x00b03d6b );
  CU_ASSERT( obsArray[11].trackingStatus.eTrackingState == NOVATELOEM4_L2PhaseLockLoop );
  CU_ASSERT( obsArray[11].trackingStatus.channelNumber == 11 );
  CU_ASSERT( obsArray[11].trackingStatus.isPhaseLocked == TRUE );
  CU_ASSERT( obsArray[11].trackingStatus.isParityKnown == TRUE );
  CU_ASSERT( obsArray[11].trackingStatus.isCodeLocked == TRUE );
  CU_ASSERT( obsArray[11].trackingStatus.eCorrelatorSpacing == NOVATELOEM4_OneChipSpacing );
  CU_ASSERT( obsArray[11].trackingStatus.eSatelliteSystem == NOVATELOEM4_GPSSystem );
  CU_ASSERT( obsArray[11].trackingStatus.isGrouped == TRUE );
  CU_ASSERT( obsArray[11].trackingStatus.eFrequency == NOVATELOEM4_L2 );
  CU_ASSERT( obsArray[11].trackingStatus.eCodeType == NOVATELOEM4_PCode );
  CU_ASSERT( obsArray[11].trackingStatus.isFECEnabled == FALSE );
  CU_ASSERT( obsArray[11].trackingStatus.isPrimaryL1Channel == FALSE );
  CU_ASSERT( obsArray[11].trackingStatus.isHalfCycleAdded == FALSE );
  CU_ASSERT( obsArray[11].trackingStatus.isForcedAssignment == FALSE );
}



void test_NOVATELOEM4_DecodeRANGECMPB(void)
{
  unsigned char rangecmpb[324] = {
    0xaa,0x44,0x12,0x1c,0x8c,0x00,0x02,0x20,0x24,0x01,0x00,0x00,0x6a,0x78,0x8a,0x04,
    0x78,0xbb,0xce,0x0c,0x08,0x00,0x04,0x00,0xa7,0xdd,0xa7,0x00,0x0c,0x00,0x00,0x00,
    0x24,0x3c,0x10,0x08,0x38,0x45,0xf6,0x8f,0xeb,0x75,0x85,0x0b,0xe7,0xff,0xde,0x67,
    0x23,0x03,0x53,0x05,0x20,0x03,0x00,0x00,0x2b,0x3c,0xb0,0x00,0x24,0x6b,0xf8,0x0f,
    0x32,0x76,0x85,0x0b,0x95,0x18,0xe4,0x7f,0x4b,0x03,0x7f,0x00,0xc0,0x01,0x00,0x00,
    0x84,0x3c,0x10,0x08,0x80,0x57,0x0b,0x10,0x43,0x19,0xa0,0x0b,0x4c,0x74,0x9e,0x81,
    0x25,0x05,0x50,0x03,0x20,0x03,0x00,0x00,0x8b,0x3c,0x30,0x01,0x77,0xd6,0x08,0x90,
    0x40,0x19,0xa0,0x0b,0xba,0x85,0x55,0x80,0xba,0x05,0x3f,0x01,0xc0,0x01,0x00,0x00,
    0xa4,0x3c,0x10,0x08,0xb8,0xb4,0xfa,0xbf,0xe9,0x49,0xa8,0x0a,0x11,0x11,0x19,0xe9,
    0x22,0x11,0xd4,0x08,0x40,0x03,0x00,0x00,0xab,0x3c,0xb0,0x00,0xf0,0xdf,0xfb,0x6f,
    0xda,0x49,0xa8,0x0a,0x14,0xab,0xff,0xff,0x18,0x11,0x00,0x04,0x20,0x02,0x00,0x00,
    0xc4,0x3c,0x10,0x08,0xf0,0xcf,0x05,0x20,0x30,0x72,0x85,0x0a,0x68,0x31,0xb9,0x6a,
    0x23,0x18,0xe4,0x05,0x40,0x03,0x00,0x00,0xcb,0x3c,0xb0,0x00,0x6e,0x87,0x04,0x50,
    0x2c,0x72,0x85,0x0a,0x36,0xbe,0xab,0x00,0x08,0x18,0xfc,0x02,0x20,0x02,0x00,0x00,
    0xe4,0x3c,0x10,0x08,0x08,0xb5,0x04,0xb0,0x6f,0xdb,0xc9,0x0b,0x07,0xfd,0xbb,0x6a,
    0x33,0x0e,0x58,0x06,0x20,0x03,0x00,0x00,0xeb,0x3c,0xb0,0x00,0xfc,0xaa,0x03,0xc0,
    0x69,0xdb,0xc9,0x0b,0x21,0x0a,0x99,0x00,0x08,0x0e,0xfc,0x02,0x00,0x02,0x00,0x00,
    0x64,0x3d,0x10,0x08,0x50,0x4a,0x0c,0x50,0x97,0x55,0x73,0x0b,0xaf,0xa3,0x07,0x6e,
    0x22,0x17,0xf9,0x08,0x20,0x03,0x00,0x00,0x6b,0x3d,0xb0,0x00,0xab,0x93,0x09,0x50,
    0x84,0x55,0x73,0x0b,0x1f,0x3d,0xcc,0x81,0x18,0x17,0x30,0x04,0x00,0x02,0x00,0x00,
    0xc6,0x36,0x8c,0x99 };

  /* The ASCII equivalent of above message
  #RANGEA,COM1,0,53.0,COARSESTEERING,1162,214875.000,00040008,dda7,167;
  12,
  3,0,24161981.438,0.169,-127410432.098,0.006,-2490.781,45.0,42.594,08103c24,
  3,0,24161990.250,9.500,-100670439.418,0.010,-1940.859,34.0,3.969,00b03c2b,
  5,0,24380200.383,0.380,-125723019.703,0.006,2903.500,45.0,26.500,08103c84,
  5,0,24380200.070,4.750,-100641402.273,0.023,2262.465,34.0,9.969,01303c8b,
  17,0,22350141.211,0.113,-118941422.934,0.006,-1355.281,46.0,70.625,08103ca4,
  17,0,22350139.297,1.281,-92274772.922,0.004,-1056.062,37.0,32.000,00b03cab,
  24,0,22064710.016,0.169,-118834894.594,0.006,1487.938,46.0,47.125,08103cc4,
  24,0,22064709.539,1.281,-92230721.789,0.002,1159.430,37.0,23.875,00b03ccb,
  14,0,24722285.961,0.169,-127222786.973,0.008,1205.031,45.0,50.750,08103ce4,
  14,0,24722285.219,1.281,-100624117.871,0.002,938.984,36.0,23.875,00b03ceb,
  23,0,24013490.914,0.113,-127006812.316,0.006,3146.312,45.0,71.781,08103d64,
  23,0,24013488.539,1.281,-100545474.879,0.004,2451.668,36.0,33.500,00b03d6b*cf846d48

  #RANGECMPA,COM1,0,53.0,COARSESTEERING,1162,214875.000,00040008,dda7,167;12,
  243c10083845f68feb75850be7ffde672303530520030000,
  2b3cb000246bf80f3276850b9518e47f4b037f00c0010000,
  843c100880570b104319a00b4c749e812505500320030000,
  8b3c300177d608904019a00bba855580ba053f01c0010000,
  a43c1008b8b4fabfe949a80a111119e92211d40840030000,
  ab3cb000f0dffb6fda49a80a14abffff1811000420020000,
  c43c1008f0cf05203072850a6831b96a2318e40540030000,
  cb3cb0006e8704502c72850a36beab000818fc0220020000,
  e43c100808b504b06fdbc90b07fdbb6a330e580620030000,
  eb3cb000fcaa03c069dbc90b210a9900080efc0200020000,
  643d1008504a0c509755730bafa3076e2217f90820030000,
  6b3db000ab9309508455730b1f3dcc811817300400020000*5779f4c5
  */
  BOOL result;
  unsigned nrObs = 0;
  unsigned i = 0;

  NOVATELOEM4_structBinaryHeader header;
  NOVATELOEM4_structObservation obsArray[24];

  result = NOVATELOEM4_DecodeRANGECMPB(
    rangecmpb, 
    324,
    &header,
    obsArray,
    24,
    &nrObs );
  CU_ASSERT( result );
  CU_ASSERT( nrObs == 12 );

  CU_ASSERT( header.gpsWeek == 1162 );
  CU_ASSERT( header.gpsMilliSeconds == 214875000 );
  CU_ASSERT( header.eTimeStatus == NOVATELOEM4_TIMESTATUS_COARSESTEERING );
  CU_ASSERT( header.receiverStatus.isErrorIndicated == 0 );
  

  for( i = 0; i < nrObs; i++ )
    CU_ASSERT( obsArray[i].reserved == 0 );

  // 3,0,24161981.438,0.169,-127410432.098,0.006,-2490.781,45.0,42.594,08103c24,
  CU_ASSERT( obsArray[0].prn == 3 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[0].psr,    24161981.438, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[0].psrstd,        0.169, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[0].adr,  -127410432.098, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[0].adrstd,        0.006, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[0].doppler,   -2490.781, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[0].cno,            45.0, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[0].locktime,     42.594, 1E-03  );
  CU_ASSERT( obsArray[0].rawTrackingStatus == 0x08103c24 );
  CU_ASSERT( obsArray[0].trackingStatus.eTrackingState == NOVATELOEM4_L1PhaseLockLoop );
  CU_ASSERT( obsArray[0].trackingStatus.channelNumber == 1 );
  CU_ASSERT( obsArray[0].trackingStatus.isPhaseLocked == TRUE );
  CU_ASSERT( obsArray[0].trackingStatus.isParityKnown == TRUE );
  CU_ASSERT( obsArray[0].trackingStatus.isCodeLocked == TRUE );
  CU_ASSERT( obsArray[0].trackingStatus.eCorrelatorSpacing == NOVATELOEM4_OneChipSpacing );
  CU_ASSERT( obsArray[0].trackingStatus.eSatelliteSystem == NOVATELOEM4_GPSSystem );
  CU_ASSERT( obsArray[0].trackingStatus.isGrouped == TRUE );
  CU_ASSERT( obsArray[0].trackingStatus.eFrequency == NOVATELOEM4_L1 );
  CU_ASSERT( obsArray[0].trackingStatus.eCodeType == NOVATELOEM4_CACode );
  CU_ASSERT( obsArray[0].trackingStatus.isFECEnabled == FALSE );
  CU_ASSERT( obsArray[0].trackingStatus.isPrimaryL1Channel == TRUE );
  CU_ASSERT( obsArray[0].trackingStatus.isHalfCycleAdded == FALSE );
  CU_ASSERT( obsArray[0].trackingStatus.isForcedAssignment == FALSE );


  // 3,0,24161990.250,9.500,-100670439.418,0.010,-1940.859,34.0,3.969,00b03c2b,
  CU_ASSERT( obsArray[1].prn == 3 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[1].psr,    24161990.250, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[1].psrstd,        9.500, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[1].adr,  -100670439.418, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[1].adrstd,        0.010, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[1].doppler,   -1940.859, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[1].cno,            34.0, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[1].locktime,      3.969, 1E-03  );
  CU_ASSERT( obsArray[1].rawTrackingStatus == 0x00b03c2b );
  CU_ASSERT( obsArray[1].trackingStatus.eTrackingState == NOVATELOEM4_L2PhaseLockLoop );
  CU_ASSERT( obsArray[1].trackingStatus.channelNumber == 1 );
  CU_ASSERT( obsArray[1].trackingStatus.isPhaseLocked == TRUE );
  CU_ASSERT( obsArray[1].trackingStatus.isParityKnown == TRUE );
  CU_ASSERT( obsArray[1].trackingStatus.isCodeLocked == TRUE );
  CU_ASSERT( obsArray[1].trackingStatus.eCorrelatorSpacing == NOVATELOEM4_OneChipSpacing );
  CU_ASSERT( obsArray[1].trackingStatus.eSatelliteSystem == NOVATELOEM4_GPSSystem );
  CU_ASSERT( obsArray[1].trackingStatus.isGrouped == TRUE );
  CU_ASSERT( obsArray[1].trackingStatus.eFrequency == NOVATELOEM4_L2 );
  CU_ASSERT( obsArray[1].trackingStatus.eCodeType == NOVATELOEM4_PCode );
  CU_ASSERT( obsArray[1].trackingStatus.isFECEnabled == FALSE );
  CU_ASSERT( obsArray[1].trackingStatus.isPrimaryL1Channel == FALSE );
  CU_ASSERT( obsArray[1].trackingStatus.isHalfCycleAdded == FALSE );
  CU_ASSERT( obsArray[1].trackingStatus.isForcedAssignment == FALSE );

  //5,0,24380200.383,0.380,-125723019.703,0.006,2903.500,45.0,26.500,08103c84,
  CU_ASSERT( obsArray[2].prn == 5 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[2].psr,    24380200.383, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[2].psrstd,        0.380, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[2].adr,  -125723019.703, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[2].adrstd,        0.006, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[2].doppler,    2903.500, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[2].cno,            45.0, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[2].locktime,     26.500, 1E-03  );
  CU_ASSERT( obsArray[2].rawTrackingStatus == 0x08103c84 );

  //5,0,24380200.070,4.750,-100641402.273,0.023,2262.465,34.0,9.969,01303c8b,
  CU_ASSERT( obsArray[3].prn == 5 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[3].psr,    24380200.070, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[3].psrstd,        4.750, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[3].adr,  -100641402.273, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[3].adrstd,        0.023, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[3].doppler,    2262.465, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[3].cno,            34.0, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[3].locktime,      9.969, 1E-03  );
  CU_ASSERT( obsArray[3].rawTrackingStatus == 0x01303c8b );

  //17,0,22350141.211,0.113,-118941422.934,0.006,-1355.281,46.0,70.625,08103ca4,
  CU_ASSERT( obsArray[4].prn == 17 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[4].psr,    22350141.211, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[4].psrstd,        0.113, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[4].adr,  -118941422.934, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[4].adrstd,        0.006, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[4].doppler,   -1355.281, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[4].cno,            46.0, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[4].locktime,     70.625, 1E-03  );
  CU_ASSERT( obsArray[4].rawTrackingStatus == 0x08103ca4 );

  //17,0,22350139.297,1.281,-92274772.922,0.004,-1056.062,37.0,32.000,00b03cab,
  CU_ASSERT( obsArray[5].prn == 17 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[5].psr,    22350139.297, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[5].psrstd,        1.281, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[5].adr,   -92274772.922, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[5].adrstd,        0.004, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[5].doppler,   -1056.062, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[5].cno,            37.0, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[5].locktime,     32.000, 1E-03  );
  CU_ASSERT( obsArray[5].rawTrackingStatus == 0x00b03cab );

  //24,0,22064710.016,0.169,-118834894.594,0.006,1487.938,46.0,47.125,08103cc4,
  CU_ASSERT( obsArray[6].prn == 24 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[6].psr,    22064710.016, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[6].psrstd,        0.169, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[6].adr,  -118834894.594, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[6].adrstd,        0.006, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[6].doppler,    1487.938, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[6].cno,            46.0, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[6].locktime,     47.125, 1E-03  );
  CU_ASSERT( obsArray[6].rawTrackingStatus == 0x08103cc4 );

  //24,0,22064709.539,1.281,-92230721.789,0.002,1159.430,37.0,23.875,00b03ccb,
  CU_ASSERT( obsArray[7].prn == 24 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[7].psr,    22064709.539, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[7].psrstd,        1.281, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[7].adr,   -92230721.789, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[7].adrstd,        0.002, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[7].doppler,    1159.430, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[7].cno,            37.0, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[7].locktime,     23.875, 1E-03  );
  CU_ASSERT( obsArray[7].rawTrackingStatus == 0x00b03ccb );

  //14,0,24722285.961,0.169,-127222786.973,0.008,1205.031,45.0,50.750,08103ce4,
  CU_ASSERT( obsArray[8].prn == 14 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[8].psr,    24722285.961, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[8].psrstd,        0.169, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[8].adr,  -127222786.973, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[8].adrstd,        0.008, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[8].doppler,    1205.031, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[8].cno,            45.0, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[8].locktime,     50.750, 1E-03  );
  CU_ASSERT( obsArray[8].rawTrackingStatus == 0x08103ce4 );

  //14,0,24722285.219,1.281,-100624117.871,0.002,938.984,36.0,23.875,00b03ceb,
  CU_ASSERT( obsArray[9].prn == 14 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[9].psr,    24722285.219, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[9].psrstd,        1.281, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[9].adr,  -100624117.871, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[9].adrstd,        0.002, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[9].doppler,     938.984, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[9].cno,            36.0, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[9].locktime,     23.875, 1E-03  );
  CU_ASSERT( obsArray[9].rawTrackingStatus == 0x00b03ceb );

  //23,0,24013490.914,0.113,-127006812.316,0.006,3146.312,45.0,71.781,08103d64,
  CU_ASSERT( obsArray[10].prn == 23 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[10].psr,    24013490.914, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[10].psrstd,        0.113, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[10].adr,  -127006812.316, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[10].adrstd,        0.006, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[10].doppler,    3146.312, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[10].cno,            45.0, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[10].locktime,     71.781, 1E-03  );
  CU_ASSERT( obsArray[10].rawTrackingStatus == 0x08103d64 );
  CU_ASSERT( obsArray[10].trackingStatus.eTrackingState == NOVATELOEM4_L1PhaseLockLoop );
  CU_ASSERT( obsArray[10].trackingStatus.channelNumber == 11 );
  CU_ASSERT( obsArray[10].trackingStatus.isPhaseLocked == TRUE );
  CU_ASSERT( obsArray[10].trackingStatus.isParityKnown == TRUE );
  CU_ASSERT( obsArray[10].trackingStatus.isCodeLocked == TRUE );
  CU_ASSERT( obsArray[10].trackingStatus.eCorrelatorSpacing == NOVATELOEM4_OneChipSpacing );
  CU_ASSERT( obsArray[10].trackingStatus.eSatelliteSystem == NOVATELOEM4_GPSSystem );
  CU_ASSERT( obsArray[10].trackingStatus.isGrouped == TRUE );
  CU_ASSERT( obsArray[10].trackingStatus.eFrequency == NOVATELOEM4_L1 );
  CU_ASSERT( obsArray[10].trackingStatus.eCodeType == NOVATELOEM4_CACode );
  CU_ASSERT( obsArray[10].trackingStatus.isFECEnabled == FALSE );
  CU_ASSERT( obsArray[10].trackingStatus.isPrimaryL1Channel == TRUE );
  CU_ASSERT( obsArray[10].trackingStatus.isHalfCycleAdded == FALSE );
  CU_ASSERT( obsArray[10].trackingStatus.isForcedAssignment == FALSE );


  //23,0,24013488.539,1.281,-100545474.879,0.004,2451.668,36.0,33.500,00b03d6b
  CU_ASSERT( obsArray[11].prn == 23 );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[11].psr,    24013488.539, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[11].psrstd,        1.281, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[11].adr,  -100545474.879, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[11].adrstd,        0.004, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[11].doppler,    2451.668, 1E-03  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[11].cno,            36.0, 1E-01  );
  CU_ASSERT_DOUBLE_EQUAL( obsArray[11].locktime,     33.500, 1E-03  );
  CU_ASSERT( obsArray[11].rawTrackingStatus == 0x00b03d6b );
  CU_ASSERT( obsArray[11].trackingStatus.eTrackingState == NOVATELOEM4_L2PhaseLockLoop );
  CU_ASSERT( obsArray[11].trackingStatus.channelNumber == 11 );
  CU_ASSERT( obsArray[11].trackingStatus.isPhaseLocked == TRUE );
  CU_ASSERT( obsArray[11].trackingStatus.isParityKnown == TRUE );
  CU_ASSERT( obsArray[11].trackingStatus.isCodeLocked == TRUE );
  CU_ASSERT( obsArray[11].trackingStatus.eCorrelatorSpacing == NOVATELOEM4_OneChipSpacing );
  CU_ASSERT( obsArray[11].trackingStatus.eSatelliteSystem == NOVATELOEM4_GPSSystem );
  CU_ASSERT( obsArray[11].trackingStatus.isGrouped == TRUE );
  CU_ASSERT( obsArray[11].trackingStatus.eFrequency == NOVATELOEM4_L2 );
  CU_ASSERT( obsArray[11].trackingStatus.eCodeType == NOVATELOEM4_PCode );
  CU_ASSERT( obsArray[11].trackingStatus.isFECEnabled == FALSE );
  CU_ASSERT( obsArray[11].trackingStatus.isPrimaryL1Channel == FALSE );
  CU_ASSERT( obsArray[11].trackingStatus.isHalfCycleAdded == FALSE );
  CU_ASSERT( obsArray[11].trackingStatus.isForcedAssignment == FALSE );

}




void test_NOVATELOEM4_DecodeRAWEPHEMB(void)
{
  FILE *fid = NULL;
  BOOL result = FALSE;
  unsigned char message[8192];
  BOOL wasMessageFound = FALSE;
  BOOL wasEndOfFileReached = FALSE;
  unsigned filePosition = 0;
  unsigned short messageLength = 0;
  unsigned short messageID = 0;
  unsigned numberBadCRC = 0;
  unsigned reference_week;
  unsigned reference_time;
  unsigned prn;
  unsigned tow;
  unsigned i = 0;
  GPS_structEphemeris ephArray[128]; // an array of GPS ephemeris
  NOVATELOEM4_structBinaryHeader header;  

  result = fopen_s( &fid, "rawephemb.bin", "rb" );
  CU_ASSERT_FATAL( result == 0 );

  if( result == 0 )
    return;

  while( !wasEndOfFileReached && i < 128 )
  {
    result = NOVATELOEM4_FindNextMessageInFile(
      fid,
      message,
      8192, 
      &wasEndOfFileReached,
      &wasMessageFound,
      &filePosition,
      &messageLength,
      &messageID,
      &numberBadCRC );
    CU_ASSERT( result );

    if( messageID == 41 ) // RAWEPHEMB
    {
      result = NOVATELOEM4_DecodeRAWEPHEMB(
        message,
        messageLength,
        &header,
        &prn,
        &reference_week,
        &reference_time,
        &tow,
        &ephArray[i].iodc,
        &ephArray[i].iode,  
        &ephArray[i].toe,   
        &ephArray[i].toc,   
        &ephArray[i].week,  
        &ephArray[i].health,
        &ephArray[i].alert_flag,
        &ephArray[i].anti_spoof,
        &ephArray[i].code_on_L2,
        &ephArray[i].ura,       
        &ephArray[i].L2_P_data_flag,
        &ephArray[i].fit_interval_flag,
        &ephArray[i].age_of_data_offset, 
        &ephArray[i].tgd,                
        &ephArray[i].af2,                
        &ephArray[i].af1,                
        &ephArray[i].af0,                
        &ephArray[i].m0,                
        &ephArray[i].delta_n,           
        &ephArray[i].ecc,               
        &ephArray[i].sqrta,             
        &ephArray[i].omega0,            
        &ephArray[i].i0,                
        &ephArray[i].w,                 
        &ephArray[i].omegadot,          
        &ephArray[i].idot,              
        &ephArray[i].cuc,               
        &ephArray[i].cus,               
        &ephArray[i].crc,               
        &ephArray[i].crs,               
        &ephArray[i].cic,               
        &ephArray[i].cis                
        );
      CU_ASSERT( result );
      ephArray[i].prn = (unsigned short)prn;

      if( i == 0 )
      {
        CU_ASSERT( ephArray[i].prn == 14 );
        CU_ASSERT( reference_week == 1186 );
        CU_ASSERT( reference_time == 50384 );
        CU_ASSERT( ephArray[i].iodc == 160 );
        CU_ASSERT( ephArray[i].iode == 160 );
        CU_ASSERT( ephArray[i].toe == 50384 );
        CU_ASSERT( ephArray[i].toc == 50384 );   
        CU_ASSERT( ephArray[i].week == 162 );
        CU_ASSERT( ephArray[i].health == 0 );
        CU_ASSERT( ephArray[i].alert_flag == 0 );
        CU_ASSERT( ephArray[i].anti_spoof == 1 );
        CU_ASSERT( ephArray[i].code_on_L2 == 1 );
        CU_ASSERT( ephArray[i].ura == 0 );
        CU_ASSERT( ephArray[i].L2_P_data_flag == 0 );
        CU_ASSERT( ephArray[i].fit_interval_flag == 0 );
        CU_ASSERT( ephArray[i].age_of_data_offset == 7200 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].tgd, -1.024455e-008, 1E-14 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].af2, 0.0, 1E-03 );               
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].af1, 1.591616e-012, 1E-18 );        
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].af0, -4.7015957e-005, 1E-11 ); 
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].m0, 1.251492, 1E-06 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].delta_n, 4.296965e-009, 1E-15 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].ecc, 1.542772e-003, 1E-09 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].sqrta, 5.153621e+003, 1E-03 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].omega0, 1.199630, 1E-06 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].i0, 9.692661e-001, 1E-07 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].w, -7.577131e-001, 1E-07 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].omegadot, -7.920330e-009, 1E-15 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].idot, 1.221479e-010, 1E-16 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].cuc, -4.442409e-006, 1E-12 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].cus, 7.834285e-006, 1E-12 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].crc, 2.326875e+002, 1E-04 ); 
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].crs, -8.746875e+001, 1E-05 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].cic, 3.166497e-008, 1E-14 );
        CU_ASSERT_DOUBLE_EQUAL( ephArray[i].cis, -1.303852e-008, 1E-14 );
      }
      i++;
    }
  }
  if( fid == NULL )
    fclose( fid );
}
