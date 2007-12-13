/**
\file    GNSS_RxData.cpp
\brief   The implementation file for the GNSS_RxData class.

\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
\since   2006-11-13

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

#include <memory.h>
#include <math.h>

#include "GNSS_RxData.h"
#include "novatel.h" 
#include "constants.h"
#include "geodesy.h"
#include "Matrix.h"

using namespace Zenautics; // for Matrix


#define GPS_NUMBER_VALID_PRNS (64)

#ifndef SECONDS_IN_DAY
#define SECONDS_IN_DAY (86400.0)
#endif

#ifndef SECONDS_IN_WEEK
#define SECONDS_IN_WEEK (604800.0)
#endif


#ifndef WIN32
#define _CRT_SECURE_NO_DEPRECATE
#endif


namespace GNSS
{

  GPS_BroadcastEphemerisAndAlmanacArray::GPS_BroadcastEphemerisAndAlmanacArray()
  : m_array(NULL),
    m_arrayLength(0)
  { 
  }


  GPS_BroadcastEphemerisAndAlmanacArray::~GPS_BroadcastEphemerisAndAlmanacArray()
  {
    if( m_array != NULL )
    {
      delete[] m_array;
      m_arrayLength = 0;
    }
  }

  bool GPS_BroadcastEphemerisAndAlmanacArray::AllocateArray()
  {
    unsigned i = 0;
    if( m_array != NULL )
    {
      if( m_arrayLength == GPS_NUMBER_VALID_PRNS )
        return true; // already allocated
      else
        return false; // error
    }
    m_array = new GPS_structOrbitParameters[GPS_NUMBER_VALID_PRNS];
    if( m_array == NULL )
      return false;
    m_arrayLength = GPS_NUMBER_VALID_PRNS;

    // Initialize all to zero.
    for( i = 0; i < GPS_NUMBER_VALID_PRNS; i++ )
    {
      memset( &m_array[i], 0, sizeof(GPS_structOrbitParameters) );
    }

    return true;
  }


  bool GPS_BroadcastEphemerisAndAlmanacArray::AddEphemeris( const unsigned short prn, const GPS_structEphemeris &eph )
  {
    unsigned short index = 0;

    if( m_arrayLength == 0 )
    {
      if( !AllocateArray() )
        return false;
    }

    if( !GetIndexGivenPRN( prn, index ) )
      return false;

    // The previous ephemeris is set based on what was the current prior this update.
    m_array[index].previousEph = m_array[index].currentEph;
    m_array[index].currentEph  = eph;

    return true;
  }

  bool GPS_BroadcastEphemerisAndAlmanacArray::AddAlmanac( const unsigned short prn, const GPS_structAlmanac &alm )
  {
    unsigned short index = 0;

    if( m_arrayLength == 0 )
    {
      if( !AllocateArray() )
        return false;
    }

    if( !GetIndexGivenPRN( prn, index ) )
      return false;
    
    m_array[index].almanac = alm;

    return true;
  }

  bool GPS_BroadcastEphemerisAndAlmanacArray::GetEphemeris( 
    const unsigned short prn, //!< The desired GPS PRN. (1-32 GPS, 120-138 SBAS).
    GPS_structEphemeris &eph, //!< A reference to an ephemeris struct in which to store the data.
    bool &isAvailable,        //!< This boolean indicates if ephemeris data is available or not.
    char iode                 //!< The issue of data for the ephemeris, -1 means get the most current.
    )
  {
    unsigned short index = 0;

    if( m_arrayLength == 0 )
    {
      isAvailable = false;
      return true;
    }
    
    if( !GetIndexGivenPRN( prn, index ) )
      return false;

    // check the prn to see if any ephemeris information is available.
    if( m_array[index].currentEph.prn == 0 )
    {
      isAvailable = false;
      return true;
    }

    isAvailable = true;
    if( iode == -1 )
    {
      eph = m_array[index].currentEph;      
    }
    else
    {
      if( m_array[index].currentEph.iode == iode )
      {
        eph = m_array[index].currentEph;
      }
      else if( m_array[index].previousEph.iode == iode )
      {
        eph = m_array[index].previousEph;
      }
      else
      {
        isAvailable = false;
      }
    }
    return true;
  }

  bool GPS_BroadcastEphemerisAndAlmanacArray::GetEphemerisTOW( 
    const unsigned short prn, //!< The desired GPS PRN. (1-32 GPS, 120-138 SBAS).
    bool &isAvailable,        //!< This boolean indicates if ephemeris data is available or not.
    unsigned short &week,     //!< The correct week corresponding to the time of week based on the Z-count in the Hand Over Word.
    unsigned &tow             //!< The time of week based on the Z-count in the Hand Over Word.
    )
  {
    unsigned short index = 0;
    unsigned short eph_week = 0;
    int eph_toe = 0;
    int eph_tow = 0;

    if( m_arrayLength == 0 )
    {
      isAvailable = false;
      return true;
    }
    
    if( !GetIndexGivenPRN( prn, index ) )
      return false;

    // check the prn to see if any ephemeris information is available.
    if( m_array[index].currentEph.prn == 0 )
    {
      isAvailable = false;
      return true;
    }

    isAvailable = true;
    if( isAvailable )
    {
      eph_toe  = (int)m_array[index].currentEph.toe;
      eph_week = m_array[index].currentEph.week;
      eph_tow  = (int)m_array[index].currentEph.tow;

      // check for week rolloever condition, 
      // if the tow of week is different by more than four days 
      // compared to the time of ephemeris, then the tow is in the next week
      if( (eph_tow - eph_toe) < (-4*86400) )
      {
        eph_week++;
      }

      week = eph_week;
      tow  = eph_tow;
    }
    else
    {
      week = 0;
      tow = 0;
    }
    return true;

  }

  bool GPS_BroadcastEphemerisAndAlmanacArray::IsEphemerisAvailable( 
    const unsigned short prn, //!< The desired GPS PRN. (1-32 GPS, 120-138 SBAS).
    bool &isAvailable,        //!< This boolean indicates if ephemeris data is available or not.
    char iode                 //!< The issue of data for the ephemeris, -1 means get the most current.
    )
  {
    unsigned short index = 0;

    // Check if there is any data in the array.
    if( m_arrayLength == 0 )
    { 
      isAvailable = false;
      return true;
    }
    
    if( !GetIndexGivenPRN( prn, index ) )
      return false;

    // check the prn to see if any ephemeris information is available.
    if( m_array[index].currentEph.prn == 0 )
    {
      isAvailable = false;
      return true;
    }

    if( iode == -1 )
    {
      isAvailable = true;
    }
    else
    {
      if( m_array[index].currentEph.iode == iode )
      {
        isAvailable = true;
      }
      else if( m_array[index].previousEph.iode == iode )
      {
        isAvailable = true;
      }
      else
      {
        isAvailable = false;
      }
    }
    return true;
  }



  bool GPS_BroadcastEphemerisAndAlmanacArray::GetIndexGivenPRN( const unsigned short prn, unsigned short &index )
  {
    // GPS 1-32
    // Pseudolites are 33-37
    // SBAS is 120-138
    // WAAS, EGNOS, MSAS
    //
    // WAAS:
    // AOR-W       122
    // Anik        138
    // POR         134
    // PanAm       135
    //
    // EGNOS:
    // AOR-E       120
    // Artemis     124
    // IOR-W       126
    // IOR-E       131
    //
    // MSAS:
    // MTSAT-1     129
    // MTSAT-2     137
    //
    // The index mapping is as follows:
    // PRN 1-37    maps to indidex 0-36
    // PRN 38-40   maps to indices 37-39 (reserved mappings)
    // PRN 120-138 maps to indicex 40-58
    

    // check unsupported prn values
    if( prn == 0 )
    {
      return false;
    }
    if( prn > 38 && prn < 120 )
    {
      return false;
    }
    if( prn > 138 )
    {
      return false;
    }
    
    if( prn < 38 )
    {
      index = prn - 1;
    }
    if( prn > 119 && prn < 139 )
    {
      index = prn - 80;
    }
    return true;
  }





  GNSS_RxData::GNSS_RxData()
  : m_nrValidObs(0), 
    m_prev_nrValidObs(0),
    m_elevationMask(5.0*DEG2RAD),
    m_cnoMask(28.0),
    m_locktimeMask(0.0),
    m_maxAgeEphemeris(14400), // 4 hours
    m_DisableTropoCorrection(false),
    m_DisableIonoCorrection(false),
    m_fid(NULL),
    m_messageLength(0),
    m_rxDataType(GNSS_RXDATA_UNKNOWN),
    m_CheckRinexObservationHeader(false),
    m_RINEX_use_eph(false),
    m_RINEX_eph_index(0)
  { 
    m_message[0] = '\0';
    ZeroAllMeasurements();
    ZeroPVT();

    memset( &m_klobuchar, 0, sizeof(GNSS_structKlobuchar) );
    memset( &m_RINEX_obs_header, 0, sizeof(RINEX_structDecodedHeader) );
    
    m_RINEX_eph.eph_array = NULL;
    m_RINEX_eph.array_length = 0;
    m_RINEX_eph.max_array_length = 0;
  }


  GNSS_RxData::~GNSS_RxData()
  {
    if( m_fid != NULL )
    {
      fclose( m_fid );
    }
    if( m_RINEX_eph.eph_array != NULL )
    {
      delete[] m_RINEX_eph.eph_array;
    }
  }


  bool GNSS_RxData::ZeroAllMeasurements()
  {
    unsigned i = 0;

    m_nrValidObs = 0;

    for( i = 0; i < GNSS_RXDATA_NR_CHANNELS; i++ )
    {
      memset( &(m_ObsArray[i]), 0, sizeof(GNSS_structMeasurement) );
      memset( &(m_prev_ObsArray[i]), 0, sizeof(GNSS_structMeasurement) );
    }
    return true;
  }


  bool GNSS_RxData::ZeroPVT()
  {
    memset( &m_pvt, 0, sizeof(GNSS_structPVT) );
    memset( &m_prev_pvt, 0, sizeof(GNSS_structPVT) );
    return true;
  }


  bool GNSS_RxData::Initialize( 
    const char* path,                  //!< The path to the Observation data file. 
    bool &isValidPath,                 //!< A boolean to indicate if the path is valid.
    const GNSS_enumRxDataType rxType,  //!< The receiver data type.
    const char* RINEX_ephemeris_path   //!< The path to a RINEX ephemeris file, NULL if not available.
    )
  {
    bool isRinexValid = false;
    isValidPath = false;
    
    if( path == NULL )
    {
      return false;
    }
    if( rxType == GNSS_RXDATA_UNKNOWN )
    {
      return false;
    }

    m_rxDataType = rxType;
    
    if( rxType == GNSS_RXDATA_RINEX21 || rxType == GNSS_RXDATA_RINEX211 )
    {
      if( !CheckRINEXObservationHeader( path, isRinexValid ) )
        return false;
      if( !isRinexValid )
        return false;
    }

    if( RINEX_ephemeris_path != NULL )
    {
      m_RINEX_eph.filepath = RINEX_ephemeris_path;
      
      if( !LoadRINEXNavigationData() )
        return false;

      // Indicate that the RINEX ephemeris data can be used.
      m_RINEX_use_eph = true;
    }

#ifndef _CRT_SECURE_NO_DEPRECATE
    if( fopen_s( &m_fid, path, "rb" ) != 0 )
      return false;
#else
    m_fid = fopen( path, "rb" );
#endif
    if( m_fid == NULL )    
      return false;


    if( rxType == GNSS_RXDATA_RINEX21 || rxType == GNSS_RXDATA_RINEX211 )
    {
      // advance over the header lines
      while( !feof(m_fid) && ferror(m_fid)==0 )
      {
        if( fgets( (char*)m_message, GNSS_RXDATA_MSG_LENGTH, m_fid ) == NULL )
          break;
        if( strstr( (char*)m_message, "END OF HEADER" ) != NULL )
          break;
      }
    }

    if( feof(m_fid) || ferror(m_fid) != 0 )
    {
      if( m_fid != NULL )
        fclose(m_fid);
      return false;
    }
    
    isValidPath = true;

    return true;
  }

  bool GNSS_RxData::LoadNext( bool &endOfStream )
  {
    bool result = false;
    switch( m_rxDataType )
    {
    case GNSS_RXDATA_NOVATELOEM4:
      {
        result = LoadNext_NOVATELOEM4( endOfStream );
        break;
      }
    case GNSS_RXDATA_RINEX21:
      {
        result = LoadNext_RINEX21( endOfStream );
        break;
      }
    case GNSS_RXDATA_RINEX211:
      {
        result = LoadNext_RINEX211( endOfStream );
        break;
      }
    default:
      {
        return false;
        break;
      }
    }

    if( result )
    {
      if( m_RINEX_use_eph )
      {
        if( !UpdateTheEphemerisArrayWithUsingRINEX() )
          return false;
      }
    }
    return result;
  }

  bool GNSS_RxData::CheckRINEXObservationHeader( const char *filepath, bool &isValid )
  {
    BOOL result;
    char RINEX_buffer[16384];
    unsigned RINEX_buffer_size = 0;
    double version = 0.0;
    RINEX_enumFileType file_type = RINEX_FILE_TYPE_UNKNOWN;

    isValid = false;

    if( filepath == NULL )
      return false;    
    
    result = RINEX_GetHeader( 
      filepath,
      RINEX_buffer,
      16384,
      &RINEX_buffer_size,
      &version,
      &file_type
    );
    if( result == FALSE )
      return false;

    result = RINEX_DecodeHeader_ObservationFile(
      RINEX_buffer,
      RINEX_buffer_size,
      &m_RINEX_obs_header
      );
    if( result == FALSE )
      return false;

    if( file_type != RINEX_FILE_TYPE_OBS )
    {
      isValid = false;
      return true;
    }

    if( m_rxDataType == GNSS_RXDATA_RINEX21 )
    {
      if( fabs( version - 2.1 ) < 1e-02 )
        isValid = true;
    }
    else if( m_rxDataType == GNSS_RXDATA_RINEX211 )
    {
      if( fabs( version - 2.11 ) < 1e-03 )
        isValid = true;
    }
    else
    {
      isValid = false;
    }
    return true;
  }

  bool GNSS_RxData::LoadNext_RINEX21( bool &endOfStream )
  {
    /*
    BOOL result;
    unsigned i = 0;

    FILE* fid = NULL;
    BOOL wasEndOfFileReached;  // Has the end of the file been reached (output).
    BOOL wasObservationFound;  // Was a valid observation found (output).
    unsigned filePosition=0;   // The file position for the start of the 
    unsigned nrObs = 0;
    
    */
    return true;
  }

  bool GNSS_RxData::LoadNext_RINEX211( bool &endOfStream )
  {
    BOOL result=0;
    unsigned i = 0;
    unsigned j = 0;
    unsigned short rx_gps_week = 0;
    double rx_gps_tow = 0.0;     

    BOOL wasEndOfFileReached=0;  // Has the end of the file been reached (output).
    BOOL wasObservationFound=0;  // Was a valid observation found (output).
    unsigned filePosition=0;   // The file position for the start of the 
    unsigned nrObs = 0;
    bool isAvailable = false;
    
    endOfStream = false;

    if( m_fid == NULL )
      return false;

    // Copy the current observations into the previous storage.
    m_prev_nrValidObs = m_nrValidObs;
    for( i = 0; i < m_nrValidObs; i++ )
    {
      m_prev_ObsArray[i] = m_ObsArray[i];
      memset( &(m_ObsArray[i]), 0, sizeof(GNSS_structMeasurement) ); // Initialize to zero.
    }

    // Get the next observation set.
    result = RINEX_GetNextObservationSet(
      m_fid,
      &m_RINEX_obs_header,
      &wasEndOfFileReached,
      &wasObservationFound,
      &filePosition,
      m_ObsArray,
      GNSS_RXDATA_NR_CHANNELS,
      &nrObs,
      &rx_gps_week,
      &rx_gps_tow
      );
    if( result == FALSE )
      return false;

    if( wasEndOfFileReached )
    {
      endOfStream = true;
      return true;
    }

    m_nrGPSL1Obs = 0;
    m_nrValidObs = 0;
    if( nrObs > 0 )
    {
      // Set the receiver time of the observation set.
      m_pvt.time.gps_week = rx_gps_week;
      m_pvt.time.gps_tow  = rx_gps_tow;

      for(i = 0; i < nrObs; i++ )
      {
        m_ObsArray[i].stdev_psr     = 1.4f;  // [m]
        m_ObsArray[i].stdev_adr     = 0.05f; // these are in cycles!.
        m_ObsArray[i].stdev_doppler = 0.5f;  // Hz

        // Check if ephemeris information is available
        if( !m_EphAlmArray.IsEphemerisAvailable( m_ObsArray[i].id, isAvailable ) )
          return false;
        m_ObsArray[i].flags.isEphemerisValid      = isAvailable;

        if( m_DisableTropoCorrection )
          m_ObsArray[i].flags.useTropoCorrection          = 0;
        else
          m_ObsArray[i].flags.useTropoCorrection          = 1; // defaults to yes

        if( m_DisableIonoCorrection )
          m_ObsArray[i].flags.useBroadcastIonoCorrection  = 0;
        else
          m_ObsArray[i].flags.useBroadcastIonoCorrection  = 1; // default to yes

        if( m_ObsArray[i].system == GNSS_GPS &&
          m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          m_nrGPSL1Obs++;
        }
        m_nrValidObs++;
      }
    }


    // Search for matching observations in the previous set of data to pass on static information
    // like ambiguities.
    for( i = 0; i < m_nrValidObs; i++ )
    {
      for( j = 0; j < m_prev_nrValidObs; j++ )
      {
        if( m_ObsArray[i].codeType == m_prev_ObsArray[j].codeType &&
          m_ObsArray[i].freqType == m_prev_ObsArray[j].freqType &&
          m_ObsArray[i].system == m_prev_ObsArray[j].system &&
          m_ObsArray[i].id == m_prev_ObsArray[j].id ) // Note should also check that channels are the same if real time!
        {
          m_ObsArray[i].ambiguity = m_prev_ObsArray[j].ambiguity;
          break;
        }
      }
    }

    return true;
  }  

  bool GNSS_RxData::LoadNext_NOVATELOEM4( bool &endOfStream )
  {
    BOOL result = FALSE;
    BOOL wasEndOfFileReached = FALSE;
    BOOL wasMessageFound = FALSE;
    unsigned filePosition = 0;
    unsigned short messageID = 0;
    NOVATELOEM4_enumMessageType messageType;
    unsigned numberBadCRC = 0;

    //  A NovAtel OEM4 header information struct.
    NOVATELOEM4_structBinaryHeader header; 

    // An array of struct_NOVATELOEM4_RANGE.
    NOVATELOEM4_structObservation obsArray[GNSS_RXDATA_NR_CHANNELS];

    // A gps ephemeris struct.
    GPS_structEphemeris eph;
    unsigned prn;            // The PRN.
    unsigned reference_week; // The ephemeris reference week.
    unsigned reference_tow;  // The ephemeris reference time of week.
    unsigned tow;            // The tow associated with the start of Subframe1.

    unsigned nrValidObs;
    unsigned i = 0;
    unsigned j = 0;
    bool isAvailable;  // A boolean used in checking if ephemeris is available.


    endOfStream = false;

    if( m_fid == NULL )
      return false;

    // Copy the current observations into the previous storage.
    m_prev_nrValidObs = m_nrValidObs;
    for( i = 0; i < m_nrValidObs; i++ )
    {
      m_prev_ObsArray[i] = m_ObsArray[i];
      memset( &(m_ObsArray[i]), 0, sizeof(GNSS_structMeasurement) ); // Initialize to zero.
    }

    switch( m_rxDataType )
    {
    case GNSS_RXDATA_NOVATELOEM4:
      {  
        while( !wasEndOfFileReached && !wasMessageFound )
        {
          result = NOVATELOEM4_FindNextMessageInFile( 
            m_fid,
            m_message,
            GNSS_RXDATA_MSG_LENGTH,
            &wasEndOfFileReached,
            &wasMessageFound,
            &filePosition,
            &m_messageLength,
            &messageID,
            &numberBadCRC 
            );
          if( result == FALSE )
            return false;
          
          messageType = (NOVATELOEM4_enumMessageType)messageID;
          if( wasMessageFound && messageType != NOVATELOEM4_RANGEB )
          { 
            if( messageType == NOVATELOEM4_RAWEPHEMB )
            {
              memset( &eph, 0, sizeof(GPS_structEphemeris) );
             
              result = NOVATELOEM4_DecodeRAWEPHEMB(
                m_message,
                m_messageLength,
                &header,
                &prn,
                &reference_week,
                &reference_tow,
                &tow,
                &eph.iodc,
                &eph.iode,
                &eph.toe,
                &eph.toc,
                &eph.week,
                &eph.health,
                &eph.alert_flag,
                &eph.anti_spoof,
                &eph.code_on_L2,
                &eph.ura,
                &eph.L2_P_data_flag,
                &eph.fit_interval_flag,
                &eph.age_of_data_offset,
                &eph.tgd,
                &eph.af2,
                &eph.af1,
                &eph.af0,
                &eph.m0,
                &eph.delta_n,
                &eph.ecc,
                &eph.sqrta,
                &eph.omega0,
                &eph.i0,
                &eph.w,
                &eph.omegadot,
                &eph.idot,
                &eph.cuc,
                &eph.cus,
                &eph.crc,
                &eph.crs,
                &eph.cic,
                &eph.cis );
              if( result )
              {
                eph.prn = prn;
                
                result = m_EphAlmArray.AddEphemeris( eph.prn, eph );
                if( !result )
                {
                  // An error occurred.
                  return false;
                }
              }
            }
            wasMessageFound = false;
          }
        }
        if( wasMessageFound && messageType == NOVATELOEM4_RANGEB )
        {
          m_nrValidObs = 0;
          m_nrGPSL1Obs = 0;
          
          result = NOVATELOEM4_DecodeRANGEB( 
            m_message,
            m_messageLength,
            &header,
            obsArray,
            GNSS_RXDATA_NR_CHANNELS,
            &nrValidObs 
            );
          if( result == FALSE )
            return false;

          // Set the receiver time of the observation set.
          m_pvt.time.gps_week = header.gpsWeek;
          m_pvt.time.gps_tow  = header.gpsMilliSeconds / 1000.0;

          for( i = 0; i < nrValidObs && i < GNSS_RXDATA_NR_CHANNELS; i++ )
          {
            m_ObsArray[i].channel   = obsArray[i].trackingStatus.channelNumber;
            m_ObsArray[i].id        = obsArray[i].prn;

            m_ObsArray[i].system    = (GNSS_enumSystem)obsArray[i].trackingStatus.eSatelliteSystem;
            m_ObsArray[i].codeType  = (GNSS_enumCodeType)obsArray[i].trackingStatus.eCodeType;
            m_ObsArray[i].freqType  = (GNSS_enumFrequency)obsArray[i].trackingStatus.eFrequency;

            if( m_ObsArray[i].system == GNSS_GPS &&
              m_ObsArray[i].freqType == GNSS_GPSL1 )
            {
              m_nrGPSL1Obs++;
            }

            m_ObsArray[i].psr       = obsArray[i].psr;
            m_ObsArray[i].adr       = -1.0*obsArray[i].adr;
            m_ObsArray[i].doppler   = obsArray[i].doppler;
            m_ObsArray[i].cno       = obsArray[i].cno;
            m_ObsArray[i].locktime  = obsArray[i].locktime;
    
            if( obsArray[i].psrstd < 0.2 )
              m_ObsArray[i].stdev_psr     = 0.2f;
            else
              m_ObsArray[i].stdev_psr     = obsArray[i].psrstd; 

            m_ObsArray[i].stdev_psr     = 1.4f;
            
            if( obsArray[i].adrstd < 0.0025 )
              m_ObsArray[i].stdev_adr     = 0.0025f; // these are in cycles!.              
            else
              m_ObsArray[i].stdev_adr     = obsArray[i].adrstd; // these are in cycles!.

            if( obsArray[i].adrstd < 0.0001 )
              int gaa = 99;

            m_ObsArray[i].stdev_adr     = 0.05f; // these are in cycles!.
            
            m_ObsArray[i].stdev_doppler = 0.5f; // Hz

            m_ObsArray[i].psr_smoothed      = 0.0;
            m_ObsArray[i].psr_predicted     = 0.0;
            m_ObsArray[i].doppler_predicted = 0.0;
            m_ObsArray[i].azimuthRads       = 0.0;
            m_ObsArray[i].elevationRads     = 0.0;

            m_ObsArray[i].flags.isActive = 1;
            m_ObsArray[i].flags.isCodeLocked   = obsArray[i].trackingStatus.isCodeLocked;
            m_ObsArray[i].flags.isPhaseLocked  = obsArray[i].trackingStatus.isPhaseLocked;
            m_ObsArray[i].flags.isParityValid  = obsArray[i].trackingStatus.isParityKnown;
            m_ObsArray[i].flags.isPsrValid     = obsArray[i].trackingStatus.isCodeLocked;
            m_ObsArray[i].flags.isAdrValid     = obsArray[i].trackingStatus.isPhaseLocked & obsArray[i].trackingStatus.isParityKnown;
            m_ObsArray[i].flags.isDopplerValid = obsArray[i].trackingStatus.isCodeLocked;            
            m_ObsArray[i].flags.isGrouped      = obsArray[i].trackingStatus.isGrouped;
            m_ObsArray[i].flags.isAutoAssigned = !obsArray[i].trackingStatus.isForcedAssignment;
            m_ObsArray[i].flags.isCarrierSmoothed     = 0; // not yet known

            // Check if ephemeris information is available
            if( !m_EphAlmArray.IsEphemerisAvailable( m_ObsArray[i].id, isAvailable ) )
              return false;

            m_ObsArray[i].flags.isEphemerisValid      = isAvailable;

            m_ObsArray[i].flags.isAlmanacValid        = 0; // not yet known
            m_ObsArray[i].flags.isAboveElevationMask  = 0; // not yet known
            m_ObsArray[i].flags.isAboveCNoMask        = 0; // not yet known
            m_ObsArray[i].flags.isAboveLockTimeMask   = 0; // not yet known
            m_ObsArray[i].flags.isNotUserRejected     = 1; // assume not rejected
            m_ObsArray[i].flags.isNotPsrRejected      = 1; // assume not rejected
            m_ObsArray[i].flags.isNotAdrRejected      = 1; // assume not rejected
            m_ObsArray[i].flags.isNotDopplerRejected  = 1; // assume not rejected
            m_ObsArray[i].flags.isNoCycleSlipDetected = 1; // assume no slip
            m_ObsArray[i].flags.isUsedInPosSolution   = 0; // not yet known
            m_ObsArray[i].flags.isUsedInVelSolution   = 0; // not yet known

            if( m_DisableTropoCorrection )
              m_ObsArray[i].flags.useTropoCorrection          = 0;
            else
              m_ObsArray[i].flags.useTropoCorrection          = 1; // defaults to yes

            if( m_DisableIonoCorrection )
              m_ObsArray[i].flags.useBroadcastIonoCorrection  = 0;
            else
              m_ObsArray[i].flags.useBroadcastIonoCorrection  = 1; // default to yes
            
            m_ObsArray[i].corrections.prcTropoDry = 0;
            m_ObsArray[i].corrections.prcTropoWet = 0;
            m_ObsArray[i].corrections.prcIono = 0;
            m_ObsArray[i].corrections.prcSatClk = 0;
            m_ObsArray[i].corrections.prcReserved1 = 0;
            m_ObsArray[i].corrections.prcReserved2 = 0;
            m_ObsArray[i].corrections.rrcSatClkDrift = 0;
            m_ObsArray[i].corrections.rrcReserved1 = 0;
            m_ObsArray[i].corrections.rrcReserved2 = 0;
            m_ObsArray[i].corrections.dX = 0;
            m_ObsArray[i].corrections.dY = 0;
            m_ObsArray[i].corrections.dZ = 0;
            
            m_ObsArray[i].residuals.psrResidual = 0;
            m_ObsArray[i].residuals.adrResidual = 0;
            m_ObsArray[i].residuals.dopplerResidual = 0;
            m_ObsArray[i].residuals.reserved = 0;

            m_nrValidObs++;
          }
          
        }
        if( wasEndOfFileReached )
        {
          endOfStream = true;
        }
        break;
      }
    default:
      {
        return false;
      }
    }


    // Search for matching observations in the previous set of data to pass on static information
    // like ambiguities.
    for( i = 0; i < m_nrValidObs; i++ )
    {
      for( j = 0; j < m_prev_nrValidObs; j++ )
      {
        if( m_ObsArray[i].codeType == m_prev_ObsArray[j].codeType &&
          m_ObsArray[i].freqType == m_prev_ObsArray[j].freqType &&
          m_ObsArray[i].system == m_prev_ObsArray[j].system &&
          m_ObsArray[i].id == m_prev_ObsArray[j].id ) // Note should also check that channels are the same if real time!
        {
          m_ObsArray[i].ambiguity = m_prev_ObsArray[j].ambiguity;
          break;
        }
      }
    }


    return true;
  }

  bool GNSS_RxData::SetInitialPVT( 
    const double latitudeRads,   //!< The latitude [rad].
    const double longitudeRads,  //!< The longitude [rad].
    const double height,         //!< The orthometric height [m].
    const double vn,             //!< The northing velocity [m/s].
    const double ve,             //!< The easting velocity [m/s].
    const double vup,            //!< The up velocity [m/s].
    const double clk,            //!< The clock offset [m].
    const double clkdrift,       //!< The clock drift [m/s].
    const double std_lat,        //!< The standard deviation uncertainty in the latitude [m].
    const double std_lon,        //!< The standard deviation uncertainty in the longitude [m].
    const double std_hgt,        //!< The standard deviation uncertainty in the height [m].
    const double std_vn,         //!< The standard deviation uncertainty in the northing velocity [m/s].
    const double std_ve,         //!< The standard deviation uncertainty in the easting velocity [m/s].
    const double std_vup,        //!< The standard deviation uncertainty in the up velocity [m/s].
    const double std_clk,        //!< The standard deviation uncertainty in the clock offset [m].
    const double std_clkdrift,   //!< The standard deviation uncertainty in the clock drift [m/s].
    const double undulation      //!< The undulation if known [m].
    )
  {
    bool result;

    result = UpdatePositionAndRxClock(
      latitudeRads,
      longitudeRads,
      height,
      clk,
      std_lat,
      std_lon,
      std_hgt,
      std_clk );
    if( !result )
      return false;

    result = UpdateVelocityAndClockDrift(
      vn,
      ve,
      vup,
      clkdrift,
      std_vn,
      std_ve,
      std_vup,
      std_clkdrift );
    if( !result )
      return false;

    //m_pvt.dop;     //!< All the associated DOP information for this solution.

    if( std_lat == 0.0 && std_lon == 0.0 && std_hgt == 0.0 )
    {
      m_pvt.std_lat = 1e-03;
      m_pvt.std_lon = 1e-03;
      m_pvt.std_hgt = 1e-03;      
      m_pvt.isPositionConstrained = 1;
    }

    m_pvt.isHeightConstrained = 0;
    m_pvt.isClockConstrained = 0;
    m_pvt.isSolutionBasedOnEphemeris = 0;

    m_pvt.undulation = undulation;

    // Set the previous pvt as well.
    m_prev_pvt = m_pvt;
    
    return true;
  }


  bool GNSS_RxData::UpdatePositionAndRxClock( 
    const double latitudeRads,   //!< The latitude [rad].
    const double longitudeRads,  //!< The longitude [rad].
    const double height,         //!< The orthometric height [m].
    const double clk,            //!< The clock offset [m].
    const double std_lat,        //!< The standard deviation uncertainty in the latitude [m].
    const double std_lon,        //!< The standard deviation uncertainty in the longitude [m].
    const double std_hgt,        //!< The standard deviation uncertainty in the height [m].
    const double std_clk         //!< The standard deviation uncertainty in the clock offset [m].
    )
  {
    bool result;
    m_pvt.latitude   = latitudeRads;
    m_pvt.longitude  = longitudeRads;
    m_pvt.height     = height;
    
    m_pvt.latitudeDegs  = latitudeRads*RAD2DEG;
    m_pvt.longitudeDegs = longitudeRads*RAD2DEG;

    result = GetDMS( 
      m_pvt.latitudeDegs, 
      m_pvt.lat_dms.degrees, 
      m_pvt.lat_dms.minutes, 
      m_pvt.lat_dms.seconds, 
      (char*)m_pvt.lat_dms.dms_str, 24 );
    if( result == false )
      return false;

    result = GetDMS( 
      m_pvt.longitudeDegs, 
      m_pvt.lon_dms.degrees, 
      m_pvt.lon_dms.minutes, 
      m_pvt.lon_dms.seconds, 
      (char*)m_pvt.lon_dms.dms_str, 24 );
    if( result == false )
      return false;

    if( GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      m_pvt.latitude,
      m_pvt.longitude,
      m_pvt.height,
      &m_pvt.x,
      &m_pvt.y,
      &m_pvt.z ) == FALSE )
    {
      return false;
    }
    m_pvt.clockOffset = clk;

    m_pvt.std_lat = std_lat;
    m_pvt.std_lon = std_lon;
    m_pvt.std_hgt = std_hgt;
    m_pvt.std_clk = std_clk;
    
    return true;
  }

  bool GNSS_RxData::UpdateVelocityAndClockDrift( 
    const double vn,             //!< The northing velocity [m/s].
    const double ve,             //!< The easting velocity [m/s].
    const double vup,            //!< The up velocity [m/s].
    const double clkdrift,       //!< The clock drift [m/s].
    const double std_vn,         //!< The standard deviation uncertainty in the northing velocity [m/s].
    const double std_ve,         //!< The standard deviation uncertainty in the easting velocity [m/s].
    const double std_vup,        //!< The standard deviation uncertainty in the up velocity [m/s].
    const double std_clkdrift    //!< The standard deviation uncertainty in the clock drift [m/s].
    )
  {
    m_pvt.vn  = vn;
    m_pvt.ve  = ve;
    m_pvt.vup = vup;

    GEODESY_RotateVectorFromLocalGeodeticFrameToEarthFixedFrame(
      m_pvt.latitude,
      m_pvt.longitude,
      m_pvt.vn,
      m_pvt.ve,
      m_pvt.vup,
      &m_pvt.vx,
      &m_pvt.vy,
      &m_pvt.vz );
    
    m_pvt.clockDrift = clkdrift;

    m_pvt.std_vn  = std_vn;
    m_pvt.std_ve  = std_ve;
    m_pvt.std_vup = std_vup;
    m_pvt.std_clkdrift  = std_clkdrift;

    return true;
  }



  //static 
  bool GNSS_RxData::GetDMS( 
    const double angleDegs,  //!< The angle [degrees].
    short &degrees,          //!< The degrees part.
    short &minutes,          //!< The minutes part.
    float &seconds,          //!< The seconds part.
    char *dms_str,           //!< A DMS string e.g. dms_str = "-180'59'59.9999\""
    const unsigned maxLength_dms_str //!< The maximum length of the dms_str string.
    )
  {
    bool isNegative = false;
    double angle = angleDegs;
    int result = 0;

    if( angleDegs < 0 )
    {
      isNegative = true;
      angle = -angleDegs;
    }
    else
    {
      angle = angleDegs;
    }

    degrees = static_cast<short>(floor(angle));
    minutes = static_cast<short>(floor((angle - degrees)*60.0));
    seconds = static_cast<float>((angle - degrees - minutes/60.0)*3600.0);

    if( isNegative )
      degrees = -degrees;

#ifndef _CRT_SECURE_NO_DEPRECATE
    result = sprintf_s( 
      dms_str, 
      maxLength_dms_str, 
      "%+4d%c%02d\'%02.4f\"", 
      degrees, 
      176, // the degrees symbol
      minutes,
      seconds );
#else
    result = sprintf( 
      dms_str,     
      "%+4d%c%02d\'%02.4f\"", 
      degrees, 
      176, // the degrees symbol
      minutes,
      seconds );
#endif

    if( result == -1 )
      return false;

    return true;
  }


  bool GNSS_RxData::CheckForCycleSlips_UsingPhaseRatePrediction( 
    const double nrThresholdCycles //!< The maximum number of cycles to use as the threshold to detect a slip [cycles].
    )
  {
    unsigned i = 0;
    unsigned j = 0;
    double t_prev = 0.0;
    double t = 0.0;
    double dt = 0.0;
    double phase_diff = 0.0; // The difference between the predicted phase and the measured phase.
    double predicted_phase = 0.0;
    double mean_doppler = 0.0;

    static double max_dif = 0.0;

    t_prev = m_prev_pvt.time.gps_week*SECONDS_IN_WEEK + m_prev_pvt.time.gps_tow;
    t = m_pvt.time.gps_week*SECONDS_IN_WEEK + m_pvt.time.gps_tow;
    dt = t - t_prev;
    if( dt <= 0.0 )
    {
      return false;
    }
    if( dt > 60.0 ) // The phase rate prediction method is not going to work well.
    {
      return true;
    }
            
    for( i = 0; i < m_nrValidObs; i++ )
    {
      for( j = 0; j < m_prev_nrValidObs; j++ )
      {
        if( m_ObsArray[i].system == m_prev_ObsArray[j].system &&
          m_ObsArray[i].id == m_prev_ObsArray[j].id &&
          m_ObsArray[i].freqType == m_prev_ObsArray[j].freqType )
        {
          // channel matching is not perforned.
          // This should be done if this a real time receiver data.
          // However, since the data source is generally not known for
          // post processing software, it is not performed here.
          if( m_ObsArray[i].flags.isPhaseLocked && m_prev_ObsArray[j].flags.isPhaseLocked )
          {
            mean_doppler = (m_ObsArray[i].doppler + m_prev_ObsArray[j].doppler)/2.0;

            predicted_phase = m_prev_ObsArray[j].adr - mean_doppler * dt; // GDM_BEWARE Doppler sign convention

            phase_diff = predicted_phase - m_ObsArray[i].adr;

            if( fabs(phase_diff) > max_dif )
              max_dif = fabs(phase_diff);
            if( fabs(phase_diff) > nrThresholdCycles )
            {
              m_ObsArray[i].flags.isNoCycleSlipDetected = 0; // Indicate a cycle slip has occured.
            }
            else
            {
              m_ObsArray[i].flags.isNoCycleSlipDetected = 1; // No cycle slip detected.
            }
            break;
          }
        }
      }
    }

    return true;
  }


  bool GNSS_RxData::DebugPrintObservationArray( const char *filepath )
  {
    unsigned i = 0;
    unsigned j = 0;
    Matrix X(100,m_nrValidObs);
    
    if( filepath == NULL )
      return false;


    for( i = 0; i < m_nrValidObs; i++ )
    {
      j = 0;
      X[j][i] = m_ObsArray[i].channel; j++;
      X[j][i] = m_ObsArray[i].id; j++;       //!< The unique id for this channel (eg PRN for GPS).    
      X[j][i] = m_ObsArray[i].system; j++;   //!< The satellite system associated with this channel.
      X[j][i] = m_ObsArray[i].codeType; j++; //!< The code type for this channel.
      X[j][i] = m_ObsArray[i].freqType; j++; //!< The frequency type for this channel.

      X[j][i] = m_ObsArray[i].flags.isActive; j++;              //!< This flag indicates that the channel is active for use. If this is not set, no other flags are valid for use.
      X[j][i] = m_ObsArray[i].flags.isCodeLocked; j++;          //!< Indicates if the code tracking is locked.
      X[j][i] = m_ObsArray[i].flags.isPhaseLocked; j++;         //!< Indicates if the phase tracking is locked.
      X[j][i] = m_ObsArray[i].flags.isParityValid; j++;         //!< Indicates if the phase parity if valid.      
      X[j][i] = m_ObsArray[i].flags.isPsrValid; j++;            //!< Indicates if the pseudorange valid for use.
      X[j][i] = m_ObsArray[i].flags.isAdrValid; j++;            //!< Indicates if the ADR is valid for use.
      X[j][i] = m_ObsArray[i].flags.isDopplerValid; j++;        //!< Indicates if the Doppler if valid for use.
      X[j][i] = m_ObsArray[i].flags.isGrouped; j++;             //!< Indicates if this channel has another associated channel. eg. L1 and L2 measurements.
      X[j][i] = m_ObsArray[i].flags.isAutoAssigned; j++;        //!< Indicates if the channel was receiver assigned (otherwise, the user forced this channel assignment).
      X[j][i] = m_ObsArray[i].flags.isCarrierSmoothed; j++;     //!< Indicates if the pseudorange has carrier smoothing enabled.
      X[j][i] = m_ObsArray[i].flags.isEphemerisValid; j++;      //!< Indicates if this channel has valid associated ephemeris information. 
      X[j][i] = m_ObsArray[i].flags.isAlmanacValid; j++;        //!< Indicates if this channel has valid associated almanac information.
      X[j][i] = m_ObsArray[i].flags.isAboveElevationMask; j++;  //!< Indicates if the satellite tracked is above the elevation mask.    
      X[j][i] = m_ObsArray[i].flags.isAboveCNoMask; j++;        //!< Indciates if the channel's C/No is above a threshold value.
      X[j][i] = m_ObsArray[i].flags.isAboveLockTimeMask; j++;   //!< Indicates if the channel's locktime is above a treshold value.
      X[j][i] = m_ObsArray[i].flags.isNotUserRejected; j++;     //!< Indicates if the user has not forced the rejection of this channel or PRN.
      X[j][i] = m_ObsArray[i].flags.isNotPsrRejected; j++;      //!< Indicates if the pseudorange was not rejetced (ie Fault Detection and Exclusion).
      X[j][i] = m_ObsArray[i].flags.isNotAdrRejected; j++;      //!< Indicates if the ADR was not rejetced (ie Fault Detection and Exclusion).
      X[j][i] = m_ObsArray[i].flags.isNotDopplerRejected; j++;  //!< Indicates if the Doppler was not rejected (ie Fault Detection and Exclusion).
      X[j][i] = m_ObsArray[i].flags.isNoCycleSlipDetected; j++; //!< Indicates that no cycle slip has occurred at this epoch.
      X[j][i] = m_ObsArray[i].flags.isUsedInPosSolution; j++;   //!< Indicates if some part (pseudorange) of this channel's measurement was used in the position solution.
      X[j][i] = m_ObsArray[i].flags.isUsedInVelSolution; j++;   //!< Indicates if some part (Doppler) of this channel's measurement was used in the velocity solution.
      X[j][i] = m_ObsArray[i].flags.isAdrUsedInSolution; j++;   //!< Indicates if the the ADR is used in the solution.
      X[j][i] = m_ObsArray[i].flags.isDifferentialPsrAvailable; j++;     //!< Indicates if a matching pseudrange observation is available from another receiver.
      X[j][i] = m_ObsArray[i].flags.isDifferentialDopplerAvailable; j++; //!< Indicates if a matching Doppler observation is available from another receiver.
      X[j][i] = m_ObsArray[i].flags.isDifferentialAdrAvailable; j++;     //!< Indicates if a matching ADR observation is available from another receiver.
      X[j][i] = m_ObsArray[i].flags.useTropoCorrection; j++;         //!< Indicates that the tropospheric correction should be applied.
      X[j][i] = m_ObsArray[i].flags.useBroadcastIonoCorrection; j++; //!< Indicates that the broadcast ionospheric correction should be applied.
      X[j][i] = m_ObsArray[i].flags.isTimeDifferntialPsrAvailable; j++;
      X[j][i] = m_ObsArray[i].flags.isTimeDifferntialDopplerAvailable; j++;

      X[j][i] = m_ObsArray[i].week; j++;  //!< The measurement gps week (at 'transmit' time) [weeks].
      X[j][i] = m_ObsArray[i].tow; j++;   //!< The measurement gps time of week (at 'transmit' time) [s].

      // The actual measurements.
      X[j][i] = m_ObsArray[i].psr; j++;               //!< The pseudorange measurement [m].
      X[j][i] = m_ObsArray[i].adr; j++;               //!< The carrier phase or accumulated Doppler range measurement [cycles].
      X[j][i] = m_ObsArray[i].doppler; j++;           //!< The Doppler measurement for this channel [Hz].
      X[j][i] = m_ObsArray[i].cno; j++;               //!< The carrier to noise density ratio for this channel [dB-Hz]
      X[j][i] = m_ObsArray[i].locktime; j++;          //!< The number of seconds of continous phase tracking (no known cycle slips) [s].

      // The variance information associated with the actual measurements.
      X[j][i] = m_ObsArray[i].stdev_psr; j++;         //!< The estimated pseudorange measurement standard deviation [m].
      X[j][i] = m_ObsArray[i].stdev_adr; j++;         //!< The estimated accumulated Doppler range measurement standard deviation [cycles].
      X[j][i] = m_ObsArray[i].stdev_doppler; j++;     //!< The estimated Doppler measurement standard deviation [Hz].

      // Derived information.
      X[j][i] = m_ObsArray[i].psr_misclosure; j++;    //!< The measured psr minus the computed psr estimate [m].
      X[j][i] = m_ObsArray[i].doppler_misclosure; j++;//!< The measured Doppler minus the computed Doppler estimate [m].
      X[j][i] = m_ObsArray[i].range; j++;             //!< The best estimate of the geometric range between the antenna and the satellite [m].
      X[j][i] = m_ObsArray[i].rangerate; j++;         //!< The best estimate of the geometric range rate between the antenna and the satellite [m/s].
      X[j][i] = m_ObsArray[i].psr_smoothed; j++;      //!< The carrier smoothed pseudorange if available [m].
      X[j][i] = m_ObsArray[i].psr_predicted; j++;     //!< The predicted pseudorange based on the satellite position, user position, and current clock offset [m].
      X[j][i] = m_ObsArray[i].ambiguity; j++;         //!< The estimated integer component of the adr. This may be the single or double differenced ambiguity [].
      X[j][i] = m_ObsArray[i].doppler_predicted; j++; //!< The predicted Doppler based on user position, velocity, satellite position, velocity and clock rate [Hz].
      X[j][i] = m_ObsArray[i].azimuthRads; j++;       //!< The associated satellite azimuth for this channel [rad].
      X[j][i] = m_ObsArray[i].elevationRads; j++;     //!< The associated satellite elevation for this channel  [rad].


      X[j][i] = m_ObsArray[i].index_differential; j++;      //!< The channel index of a matching differential observation. -1 means there is no matching channel.
      X[j][i] = m_ObsArray[i].index_time_differential; j++; //!< The channel index of a matching time differential observation. -1 means there is no matching channel.

      X[j][i] = m_ObsArray[i].index_differential_adr; j++;  //!< The channel index of a matching differential observation. -1 means there is no matching channel.
      X[j][i] = m_ObsArray[i].index_ambiguity_state; j++;   //!< The index into the state vector for this ambiguity state. -1 not estimated.
      X[j][i] = m_ObsArray[i].adr_misclosure; j++;          //!< The measured ADR minus the computed ADR estimate [m]. This is likely a differential quantity.

      X[j][i] = m_ObsArray[i].H_p[0]; j++; //!< The design matrix row relating the pseudorange measurements to the position solution. dP/d(lat), dP/d(lon), dP/d(hgt).
      X[j][i] = m_ObsArray[i].H_p[1]; j++;
      X[j][i] = m_ObsArray[i].H_p[2]; j++;
      X[j][i] = m_ObsArray[i].H_v[0]; j++; //!< The design matrix row relating the Doppler measurements to the velocity solution. dD/d(lat), dD/d(lon), dD/d(hgt).
      X[j][i] = m_ObsArray[i].H_v[1]; j++;
      X[j][i] = m_ObsArray[i].H_v[2]; j++;

      X[j][i] = m_ObsArray[i].corrections.dX; j++; //!< The corrections associated with this channel.
      X[j][i] = m_ObsArray[i].corrections.dY; j++;
      X[j][i] = m_ObsArray[i].corrections.dZ; j++;
      X[j][i] = m_ObsArray[i].corrections.prcIono; j++;
      X[j][i] = m_ObsArray[i].corrections.prcReserved1; j++;
      X[j][i] = m_ObsArray[i].corrections.prcReserved2; j++;
      X[j][i] = m_ObsArray[i].corrections.prcSatClk; j++;
      X[j][i] = m_ObsArray[i].corrections.prcTropoDry; j++;
      X[j][i] = m_ObsArray[i].corrections.prcTropoWet; j++;
      X[j][i] = m_ObsArray[i].corrections.rrcReserved1; j++;

      X[j][i] = m_ObsArray[i].residuals.psrResidual; j++;   //!< The post-adjustment (filtering) measurement residual associated with this channel.
      X[j][i] = m_ObsArray[i].residuals.dopplerResidual; j++;
      X[j][i] = m_ObsArray[i].residuals.adrResidual; j++;
      X[j][i] = m_ObsArray[i].residuals.reserved; j++;      
    }
    X.Redim(j,m_nrValidObs);

    X.Print( filepath, 12 ); // print to the file in append mode.

    return true;
  }



  bool GNSS_RxData::Debug_WriteSuperMsg80CharsWide(
    char* buffer,                    //!< A large character buffer (8KB min).
    const unsigned maxBufferLength,  //!< The maximum buffer length [bytes].
    const double referenceLatitude,  //!< Reference position latitude [rad].
    const double referenceLongitude, //!< Reference position longitude [rad].
    const double referenceHeight,    //!< Reference position height [m].
    unsigned& nrBytesInBuffer )      //!< The number of bytes set in the buffer.
  {
    unsigned i = 0;
    unsigned j = 0;
    int week = -1;
    int tmpi = 0;

    unsigned scount = 0;

    double x = 0;
    double y = 0;
    double z = 0;

    double n_err  = 0.0;
    double e_err  = 0.0;
    double up_err = 0.0;
    double dtmp = 0.0;
    double v_azimuth = 0.0;

    if( maxBufferLength < 1024*8 )
    {
      nrBytesInBuffer = 0;
      return false;
    }
    
    scount += sprintf( buffer, "\n");
    scount += sprintf( buffer+scount, "GPS_Week   GPS_TOW(s)   DayOfYear   UTC_Time\n" );
    scount += sprintf( buffer+scount, "    %4d", m_pvt.time.gps_week );
    scount += sprintf( buffer+scount, "%13.3lf", m_pvt.time.gps_tow );
    scount += sprintf( buffer+scount, "         %03d", m_pvt.time.day_of_year );
    scount += sprintf( buffer+scount, "   %04d, %02d, %02d, %02d:%02d:%05.2lf\n", 
      m_pvt.time.utc_year, 
      m_pvt.time.utc_month,
      m_pvt.time.utc_day,
      m_pvt.time.utc_hour,
      m_pvt.time.utc_minute,
      m_pvt.time.utc_seconds );
    scount += sprintf( buffer+scount, "\n" );

    scount += sprintf( buffer+scount, "            Lat(deg)  Long(deg)  Height(m)         X(m)        Y(m)        Z(m)\n");
    scount += sprintf( buffer+scount, "Estimated:%10.5f %10.5f %10.3f ", m_pvt.latitudeDegs, m_pvt.longitudeDegs, m_pvt.height );    
    scount += sprintf( buffer+scount, "%12.1lf %11.1lf %11.1lf\n", m_pvt.x, m_pvt.y, m_pvt.z );

    if( referenceLatitude == 0 && referenceLongitude == 0 && referenceHeight == 0 )
    {
      x = y = z = 0;
    }
    else
    {
      GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
        GEODESY_REFERENCE_ELLIPSE_WGS84,
        referenceLatitude,
        referenceLongitude,
        referenceHeight,
        &x, 
        &y,
        &z );
    }

    scount += sprintf( buffer+scount, "Reference:%10.5f %10.5f %10.3f ", referenceLatitude*RAD2DEG, referenceLongitude*RAD2DEG, referenceHeight );      
    scount += sprintf( buffer+scount, "%12.1lf %11.1lf %11.1lf\n", x, y, z );

    if( x == 0 && y == 0 && z == 0 )
    {
      n_err = e_err = up_err = 0.0;
    }
    else
    {
      GEODESY_ComputePositionDifference(
        GEODESY_REFERENCE_ELLIPSE_WGS84,
        referenceLatitude,  
        referenceLongitude, 
        referenceHeight,    
        m_pvt.latitude,
        m_pvt.longitude,
        m_pvt.height,  
        &n_err,
        &e_err,
        &up_err
        );
    }
 
    scount += sprintf( buffer+scount, "Diff(m)  :");
    if (fabs(n_err) < 1000)  scount += sprintf( buffer+scount, " %9.5f", n_err);
    else                     scount += sprintf( buffer+scount, "          ");
    if (fabs(e_err) < 1000)  scount += sprintf( buffer+scount, "  %9.5f", e_err);
    else                     scount += sprintf( buffer+scount, "           ");
    if (fabs(up_err) < 1000) scount += sprintf( buffer+scount, "  %9.3f\n\n", up_err );
    else                     scount += sprintf( buffer+scount, "           \n\n");

    scount += sprintf( buffer+scount, "East(m/s)      North(m/s)       Up(m/s)      Ground_Speed(km/h)    Azimuth(deg)\n" );
    dtmp = sqrt( m_pvt.ve*m_pvt.ve + m_pvt.vn*m_pvt.vn )*3.6;
    v_azimuth = atan2(m_pvt.ve, m_pvt.vn);
    scount += sprintf( buffer+scount, "%9.2lf %15.2lf %13.2lf %23.2lf %15.3lf\n", 
      m_pvt.vn,
      m_pvt.ve,
      m_pvt.vup,
      dtmp,
      v_azimuth*RAD2DEG );      
    scount += sprintf( buffer+scount, "\n" );  


    scount += sprintf( buffer+scount, "Nr: PSR  U / R  Doppler  U / R  ADR  U / R    HDOP   VDOP   PDOP   TDOP   GDOP\n" );
    scount += sprintf( buffer+scount, "    %3d%3d%4d", m_pvt.nrPsrObsAvailable, m_pvt.nrPsrObsUsed, m_pvt.nrPsrObsRejected ); 
    scount += sprintf( buffer+scount, "%9d%3d%4d", m_pvt.nrDopplerObsAvailable, m_pvt.nrDopplerObsUsed, m_pvt.nrDopplerObsRejected ); 
    scount += sprintf( buffer+scount, "%5d%3d%4d", m_pvt.nrAdrObsAvailable, m_pvt.nrAdrObsUsed, m_pvt.nrAdrObsRejected ); 
    scount += sprintf( buffer+scount, "%8.1lf %6.1lf %6.1lf %6.1lf %6.1lf\n", 
      m_pvt.dop.hdop, 
      m_pvt.dop.vdop, 
      m_pvt.dop.pdop, 
      m_pvt.dop.tdop, 
      m_pvt.dop.gdop );
    scount += sprintf( buffer+scount, "\n" );  

    scount += sprintf( buffer+scount, "%2s", "CH" );
    scount += sprintf( buffer+scount, "%3s", "ID" );
    scount += sprintf( buffer+scount, "%3s", "S" );
    scount += sprintf( buffer+scount, "%13s", "PSR" );
    scount += sprintf( buffer+scount, "%2s", "U" );
    scount += sprintf( buffer+scount, "%9s", "DOPPLER" );
    scount += sprintf( buffer+scount, "%2s", "U" );
    scount += sprintf( buffer+scount, "%13s", "ADR" );
    scount += sprintf( buffer+scount, "%2s", "U" );
    scount += sprintf( buffer+scount, "%6s", "E" );
    scount += sprintf( buffer+scount, "%7s", "A" );
    scount += sprintf( buffer+scount, "%5s", "C/No" );
    scount += sprintf( buffer+scount, "%8s", "LOCK" );
    scount += sprintf( buffer+scount, "%3s", "SI" );
    scount += sprintf( buffer+scount, "\n" );

    for( i = 0; i < m_nrValidObs; i++ )
    {
      if( m_ObsArray[i].flags.isActive && m_ObsArray[i].freqType == GNSS_GPSL1)
      {
        scount += sprintf( buffer+scount, "%-2d", m_ObsArray[i].channel );
        scount += sprintf( buffer+scount, "%3d", m_ObsArray[i].id );
        if( m_ObsArray[i].system == GNSS_GPS && m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          scount += sprintf( buffer+scount, "%3s", "L1" );
        }
        else if( m_ObsArray[i].system == GNSS_GPS && m_ObsArray[i].freqType == GNSS_GPSL2 )
        {
          scount += sprintf( buffer+scount, "%3s", "L2" );
        }
        else
        {
          scount += sprintf( buffer+scount, "%3s", "--" );
        }
        scount += sprintf( buffer+scount, "%13.1lf", m_ObsArray[i].psr );
        scount += sprintf( buffer+scount, "%2d", m_ObsArray[i].flags.isUsedInPosSolution );

        scount += sprintf( buffer+scount, "%9.1lf", m_ObsArray[i].doppler );
        scount += sprintf( buffer+scount, "%2d", m_ObsArray[i].flags.isUsedInVelSolution );
        
        scount += sprintf( buffer+scount, "%13.1lf", m_ObsArray[i].adr );    
        scount += sprintf( buffer+scount, "%2d", m_ObsArray[i].flags.isAdrUsedInSolution );

        scount += sprintf( buffer+scount, "%6.1lf", m_ObsArray[i].satellite.elevation*RAD2DEG );
        scount += sprintf( buffer+scount, "%7.1lf", m_ObsArray[i].satellite.azimuth*RAD2DEG );
        scount += sprintf( buffer+scount, "%5.1lf", m_ObsArray[i].cno );
        scount += sprintf( buffer+scount, "%8.1lf", m_ObsArray[i].locktime );
        scount += sprintf( buffer+scount, "%3d", m_ObsArray[i].index_ambiguity_state );
        scount += sprintf( buffer+scount, "\n" );
      }
    }  
    scount += sprintf( buffer+scount, "\n" );

    nrBytesInBuffer = scount;
    return true;
  }


  bool GNSS_RxData::LoadRINEXNavigationData(void)
  {
    // First estimate the size of the ephemeris array needed based on the number of lines
    // in the data file after the header.
    unsigned line_count = 0;
    unsigned estimated_nr_eph = 0;
    BOOL result = 0;

    FILE* fid = NULL;
    fid = fopen( m_RINEX_eph.filepath.c_str(), "r" );
    if( fid == NULL )
      return false;

    // Advance over the header.
    while( !feof(fid) && ferror(fid)==0 )
    {
      if( fgets( (char*)m_message, GNSS_RXDATA_MSG_LENGTH, fid ) == NULL )
        break;
      if( strstr( (char*)m_message, "END OF HEADER" ) != NULL )
        break;
    }

    // Count the number of lines in the remaining data.
    while( !feof(fid) && ferror(fid)==0 )
    {
      if( fgets( (char*)m_message, GNSS_RXDATA_MSG_LENGTH, fid ) == NULL )
        break;
      line_count++;
    }

    fclose(fid);

    // There are 8 lines per ephemeris record generally.
    estimated_nr_eph = line_count/8;

    estimated_nr_eph += 32; // for good measure.

    // Allocate enough memory for the ephemeris array.
    m_RINEX_eph.eph_array = NULL;
    m_RINEX_eph.eph_array = new GPS_structEphemeris[estimated_nr_eph];
    if( m_RINEX_eph.eph_array == NULL )
      return FALSE;
    
    m_RINEX_eph.max_array_length = estimated_nr_eph;
    
    result = RINEX_DecodeGPSNavigationFile(
      m_RINEX_eph.filepath.c_str(),
      &m_RINEX_eph.iono_model,
      m_RINEX_eph.eph_array,
      m_RINEX_eph.max_array_length,
      &m_RINEX_eph.array_length
      );
    if( result == FALSE )
      return false;

    return true;
  }

  bool GNSS_RxData::UpdateTheEphemerisArrayWithUsingRINEX()
  {
    int RINEX_eph_index = -1;
    unsigned i = 0;
    unsigned j = 0;
    unsigned k = 0;
    unsigned short eph_week = 0;
    unsigned eph_tow = 0;
    bool result = false;
    double eph_time = 0;
    double current_eph_time = 0;
    const double rx_time = m_pvt.time.gps_week*SECONDS_IN_WEEK + m_pvt.time.gps_tow;
    bool isEphUpToDate;
    bool isAvailable;

    // GDM CONTINUE HERE

    /*
    - Only update thte active satellites
    - Update the ephemeris object as if the data is real time.
    - i.e. Not the best ephemeris but the most up to date.
    */
    for( j = 0; j < m_nrValidObs; j++ )
    {
      RINEX_eph_index = -1; // Not available.
      eph_time = 0;
      current_eph_time = 0;
      isEphUpToDate = false;
      isAvailable = false;
      if( m_ObsArray[j].flags.isActive )
      {
        if( m_ObsArray[j].system == GNSS_GPS )
        {
          // Check if new ephemeris information is available. 
          // Process as if the data is real-time.
    
          result = m_EphAlmArray.GetEphemerisTOW(
            m_ObsArray[j].id, 
            isAvailable,
            eph_week,
            eph_tow
            );
          if( !result )
            return false;

          if( isAvailable )
          {
            current_eph_time = eph_week*SECONDS_IN_WEEK + eph_tow;
          }

          // Go through the entire ephemeris array. This algorihtms assumes
          // the ephemeris records are increasing in time.
          for( k = 0; k < m_RINEX_eph.array_length; k++ )
          {
            if( m_RINEX_eph.eph_array[k].prn == m_ObsArray[j].id )
            {
              eph_time = m_RINEX_eph.eph_array[k].tow_week*SECONDS_IN_WEEK + m_RINEX_eph.eph_array[k].tow;

              if( !isAvailable )
              {
                if( rx_time > eph_time )
                {
                  RINEX_eph_index = k;
                }
                else
                {
                  if( RINEX_eph_index >= 0 )
                  {
                    result = m_EphAlmArray.AddEphemeris( 
                      m_RINEX_eph.eph_array[RINEX_eph_index].prn, 
                      m_RINEX_eph.eph_array[RINEX_eph_index]
                    );
                    if( !result )
                      return false;
                    isEphUpToDate = true;
                    break;
                  }
                  else
                  {
                    // There are no ephemeris records for this rx time yet.                    
                    break;
                  }                  
                }
              }
              else
              {
                if( rx_time > eph_time )
                {
                  if( current_eph_time < eph_time )
                  {
                    RINEX_eph_index = k;
                  }
                }
                else
                {
                  if( RINEX_eph_index >= 0 )
                  {
                    result = m_EphAlmArray.AddEphemeris( 
                      m_RINEX_eph.eph_array[RINEX_eph_index].prn, 
                      m_RINEX_eph.eph_array[RINEX_eph_index]
                    );
                    if( !result )
                      return false;
                    isEphUpToDate = true;                    
                  }
                  break;
                }
              }
            }
          }

          // Deal with end of file ephemeris records, no records are greater in time.
          if( !isEphUpToDate )
          {
            if( RINEX_eph_index >= 0 )
            {
              result = m_EphAlmArray.AddEphemeris( 
                m_RINEX_eph.eph_array[RINEX_eph_index].prn, 
                m_RINEX_eph.eph_array[RINEX_eph_index]
              );
              if( !result )
                return false;
              isEphUpToDate = true;
            }
          }
        }
      }
    }

    // Update the observation flags to indicate that ephemeris is available.
    for( j = 0; j < m_nrValidObs; j++ )
    {
      if( m_ObsArray[j].flags.isActive )
      {
        if( m_ObsArray[j].system == GNSS_GPS )
        {
          result = m_EphAlmArray.IsEphemerisAvailable( m_ObsArray[j].id, isAvailable );
          if( !result )
            return false;

          m_ObsArray[j].flags.isEphemerisValid = isAvailable;
        }
      }
    }
      
    

    return true;
  }


} // end namespace GNSS



