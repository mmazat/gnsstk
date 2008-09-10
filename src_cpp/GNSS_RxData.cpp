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
#include "gnss_error.h"
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
      {
        return true; // already allocated
      }
      else
      {
        GNSS_ERROR_MSG( "if( m_arrayLength != GPS_NUMBER_VALID_PRNS )" );
        return false; // error
      }
    }
    m_array = new GPS_structOrbitParameters[GPS_NUMBER_VALID_PRNS];
    if( m_array == NULL )
    {
      GNSS_ERROR_MSG( "new failed" );
      return false;
    }
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
      {
        GNSS_ERROR_MSG( "AllocateArray returned false." );
        return false;
      }
    }

    if( !GetIndexGivenPRN( prn, index ) )
    {
      GNSS_ERROR_MSG( "GetIndexGivenPRN returned false." );
      return false;
    }

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
      {
        GNSS_ERROR_MSG( "AllocateArray returned false." );
        return false;
      }
    }

    if( !GetIndexGivenPRN( prn, index ) )
    {
      GNSS_ERROR_MSG( "GetIndexGivenPRN returned false." );
      return false;
    }
    
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
    {
      GNSS_ERROR_MSG( "GetIndexGivenPRN returned false." );
      return false;
    }

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
    {
      GNSS_ERROR_MSG( "GetIndexGivenPRN returned false." );
      return false;
    }

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
    {
      GNSS_ERROR_MSG( "GetIndexGivenPRN returned false." );
      return false;
    }

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
      GNSS_ERROR_MSG( "if( prn == 0 )" );
      return false;
    }
    if( prn > 38 && prn < 120 )
    {
      GNSS_ERROR_MSG( "if( prn > 38 && prn < 120 )" );
      return false;
    }
    if( prn > 138 )
    {
      GNSS_ERROR_MSG( "if( prn > 138 )" );
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
    m_msJumpDetected_Positive(false),
    m_msJumpDetected_Negative(false),
    m_clockJumpDetected(false),
    m_clockJump(0.0),
    m_isStatic(false),
    m_heightConstraint(false),
    m_heightConstraintStdev(0.0),
    m_fid(NULL),
    m_messageLength(0),
    m_rxDataType(GNSS_RXDATA_UNKNOWN),
    m_CheckRinexObservationHeader(false),
    m_RINEX_use_eph(false),
    m_RINEX_eph_index(0),
    m_ambiguity_validation_ratio(0),
    m_probability_of_correct_ambiguities(0),
    m_norm(0.0),
    m_default_stdev_GPSL1_psr(0.8),
    m_default_stdev_GPSL1_doppler(0.09),
    m_default_stdev_GPSL1_adr(0.03)    
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
      m_RINEX_eph.eph_array = NULL;
    }
  }


  bool GNSS_RxData::SetDefaultMeasurementStdev_GPSL1(
    const double default_stdev_GPSL1_psr,     //!< default psr measurement standard deviation [m]
    const double default_stdev_GPSL1_doppler, //!< default doppler measurement standard deviation [m]
    const double default_stdev_GPSL1_adr      //!< default adr measurement standard deviation [m]
    )
  {
    if( default_stdev_GPSL1_psr <= 0 )
    {
      GNSS_ERROR_MSG( "if( default_stdev_GPSL1_psr <= 0 )" );
      return false;
    }      
    if( default_stdev_GPSL1_doppler <= 0 )
    {
      GNSS_ERROR_MSG( "if( default_stdev_GPSL1_doppler <= 0 )" );
      return false;
    }      
    if( default_stdev_GPSL1_adr <= 0 )
    {
      GNSS_ERROR_MSG( "if( default_stdev_GPSL1_adr <= 0 )" );
      return false;
    }      
    m_default_stdev_GPSL1_doppler = default_stdev_GPSL1_doppler;
    m_default_stdev_GPSL1_psr = default_stdev_GPSL1_psr;
    m_default_stdev_GPSL1_adr = default_stdev_GPSL1_adr ;
    return true;
  }
    

#define MSS

#ifdef GDM_UWB_RANGE_HACK
  bool GNSS_RxData::EnableAndLoadUWBData( 
    const char* filepath,  //!< The path to the UWB range data.
    const int a_id,
    const double xUWB_a,
    const double yUWB_a,
    const double zUWB_a,
    const int b_id,
    const double xUWB_b,
    const double yUWB_b,
    const double zUWB_b,
    const int c_id,
    const double xUWB_c,
    const double yUWB_c,
    const double zUWB_c,
    const bool isStatic //!< If the data is static, measurement outliers based on a somewaht ad-hoc 2 sigma rejection are removed.
    )
  {    
    unsigned i = 0;
    unsigned j = 0;
    char msg[64];

    strcpy( m_UWB.filepath, filepath );

#ifdef GDM_UWB_ALREADY_OUTLIER_REMOVED
    if( !m_UWB.data.ReadFromFile(filepath) )
    {
      GNSS_ERROR_MSG( "m_UWB.data.ReadFromFile() returned false." );
      return false;
    }
#else
    Matrix tmp;
    if( filepath == NULL )
    {
      GNSS_ERROR_MSG( "if( filepath == NULL )" );
      return false;
    }

    if( !tmp.ReadFromFile(filepath) )
    {
      GNSS_ERROR_MSG( "if( !tmp.ReadFromFile(filepath) )" );     
      return false;
    }

  //  tmp.Print( "tmpFilteredUWBRangeData.txt", 9 );

    ///////////DSC
 /*   // Remove unnecessary data
    for( i = 2; i < 5; i++ )
      tmp.RemoveColumn( 2 ); 

    for( i = 4; i < 10; i++ )
      tmp.RemoveColumn( 2 ); 

    for( i = 5; i < tmp.GetNrCols(); i++ )
      tmp.RemoveColumn( 2 ); 

    //tmp.RemoveColumnsAfterIndex( 3 );

    if( !m_UWB.data.Redim( tmp.nrows(), tmp.ncols()-1 )  )
    {
      GNSS_ERROR_MSG( "if( !m_UWB.data.Redim( tmp.nrows(), tmp.ncols()-1 )  )" );
      return false;
    }*/
    
#ifdef MSS
    if (!tmp.Redim(tmp.nrows(), (tmp.ncols() + 1)))
    {
      GNSS_ERROR_MSG( "if (!tmp.Redim(tmp.nrows(), (tmp.ncols() + 1))" );
      return false;
    }
#endif

    if( !m_UWB.data.Redim( tmp.nrows(), 5 ) )
    {
      GNSS_ERROR_MSG( "if( !m_UWB.data.Redim( j, 5 ) )" );
      return false;
    }

  /*  double tmp2 = 0.0;
    double tmp3 = 0.0;
    double tmp4 = 0.0;
    double tmp5 = 0.0;*/

    // Convert to meters
    for( i = 0; i < tmp.nrows(); i++ )
    {    

      // double sum = 0.0;
      double minimum = 0.0;

#ifdef TimeDomain

     /* tmp2 = tmp[i][26];
      tmp3 = tmp[i][13];
      tmp4 = tmp[i][5];
      tmp5 = tmp[i][6];*/


      if( (tmp[i][26] > 0.0 && fabs(tmp[i][13]) < 1e-06) && (tmp[i][5] != 0 && tmp[i][6] != 0) )
      {
        m_UWB.data[j][0] = tmp[i][0]; //GPS TOW
        m_UWB.data[j][1] = tmp[i][1]; //GPS Week
        m_UWB.data[j][2] = tmp[i][26]; //Range in meters
        m_UWB.data[j][3] = tmp[i][5]; //Requester ID
        m_UWB.data[j][4] = tmp[i][6]; //Responder ID
        j++;
      }
#else if MSS

     /* tmp2 = tmp[i][];
      tmp3 = tmp[i][];
      tmp4 = tmp[i][];
      tmp5 = tmp[i][];*/

      if (tmp[i][4] > 0)
      {
        if (tmp[i][5] > 0)
        {
          minimum = tmp[i][5];
        }

        for (int a = 1; a < tmp[i][4]; a++)
        {
          if( (tmp[i][5+a] < minimum) && (tmp[i][5+a] > 0) )
          {
            minimum = tmp[i][5+a];
          }
           
          // sum += tmp[i][5+a];
        }

        tmp[i][21] = minimum;

        // tmp[i][21] = sum/tmp[i][4];
      } 

      if ( (tmp[i][4] != 0) && (tmp[i][2] != 0 && tmp[i][3] != 0) )
      {
        m_UWB.data[j][0] = tmp[i][0]; //GPS TOW
        m_UWB.data[j][1] = tmp[i][1]; //GPS Week
        m_UWB.data[j][2] = tmp[i][21]; //Range in meters
        m_UWB.data[j][3] = tmp[i][3]; //Requester ID
        m_UWB.data[j][4] = tmp[i][2]; //Responder ID
        j++;
      }

#endif
    }

       
    if( !m_UWB.data.Redim( j, 5 ) )
    {
      GNSS_ERROR_MSG( "if( !m_UWB.data.Redim( j, 5 ) )" );
      return false;
    }

/*
    // Remove unnecessary data
    for( i = 2; i < 13; i++ )
      tmp.RemoveColumn( 2 );    
    tmp.RemoveColumnsAfterIndex( 3 );

    if( !m_UWB.data.Redim( tmp.nrows(), tmp.ncols()-1 )  )
    {
      GNSS_ERROR_MSG( "if( !m_UWB.data.Redim( tmp.nrows(), tmp.ncols()-1 )  )" );
      return false;
    }

    // Convert to meters
    for( i = 0; i < tmp.nrows(); i++ )
    {
      if( tmp[i][3] > 0.0 && fabs(tmp[i][2]) < 1e-06 )
      {
        m_UWB.data[j][0] = tmp[i][0];
        m_UWB.data[j][1] = tmp[i][1]; 
        m_UWB.data[j][2] = tmp[i][3] * 0.3048 / 100.0; // convert to meters
        j++;
      }
    }
    if( !m_UWB.data.Redim( j, 3 ) )
    {
      GNSS_ERROR_MSG( "if( !m_UWB.data.Redim( j, 3 ) )" );
      return false;
    }
*/

    // count number of responders and sort based on responder ID
    int count = 0;
    int a = 0;
    int b = 0;
    int c = 0;
    Matrix responder1;
    Matrix responder2;
    Matrix responder3;
    Matrix tmpresponder1;
    Matrix tmpresponder2;
    Matrix tmpresponder3;

    if( !m_UWB.responderID.Resize( 10, 1 ) )
    {
      GNSS_ERROR_MSG( "if( !m_UWB.responderID.Redim( 4,1 ) )" );
      return false;
    }

    m_UWB.id_a = a_id;
    if( a_id > 0 )
    {
      if( !responder1.Redim( m_UWB.data.nrows(), 5 ) )
      {
        GNSS_ERROR_MSG( "if( !responder1.Redim( j, 5 ) )" );
        return false;
      }
    }

    m_UWB.id_b = b_id;
    if( b_id > 0 )
    {
      if( !responder2.Redim( m_UWB.data.nrows(), 5 ) )
      {
        GNSS_ERROR_MSG( "if( !responder2.Redim( j, 5 ) )" );
        return false;
      }
    }

    m_UWB.id_c = c_id;
    if( c_id > 0 )
    {
      if( !responder3.Redim( m_UWB.data.nrows(), 5 ) )
      {
        GNSS_ERROR_MSG( "if( !responder3.Redim( j, 5 ) )" );
        return false;
      }
    }

    m_UWB.responderID[0] = m_UWB.data[0][4];

    for( i = 1; i < m_UWB.data.nrows() && count < 10; i++ )
    {
      if ( (m_UWB.data[i][4] != m_UWB.responderID[0][0]) && (m_UWB.data[i][4] != m_UWB.responderID[1][0] && m_UWB.data[i][4] != m_UWB.responderID[2][0]) )
      {
        count++;        
        m_UWB.responderID[count] = m_UWB.data[i][4];        
      }
    }

    m_UWB.nresponders = count+1;

    // sort responder ID from smallest Serial Number to biggest Serial Number
    if( !m_UWB.responderID.Redim( m_UWB.nresponders, 1 ) )
    {
      GNSS_ERROR_MSG( "if( !m_UWB.responderID.Redim( m_UWB.nresponders(), 1 ) )" );
      return false;
    }
    if ( !m_UWB.responderID.Inplace_SortAscending() )
    {
      GNSS_ERROR_MSG( "if ( !m_UWB.responderID.Inplace_SortAscending() )" );
      return false;
    }

    double test = 0.0;
    a = 0;
    b = 0;
    c = 0;
    if( m_UWB.nresponders != 0 )
    {
      for( i = 1; i < m_UWB.data.nrows(); i++ )
      {
        if( m_UWB.data[i][4] == a_id )
        {
          responder1[a][0] = m_UWB.data[i][0];  //GPS TOW
          responder1[a][1] = m_UWB.data[i][1];  //GPS Week
          responder1[a][2] = m_UWB.data[i][2];  //Range in meters
          responder1[a][3] = m_UWB.data[i][3];  //Requester ID
          responder1[a][4] = m_UWB.data[i][4];  //Responder ID
          a++;
        }
        else if( m_UWB.data[i][4] == b_id )
        {
          responder2[b][0] = m_UWB.data[i][0];  //GPS TOW
          responder2[b][1] = m_UWB.data[i][1];  //GPS Week
          responder2[b][2] = m_UWB.data[i][2];  //Range in meters
          responder2[b][3] = m_UWB.data[i][3];  //Requester ID
          responder2[b][4] = m_UWB.data[i][4];  //Responder ID
          b++;
        }
        else if( m_UWB.data[i][4] == c_id )
        {
          responder3[c][0] = m_UWB.data[i][0];  //GPS TOW
          responder3[c][1] = m_UWB.data[i][1];  //GPS Week
          responder3[c][2] = m_UWB.data[i][2];  //Range in meters
          responder3[c][3] = m_UWB.data[i][3];  //Requester ID
          responder3[c][4] = m_UWB.data[i][4];  //Responder ID
          c++;
        }
        else
        {
          GNSS_ERROR_MSG( "Invalid responder id" );
          return false;
        }
      }
    }
    else
    {
      GNSS_ERROR_MSG( "unexpected" );
      return false;
    }
        
    if( responder1.Redim( a, 5 ) )
    {
      sprintf( msg, "UWB_MinimumRanges_%d.csv", (int)(m_UWB.responderID[0]) );
      responder1.PrintDelimited( msg, 9, ',' );
    }
    else
    {
      GNSS_ERROR_MSG( "if( !responder1.Redim( j, 5 ) )" );
      return false;
    }
    if( responder1.isEmpty() )
    {
      m_UWB.id_a = -1;
    }


    if( responder2.Redim( b, 5 ) )
    {
      sprintf( msg, "UWB_MinimumRanges_%d.csv", (int)(m_UWB.responderID[1]) );
      responder2.PrintDelimited( msg, 9, ',' );
    }
    else
    {
      GNSS_ERROR_MSG( "if( !responder2.Redim( j, 5 ) )" );
      return false;
    }
    if( responder2.isEmpty() )
    {
      m_UWB.id_b = -1;
    }

    if( responder3.Redim( c, 5 ) )
    {
      sprintf( msg, "UWB_MinimumRanges_%d.csv", (int)(m_UWB.responderID[2]) );
      responder3.PrintDelimited( msg, 9, ',' );
    }
    else
    {
      GNSS_ERROR_MSG( "if( !responder3.Redim( j, 5 ) )" );
      return false;
    }
    if( responder3.isEmpty() )
    {
      m_UWB.id_c = -1;
    }

    m_UWB.x_a = xUWB_a;
    m_UWB.y_a = yUWB_a;
    m_UWB.z_a = zUWB_a;

    m_UWB.x_b = xUWB_b;
    m_UWB.y_b = yUWB_b;
    m_UWB.z_b = zUWB_b;

    m_UWB.x_c = xUWB_c;
    m_UWB.y_c = yUWB_c;
    m_UWB.z_c = zUWB_c;

#ifdef MSS    
    m_UWB.isHackOn = true;

    return true;
#endif

    /*
    if( isStatic )
    {
      double tmpd[3];
      double range_mean[3];
      double range_std[3];
      
      // sort data based on responder ID

      if( !responder1.GetStats_ColumnMean( 2, range_mean[0], tmpd[0] ) )
      {
        GNSS_ERROR_MSG( "if( !responder1.GetStats_ColumnMean( 2, range_mean[0], tmpd[0] ) )" );
        return false;
      }
      if( !responder1.GetStats_ColumnStdev( 2, range_std[0] ) )
      {
        GNSS_ERROR_MSG( "if( !responder1.GetStats_ColumnStdev( 2, range_std[0] ) )" );
        return false;
      }

      if (count == 1 || count == 2)
      {
        if( !responder2.GetStats_ColumnMean( 2, range_mean[1], tmpd[1] ) )
        {
          GNSS_ERROR_MSG( "if( !responder2.GetStats_ColumnMean( 2, range_mean[1], tmpd[1] ) )" );
          return false;
        }
        if( !responder2.GetStats_ColumnStdev( 2, range_std[1] ) )
        {
          GNSS_ERROR_MSG( "if( !responder2.GetStats_ColumnStdev( 2, range_std[1] ) )" );
          return false;
        }
      }

      if (count == 2)
      {
        if( !responder3.GetStats_ColumnMean( 2, range_mean[2], tmpd[2] ) )
        {
          GNSS_ERROR_MSG( "if( !responder3.GetStats_ColumnMean( 2, range_mean[2], tmpd[2] ) )" );
          return false;
        }
        if( !responder3.GetStats_ColumnStdev( 2, range_std[2] ) )
        {
          GNSS_ERROR_MSG( "if( !responder3.GetStats_ColumnStdev( 2, range_std[2] ) )" );
          return false;
        }
      }

      // first reject those measurements that are 3 sigma from the mean
      // then recompute the mean and standard deviation
      // then reject at 2 sigma

      j = 0;
      for( i = 0; i < responder1.nrows(); i++ )
      {
        if( responder1[i][2] > range_mean[0]+3.0*range_std[0] )
          continue;
        if( responder1[i][2] < range_mean[0]-3.0*range_std[0] )
          continue;

        tmpresponder1[j][0] = responder1[i][0];
        tmpresponder1[j][1] = responder1[i][1];
        tmpresponder1[j][2] = responder1[i][2];   
        tmpresponder1[j][3] = responder1[i][3];
        tmpresponder1[j][4] = responder1[i][4];
        j++;    
      }

      if( !tmpresponder1.Redim( j, 5 ) )
      {
        GNSS_ERROR_MSG( "if( !tmp.Redim( j, 5 ) )" );
        return false;
      }

      if( !tmpresponder1.GetStats_ColumnMean( 2, range_mean[0], tmpd[0] ) )
      {
        GNSS_ERROR_MSG( "if( !tmp.GetStats_ColumnMean( 2, range_mean[0], tmpd[0] ) )" );
        return false;
      }
      if( !tmp.GetStats_ColumnStdev( 2, range_std[0] ) )
      {
        GNSS_ERROR_MSG( "if( !tmp.GetStats_ColumnStdev( 2, range_std[0] ) )" );
        return false;
      }

      j = 0;
      for( i = 0; i < tmpresponder1.nrows(); i++ )
      {
        if( tmpresponder1[i][2] > range_mean[0]+2.0*range_std[0] )
          continue;
        if( tmpresponder1[i][2] < range_mean[0]-2.0*range_std[0] )
          continue;
      }

      if (count == 1 || count == 2) // 2 or 3 responder
      {
        j = 0;
        for( i = 0; i < responder2.nrows(); i++ )
        {
          if( responder2[i][2] > range_mean[1]+3.0*range_std[1] )
            continue;
          if( responder2[i][2] < range_mean[1]-3.0*range_std[1] )
            continue;

          tmpresponder2[j][0] = responder2[i][0];
          tmpresponder2[j][1] = responder2[i][1];
          tmpresponder2[j][2] = responder2[i][2];   
          tmpresponder2[j][3] = responder2[i][3];
          tmpresponder2[j][4] = responder2[i][4];
          j++;    
        }

        if( !tmpresponder2.Redim( j, 5 ) )
        {
          GNSS_ERROR_MSG( "if( !tmpresponder2.Redim( j, 5 ) )" );
          return false;
        }

        if( !tmpresponder2.GetStats_ColumnMean( 2, range_mean[1], tmpd[1] ) )
        {
          GNSS_ERROR_MSG( "if( !tmpresponder2.GetStats_ColumnMean( 2, range_mean[1], tmpd[1] ) )" );
          return false;
        }
        if( !tmpresponder2.GetStats_ColumnStdev( 2, range_std[1] ) )
        {
          GNSS_ERROR_MSG( "if( !tmpresponder2.GetStats_ColumnStdev( 2, range_std[1] ) )" );
          return false;
        }

        j = 0;
        for( i = 0; i < tmpresponder2.nrows(); i++ )
        {
          if( tmpresponder2[i][2] > range_mean[1]+2.0*range_std[1] )
            continue;
          if( tmpresponder2[i][2] < range_mean[1]-2.0*range_std[1] )
            continue;
        }
      }

      if (count == 2) // 3 responders
      {
        j = 0;
        for( i = 0; i < responder3.nrows(); i++ )
        {
          if( responder3[i][2] > range_mean[2]+3.0*range_std[2] )
            continue;
          if( responder3[i][2] < range_mean[2]-3.0*range_std[2] )
            continue;

          tmpresponder3[j][0] = responder3[i][0];
          tmpresponder3[j][1] = responder3[i][1];
          tmpresponder3[j][2] = responder3[i][2];   
          tmpresponder3[j][3] = responder3[i][3];
          tmpresponder3[j][4] = responder3[i][4];
          j++;    
        }
              
        if( !tmpresponder3.Redim( j, 5 ) )
        {
          GNSS_ERROR_MSG( "if( !tmpresponder3.Redim( j, 5 ) )" );
          return false;
        }

        if( !tmpresponder3.GetStats_ColumnMean( 2, range_mean[2], tmpd[2] ) )
        {
          GNSS_ERROR_MSG( "if( !tmpresponder3.GetStats_ColumnMean( 2, range_mean[2], tmpd[2] ) )" );
          return false;
        }
        if( !tmpresponder3.GetStats_ColumnStdev( 2, range_std[2] ) )
        {
          GNSS_ERROR_MSG( "if( !tmpresponder3.GetStats_ColumnStdev( 2, range_std[2] ) )" );
          return false;
        }

        j = 0;
        for( i = 0; i < tmpresponder3.nrows(); i++ )
        {
          if( tmpresponder3[i][2] > range_mean[2]+2.0*range_std[2] )
            continue;
          if( tmpresponder3[i][2] < range_mean[2]-2.0*range_std[2] )
            continue;
        }
      }
      

      // need to sort by GPS TOW and put back into m_UWB.data

      if( !tmp.Resize( tmpresponder1.nrows() + tmpresponder2.nrows() + tmpresponder3.nrows(), 5 ) )
      {
        GNSS_ERROR_MSG( "if( !tmp.Resize( j, 5 ) )" );
        return false;
      }

      for ( i = 0 ; i < tmpresponder1.nrows(); i++)
      {
        tmp[j][0] = tmpresponder1[i][0];
        tmp[j][1] = tmpresponder1[i][1];
        tmp[j][2] = tmpresponder1[i][2];
        tmp[j][3] = tmpresponder1[i][3];
        tmp[j][4] = tmpresponder1[i][4];
        j++;    
      }

      for ( i = 0 ; i < tmpresponder2.nrows(); i++)
      {
        tmp[j][0] = tmpresponder2[i][0];
        tmp[j][1] = tmpresponder2[i][1];
        tmp[j][2] = tmpresponder2[i][2];
        tmp[j][3] = tmpresponder2[i][3];
        tmp[j][4] = tmpresponder2[i][4];
        j++;  
      }

      for ( i = 0 ; i < tmpresponder3.nrows(); i++)
      {
        tmp[j][0] = tmpresponder3[i][0];
        tmp[j][1] = tmpresponder3[i][1];
        tmp[j][2] = tmpresponder3[i][2];
        tmp[j][3] = tmpresponder3[i][3];
        tmp[j][4] = tmpresponder3[i][4];
        j++;  
      }

      if( !tmp.Inplace_SortByColumn(0) )
      {
        GNSS_ERROR_MSG( "if( !tmp.Inplace_SortByColumn(0) )" );
        return false;
      }

      if( !m_UWB.data.Redim( tmp.nrows(), 5 ) )
      {
        GNSS_ERROR_MSG( "if( !m_UWB.data.Redim( j, 5 ) )" );
        return false;
      }

      j = 0;
      for ( i = 0; i < tmp.nrows(); i++)
      {
        m_UWB.data[j][0] = tmp[i][0];
        m_UWB.data[j][1] = tmp[i][1];
        m_UWB.data[j][2] = tmp[i][2];
        m_UWB.data[j][3] = tmp[i][3];
        m_UWB.data[j][4] = tmp[i][4];
        j++;
      }

      j = 0;
    }

    m_UWB.data.Print( "FilteredUWBRangeData_TDC.txt", 9 );
    */

#endif

    /*
    if( isStatic )
    {
      double tmpd;
      double range_mean = 0;
      double range_std = 0;
      
      if( !tmp.Resize( j, 3 ) )
      {
        GNSS_ERROR_MSG( "if( !tmp.Resize( j, 3 ) )" );
        return false;
      }

      if( !m_UWB.data.GetStats_ColumnMean( 2, range_mean, tmpd ) )
      {
        GNSS_ERROR_MSG( "if( !m_UWB.data.GetStats_ColumnMean( 2, range_mean, tmpd ) )" );
        return false;
      }
      if( !m_UWB.data.GetStats_ColumnStdev( 2, range_std ) )
      {
        GNSS_ERROR_MSG( "if( !m_UWB.data.GetStats_ColumnStdev( 2, range_std ) )" );
        return false;
      }

      // first reject those measurements that are 3 sigma from the mean
      // then recompute the mean and standard deviation
      // then reject at 2 sigma

      j = 0;
      for( i = 0; i < m_UWB.data.nrows(); i++ )
      {
        if( m_UWB.data[i][2] > range_mean+3.0*range_std )
          continue;
        if( m_UWB.data[i][2] < range_mean-3.0*range_std )
          continue;

        tmp[j][0] = m_UWB.data[i][0];
        tmp[j][1] = m_UWB.data[i][1];
        tmp[j][2] = m_UWB.data[i][2];        
        j++;    
      }

      if( !tmp.Redim( j, 3 ) )
      {
        GNSS_ERROR_MSG( "if( !tmp.Redim( j, 3 ) )" );
        return false;
      }

      if( !tmp.GetStats_ColumnMean( 2, range_mean, tmpd ) )
      {
        GNSS_ERROR_MSG( "if( !tmp.GetStats_ColumnMean( 2, range_mean, tmpd ) )" );
        return false;
      }
      if( !tmp.GetStats_ColumnStdev( 2, range_std ) )
      {
        GNSS_ERROR_MSG( "if( !tmp.GetStats_ColumnStdev( 2, range_std ) )" );
        return false;
      }

      j = 0;
      for( i = 0; i < tmp.nrows(); i++ )
      {
        if( tmp[i][2] > range_mean+2.0*range_std )
          continue;
        if( tmp[i][2] < range_mean-2.0*range_std )
          continue;

        m_UWB.data[j][0] = tmp[i][0];
        m_UWB.data[j][1] = tmp[i][1];
        m_UWB.data[j][2] = tmp[i][2];
        j++;    
      }
      if( !m_UWB.data.Redim( j, 3 ) )
      {
        GNSS_ERROR_MSG( "if( !m_UWB.data.Redim( j, 3 ) )" );
        return false;
      }

      // just for debug
      if( !m_UWB.data.GetStats_ColumnMean( 2, range_mean, tmpd ) )
      {
        GNSS_ERROR_MSG( "if( !m_UWB.data.GetStats_ColumnMean( 2, range_mean, tmpd ) )" );
        return false;
      }
      if( !m_UWB.data.GetStats_ColumnStdev( 2, range_std ) )
      {
        GNSS_ERROR_MSG( "if( !m_UWB.data.GetStats_ColumnStdev( 2, range_std ) )" );
        return false;    
      }
      j = 0;
    }
        
    m_UWB.data.Print( "FilteredUWBRangeData.txt", 9 );

#endif
*/
    m_UWB.isHackOn = true;

    return true;
  }

  bool GNSS_RxData::LoadUWBRangeForThisEpoch()
  {
    unsigned i = 0;
    unsigned j = 0;
    double uwb_time = 0.0;
    double tdiff = 0.0;
    double uwb_range = 0.0;
    double time = 0.0;
    bool boolResponder1 = true;
    bool boolResponder2 = true;
    bool boolResponder3 = true;

    if( m_UWB.data.nrows() < 1 || m_UWB.data.ncols() != 5 )
    {
      GNSS_ERROR_MSG( "if( m_UWB.data.nrows() < 1 || m_UWB.data.ncols() != 5 )" );
      return false;
    }

    /// Just match the closest epoch before the gps measurment, not after.
    m_UWB.current_epoch = m_pvt.time.gps_tow + m_pvt.time.gps_week*SECONDS_IN_WEEK;

    for( i = 0; i < m_UWB.data.nrows(); i++ )
    {
      uwb_time = m_UWB.data[i][0] + m_UWB.data[i][1]*SECONDS_IN_WEEK;
      if( uwb_time > m_UWB.current_epoch )
      {
        // in the case where there are 0 satellites and only UWB ranges, LoadNext for RINEX will fail and m_pvt.time.gps_tow
        // time does not get updated. So, m_pvt.time.gps_tow will be set to UWB time until satellites get observed again
        if (m_nrValidObs == 0)
        {
          time = floor(m_pvt.time.gps_tow) + 1;
          m_pvt.time.gps_tow = time;
          m_pvt.time.gps_week = (unsigned int) (m_UWB.data[i][1]);

          m_pvt_lsq.time.gps_tow = time;
          m_pvt_lsq.time.gps_week = (unsigned int) (m_UWB.data[i][1]);
        }
        break;
      }
    }
    

    if( i == m_UWB.data.nrows() )
    {
      m_UWB.isValidForThisEpoch = false;
      m_UWB.index_in_obs_array = -1;      
      return true; // no measurement;
    }

    tdiff = fabs(uwb_time-m_UWB.current_epoch);
    if( tdiff > 1.0 )
    {
      m_UWB.isValidForThisEpoch = false;
      m_UWB.index_in_obs_array = -1;
      return true; // no measurement
    }
    m_UWB.isValidForThisEpoch = true;
    m_UWB.occurances = 0;

    unsigned k = 0;
    int maxchannel = -1;
    for( k = 0; k < m_nrValidObs; k++ )
    {
      if( m_ObsArray[k].channel > maxchannel )
        maxchannel = m_ObsArray[k].channel;
    }      

    j = i;

    do
    {
      if( (m_UWB.data[j][4] == m_UWB.id_a) && boolResponder1 )
      {
        uwb_range = m_UWB.data[j][2];

        m_nrValidObs++;
        i = m_nrValidObs-1;
        m_UWB.index_in_obs_array = i;
        memset( &(m_ObsArray[i]), 0, sizeof(GNSS_structMeasurement) );
        m_ObsArray[i].flags.isActive = true;
        m_ObsArray[i].flags.isPsrValid = true;
        m_ObsArray[i].flags.isPsrUsedInSolution = true;
        m_ObsArray[i].psr = uwb_range;
        m_ObsArray[i].codeType = GNSS_UWBCodeType;
        m_ObsArray[i].freqType = GNSS_UWBFrequency;
        m_ObsArray[i].system = GNSS_UWBSystem;
        m_ObsArray[i].id = 1600 + m_UWB.id_a; // arbitrary
        m_ObsArray[i].satellite.x = m_UWB.x_a;
        m_ObsArray[i].satellite.y = m_UWB.y_a;
        m_ObsArray[i].satellite.z = m_UWB.z_a;
        m_ObsArray[i].satellite.isValid = true;    
        m_ObsArray[i].stdev_psr = 1.15f; // GDM - for MSSI, this is the output precision
        m_ObsArray[i].tow = m_UWB.data[j][0];
        m_ObsArray[i].week = (unsigned short)m_UWB.data[j][1];
        m_ObsArray[i].flags.isAdrValid = 0;
        m_ObsArray[i].flags.isDopplerValid = 0;
        maxchannel++;
        m_ObsArray[i].channel = maxchannel;

        boolResponder1 = false;
        m_UWB.occurances++;
      }
      else if( (m_UWB.data[j][4] == m_UWB.id_b) && boolResponder2 )
      {
        uwb_range = m_UWB.data[j][2];

        m_nrValidObs++;
        i = m_nrValidObs-1;
        m_UWB.index_in_obs_array = i;
        memset( &(m_ObsArray[i]), 0, sizeof(GNSS_structMeasurement) );
        m_ObsArray[i].flags.isActive = true;
        m_ObsArray[i].flags.isPsrValid = true;
        m_ObsArray[i].flags.isPsrUsedInSolution = true;
        m_ObsArray[i].psr = uwb_range;
        m_ObsArray[i].codeType = GNSS_UWBCodeType;
        m_ObsArray[i].freqType = GNSS_UWBFrequency;
        m_ObsArray[i].system = GNSS_UWBSystem;
        m_ObsArray[i].id = 1600 + m_UWB.id_b; // arbitrary
        m_ObsArray[i].satellite.x = m_UWB.x_b;
        m_ObsArray[i].satellite.y = m_UWB.y_b;
        m_ObsArray[i].satellite.z = m_UWB.z_b;
        m_ObsArray[i].satellite.isValid = true;    
        m_ObsArray[i].stdev_psr = 1.15f; // GDM - for MSSI, this is the output precision
        m_ObsArray[i].tow = m_UWB.data[j][0];
        m_ObsArray[i].week = (unsigned short)m_UWB.data[j][1];
        m_ObsArray[i].flags.isAdrValid = 0;
        m_ObsArray[i].flags.isDopplerValid = 0;
        maxchannel++;
        m_ObsArray[i].channel = maxchannel;

        boolResponder2 = false;
        m_UWB.occurances++;
      }
      else if( (m_UWB.data[j][4] == m_UWB.id_c) && boolResponder3 )
      {
        uwb_range = m_UWB.data[j][2];

        m_nrValidObs++;
        i = m_nrValidObs-1;
        m_UWB.index_in_obs_array = i;
        memset( &(m_ObsArray[i]), 0, sizeof(GNSS_structMeasurement) );
        m_ObsArray[i].flags.isActive = true;
        m_ObsArray[i].flags.isPsrValid = true;
        m_ObsArray[i].flags.isPsrUsedInSolution = true;
        m_ObsArray[i].psr = uwb_range;
        m_ObsArray[i].codeType = GNSS_UWBCodeType;
        m_ObsArray[i].freqType = GNSS_UWBFrequency;
        m_ObsArray[i].system = GNSS_UWBSystem;
        m_ObsArray[i].id = 1600 + m_UWB.id_c; // arbitrary
        m_ObsArray[i].satellite.x = m_UWB.x_c;
        m_ObsArray[i].satellite.y = m_UWB.y_c;
        m_ObsArray[i].satellite.z = m_UWB.z_c;
        m_ObsArray[i].satellite.isValid = true;    
        m_ObsArray[i].stdev_psr = 1.15f; // GDM - for MSSI, this is the output precision
        m_ObsArray[i].tow = m_UWB.data[j][0];
        m_ObsArray[i].week = (unsigned short)m_UWB.data[j][1];
        m_ObsArray[i].flags.isAdrValid = 0;
        m_ObsArray[i].flags.isDopplerValid = 0;
        maxchannel++;
        m_ObsArray[i].channel = maxchannel;

        boolResponder3 = false;
        m_UWB.occurances++;
      }

      j++;
      if( j >= m_UWB.data.nrows() )
        break;

    } while ( fabs(m_UWB.data[j][0] - m_pvt.time.gps_tow) < 1.0 );

    return true;
  }

#endif


  bool GNSS_RxData::ZeroAllMeasurements()
  {
    unsigned i = 0;

    m_nrValidObs = 0;

    for( i = 0; i < GNSS_RXDATA_NR_CHANNELS; i++ )
    {
      memset( &(m_ObsArray[i]), 0, sizeof(GNSS_structMeasurement) );
      memset( &(m_prev_ObsArray[i]), 0, sizeof(GNSS_structMeasurement) );

      m_ObsArray[i].index_differential = -1;
      m_prev_ObsArray[i].index_differential = -1;
    }
    return true;
  }


  bool GNSS_RxData::ZeroPVT()
  {
    memset( &m_pvt, 0, sizeof(GNSS_structPVT) );
    memset( &m_prev_pvt, 0, sizeof(GNSS_structPVT) );
    memset( &m_datum_pvt, 0, sizeof(GNSS_structPVT) );
    memset( &m_pvt_lsq, 0, sizeof(GNSS_structPVT) );
    memset( &m_pvt_fixed, 0, sizeof(GNSS_structPVT) );
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
      GNSS_ERROR_MSG( "if( path == NULL )" );
      return false;
    }
    if( rxType == GNSS_RXDATA_UNKNOWN )
    {
      GNSS_ERROR_MSG( "if( rxType == GNSS_RXDATA_UNKNOWN )" );
      return false;
    }

    m_rxDataType = rxType;
    
    if( rxType == GNSS_RXDATA_RINEX21 || rxType == GNSS_RXDATA_RINEX211 )
    {
      if( !CheckRINEXObservationHeader( path, isRinexValid ) )
      {
        GNSS_ERROR_MSG( "CheckRINEXObservationHeader returned false." );
        return false;
      }
      if( !isRinexValid )
      {
        GNSS_ERROR_MSG( "if( !isRinexValid )" );
        return false;
      }
    }

    if( RINEX_ephemeris_path != NULL )
    {
      m_RINEX_eph.filepath = RINEX_ephemeris_path;
      
      if( !LoadRINEXNavigationData() )
      {
        GNSS_ERROR_MSG( "LoadRINEXNavigationData returned false." );
        return false;
      }

      // Indicate that the RINEX ephemeris data can be used.
      m_RINEX_use_eph = true;
    }

#ifndef _CRT_SECURE_NO_DEPRECATE
    if( fopen_s( &m_fid, path, "rb" ) != 0 )
    {
      GNSS_ERROR_MSG( "fopen_s returned non zero." );
      return false;
    }
#else
    m_fid = fopen( path, "rb" );
#endif
    if( m_fid == NULL )  
    {
      GNSS_ERROR_MSG( "if( m_fid == NULL )" );
      return false;
    }


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
      GNSS_ERROR_MSG( "if( feof(m_fid) || ferror(m_fid) != 0 )" );
      return false;
    }
    
    isValidPath = true;

    return true;
  }

  bool GNSS_RxData::LoadNext( bool &endOfStream )
  {
    unsigned i = 0;
    unsigned j = 0;
    double psr_difference = 0;
    bool result = false;
    char msg[256];
    double estimated_psr = 0;
    double mean_doppler = 0;
    double current_time = 0;
    double prev_time = 0;
    double dT = 0;

    // Reset the millisecond jump detectors.
    m_msJumpDetected_Positive = false;
    m_msJumpDetected_Negative = false;
    m_clockJumpDetected = false;
    m_clockJump = 0.0;

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
        GNSS_ERROR_MSG( "Unexpected default case reached." );
        return false;
        break;
      }
    }

    // Check for millisecond jumps in the data.
    // First determine the index of the corresponding measurement in the previous epoch
    for( i = 0; i < m_nrValidObs; i++ )
    {
      if( m_ObsArray[i].tow == 0.0 &&
        m_ObsArray[i].week == 0 )
        continue;

      m_ObsArray[i].index_time_differential = -1;
      if( m_ObsArray[i].flags.isActive && 
        m_ObsArray[i].flags.isPsrValid )
      {
        for( j = 0; j < m_prev_nrValidObs; j++ )
        {
          if( m_prev_ObsArray[j].flags.isActive && 
            m_prev_ObsArray[j].flags.isPsrValid )
          {
            if( m_ObsArray[i].id == m_prev_ObsArray[j].id )
            {
              m_ObsArray[i].index_time_differential = j;

              current_time = m_ObsArray[i].tow + m_ObsArray[i].week*SECONDS_IN_WEEK;
              prev_time = m_prev_ObsArray[j].tow + m_prev_ObsArray[j].week*SECONDS_IN_WEEK;
              dT = current_time - prev_time;

              mean_doppler = 0;
              if( m_ObsArray[i].flags.isDopplerValid && m_prev_ObsArray[j].flags.isDopplerValid )
              {
                mean_doppler = m_ObsArray[i].doppler + m_prev_ObsArray[j].doppler;
                mean_doppler /= 2.0;
              }
              else if( m_ObsArray[i].flags.isDopplerValid )
              {
                mean_doppler = m_ObsArray[i].doppler;
              }
              if( m_ObsArray[i].freqType == GNSS_GPSL1 )
              {
                estimated_psr = m_prev_ObsArray[j].psr - mean_doppler*GPS_WAVELENGTHL1*dT;
              }
              else if( m_ObsArray[i].freqType == GNSS_GPSL2 )
              {
                estimated_psr = m_prev_ObsArray[j].psr - mean_doppler*GPS_WAVELENGTHL2*dT;
              }
              else
              {
                estimated_psr = m_prev_ObsArray[j].psr;
              }

              psr_difference = m_ObsArray[i].psr - estimated_psr;

              if( fabs( psr_difference ) > ONE_MS_IN_M/2.0 )
              {
                // detect allowing 10 km psr changes between epochs
                if( psr_difference > 0 )
                {
                  if( (psr_difference > (ONE_MS_IN_M - 10000.0)) && (psr_difference < (ONE_MS_IN_M + 10000.0)) )
                  {
                    sprintf( msg, "%.1Lf  %d  Clock Jump of %.1Lf m, %.6Lf ms detected.", m_pvt.time.gps_tow, m_pvt.time.gps_week, psr_difference, psr_difference/ONE_MS_IN_M );
                    GNSS_ERROR_MSG( msg );
                    m_msJumpDetected_Positive = true;
                  }
                }
                else
                {
                  psr_difference *= -1.0;
                  if( (psr_difference > (ONE_MS_IN_M - 10000.0)) && (psr_difference < (ONE_MS_IN_M + 10000.0)) )
                  {
                    sprintf( msg, "%.1Lf  %d  Clock Jump of %.1Lf m, %.6Lf ms detected.", m_pvt.time.gps_tow, m_pvt.time.gps_week, psr_difference, psr_difference/ONE_MS_IN_M );
                    GNSS_ERROR_MSG( msg );                  
                    m_msJumpDetected_Negative = true;
                  }
                }
              }
              break;
            }
          }
        }
      }
    }

    if( !m_msJumpDetected_Positive && !m_msJumpDetected_Negative )
    {
      int mean_n = 0;
      double mean_val = 0.0;

      // Check for large clock corrections that are not modulo 1 ms.
      for( i = 0; i < m_nrValidObs; i++ )
      {
        if( m_ObsArray[i].tow == 0.0 &&
          m_ObsArray[i].week == 0 )
          continue;

        m_ObsArray[i].index_time_differential = -1;
        if( m_ObsArray[i].flags.isActive && 
          m_ObsArray[i].flags.isPsrValid )
        {
          for( j = 0; j < m_prev_nrValidObs; j++ )
          {
            if( m_prev_ObsArray[j].flags.isActive && 
              m_prev_ObsArray[j].flags.isPsrValid )
            {
              if( m_ObsArray[i].id == m_prev_ObsArray[j].id )
              {
                m_ObsArray[i].index_time_differential = j;

                current_time = m_ObsArray[i].tow + m_ObsArray[i].week*SECONDS_IN_WEEK;
                prev_time = m_prev_ObsArray[j].tow + m_prev_ObsArray[j].week*SECONDS_IN_WEEK;
                dT = current_time - prev_time;

                mean_doppler = 0;
                if( m_ObsArray[i].flags.isDopplerValid && m_prev_ObsArray[j].flags.isDopplerValid )
                {
                  mean_doppler = m_ObsArray[i].doppler + m_prev_ObsArray[j].doppler;
                  mean_doppler /= 2.0;
                }
                else if( m_ObsArray[i].flags.isDopplerValid )
                {
                  mean_doppler = m_ObsArray[i].doppler;
                }
                if( m_ObsArray[i].freqType == GNSS_GPSL1 )
                {
                  estimated_psr = m_prev_ObsArray[j].psr - mean_doppler*GPS_WAVELENGTHL1*dT;
                }
                else if( m_ObsArray[i].freqType == GNSS_GPSL2 )
                {
                  estimated_psr = m_prev_ObsArray[j].psr - mean_doppler*GPS_WAVELENGTHL2*dT;
                }
                else
                {
                  estimated_psr = m_prev_ObsArray[j].psr;
                }

                psr_difference = m_ObsArray[i].psr - estimated_psr;

                // Look for any arbitrary clock jumps larger than 1/2 millisecond
                if( fabs(psr_difference) > ONE_MS_IN_M/2.0 )
                {
                  mean_n++;
                  mean_val += psr_difference;
                  m_clockJumpDetected = true;                  
                }              
                break;
              }
            }
          }
        }
      }
      if( m_clockJumpDetected )
      {
        mean_val /= double(mean_n);
        m_clockJump = mean_val;
        sprintf( msg, "%.1Lf  %d  Clock Jump of %.1Lf m, %.6Lf ms detected.", m_pvt.time.gps_tow, m_pvt.time.gps_week, m_clockJump, m_clockJump/ONE_MS_IN_M );
        GNSS_ERROR_MSG( msg );          

        // indicate a cycle slip on all channels
        for( i = 0; i < m_nrValidObs; i++ )
        {
          m_ObsArray[i].flags.isNoCycleSlipDetected = 0; // Indicate a cycle slip has occured.
        }
      }
    }

    if( m_msJumpDetected_Positive || m_msJumpDetected_Negative )
    {
      // indicate a cycle slip on all channels
      for( i = 0; i < m_nrValidObs; i++ )
      {
        m_ObsArray[i].flags.isNoCycleSlipDetected = 0; // Indicate a cycle slip has occured.
      }
    }

#ifdef GDM_UWB_RANGE_HACK
    if( m_pvt.time.gps_tow > 179705 )
      int gaa = 100;

    if( result && m_UWB.isHackOn )
    {
      result = LoadUWBRangeForThisEpoch();
      if( !result )
      {
        GNSS_ERROR_MSG( "LoadUWBRangeForThisEpoch returned false." );
        return false;
      }
    }
#endif


    if( result )
    {
      if( m_RINEX_use_eph )
      {
        if( !UpdateTheEphemerisArrayWithUsingRINEX() )
        {
          GNSS_ERROR_MSG( "UpdateTheEphemerisArrayWithUsingRINEX returned false." );
          return false;
        }
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
    {
      GNSS_ERROR_MSG( "if( filepath == NULL )" );
      return false;    
    }
    
    result = RINEX_GetHeader( 
      filepath,
      RINEX_buffer,
      16384,
      &RINEX_buffer_size,
      &version,
      &file_type
    );
    if( result == FALSE )
    {
      GNSS_ERROR_MSG( "RINEX_GetHeader returned FALSE." );
      return false;
    }

    result = RINEX_DecodeHeader_ObservationFile(
      RINEX_buffer,
      RINEX_buffer_size,
      &m_RINEX_obs_header
      );
    if( result == FALSE )
    {
      GNSS_ERROR_MSG( "RINEX_DecodeHeader_ObservationFile returned false." );
      return false;
    }

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

    double current_time = 0;
    double prev_time = 0;
    double delta_time = 0;

    BOOL wasEndOfFileReached=0;  // Has the end of the file been reached (output).
    BOOL wasObservationFound=0;  // Was a valid observation found (output).
    unsigned filePosition=0;   // The file position for the start of the 
    unsigned nrObs = 0;
    bool isAvailable = false;
    
    endOfStream = false;

    if( m_fid == NULL )
    {
      GNSS_ERROR_MSG( "if( m_fid == NULL )" );
      return false;
    }

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
    {
      GNSS_ERROR_MSG( "RINEX_GetNextObservationSet returned false." );
      return false;
    }

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

      // It's the same for the least squares container.
      m_pvt_lsq.time.gps_week = rx_gps_week;
      m_pvt_lsq.time.gps_tow  = rx_gps_tow;

      for(i = 0; i < nrObs; i++ )
      {
        // Use a constrained solution and variance-covariance analysis to determine these values.
        if( m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          m_ObsArray[i].stdev_psr     = static_cast<float>(m_default_stdev_GPSL1_psr);     // [m]
          m_ObsArray[i].stdev_adr     = static_cast<float>(m_default_stdev_GPSL1_adr);     // [cycles]
          m_ObsArray[i].stdev_doppler = static_cast<float>(m_default_stdev_GPSL1_doppler); // [Hz]
        }
        else
        {
          // L2, L5 to deal with later.
          m_ObsArray[i].stdev_psr     = 0.8f;  // [m]
          m_ObsArray[i].stdev_adr     = 0.03f; // [cycles]
          m_ObsArray[i].stdev_doppler = 0.09f; // [Hz]
        }

        // Check if ephemeris information is available
        if( !m_EphAlmArray.IsEphemerisAvailable( m_ObsArray[i].id, isAvailable ) )
        {
          GNSS_ERROR_MSG( "m_EphAlmArray.IsEphemerisAvailable() returned false" );
          return false;
        }
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

#define GDM_HACK_FOR_ONLY_L1
#ifdef GDM_HACK_FOR_ONLY_L1
    j = 0;
    for( i = 0; i < m_nrValidObs; i++ )
    {
      if( m_ObsArray[i].freqType == GNSS_GPSL1 )
      {
        memcpy( &m_ObsArray[j], &m_ObsArray[i], sizeof(GNSS_structMeasurement) );
        if( i != j )
          memset( &(m_ObsArray[i]), 0, sizeof(GNSS_structMeasurement) );
        j++;
      }      
      else
      {
        memset( &(m_ObsArray[i]), 0, sizeof(GNSS_structMeasurement) );
      }
    }
    m_nrValidObs = j;
#endif
    

    // Search for matching observations in the previous set of data to pass on static information
    // like ambiguities.
    for( i = 0; i < m_nrValidObs; i++ )
    {
      delta_time = 0;
      for( j = 0; j < m_prev_nrValidObs; j++ )
      {
        if( m_ObsArray[i].codeType == m_prev_ObsArray[j].codeType &&
          m_ObsArray[i].freqType == m_prev_ObsArray[j].freqType &&
          m_ObsArray[i].system == m_prev_ObsArray[j].system &&
          m_ObsArray[i].id == m_prev_ObsArray[j].id ) // Note should also check that channels are the same if real time!
        {
          m_ObsArray[i].sd_ambiguity = m_prev_ObsArray[j].sd_ambiguity;

          current_time = m_ObsArray[i].tow + m_ObsArray[i].week*SECONDS_IN_WEEK;
          prev_time    = m_prev_ObsArray[j].tow + m_prev_ObsArray[j].week*SECONDS_IN_WEEK;
          delta_time   = current_time - prev_time;
          
          if( m_ObsArray[i].flags.isAdrValid && m_ObsArray[i].flags.isPhaseLocked )
          {
            m_ObsArray[i].locktime = m_prev_ObsArray[j].locktime + static_cast<float>(delta_time);
          }
          else
          {
            m_ObsArray[i].locktime = 0;
          }
          //if( m_prev_ObsArray[j].ambiguity != 0 )
            //printf( "%10.1Lf    %2d %12.1Lf\n", m_pvt.time.gps_tow, m_ObsArray[i].id, m_ObsArray[i].ambiguity );
          break;
        }
      }
      if( delta_time == 0 )
      {
        m_ObsArray[i].locktime = 0;
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
    {
      GNSS_ERROR_MSG( "if( m_fid == NULL )" );
      return false;
    }

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
          {
            GNSS_ERROR_MSG( "NOVATELOEM4_FindNextMessageInFile returned false." );
            return false;
          }
          
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
                  GNSS_ERROR_MSG( "m_EphAlmArray.AddEphemeris() returned false." );
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
          {
            GNSS_ERROR_MSG( "NOVATELOEM4_DecodeRANGEB returned false." );
            return false;
          }

          // Set the receiver time of the observation set.
          m_pvt.time.gps_week = header.gpsWeek;
          m_pvt.time.gps_tow  = header.gpsMilliSeconds / 1000.0;

          // It's the same for the least squares container.
          m_pvt_lsq.time.gps_week = header.gpsWeek;
          m_pvt_lsq.time.gps_tow  = header.gpsMilliSeconds / 1000.0;

          for( i = 0; i < nrValidObs && i < GNSS_RXDATA_NR_CHANNELS; i++ )
          {
            m_ObsArray[i].tow = m_pvt.time.gps_tow - obsArray[i].psr/LIGHTSPEED;
            m_ObsArray[i].week = m_pvt.time.gps_week;
            if( m_ObsArray[i].tow < 0.0 )
            {
              m_ObsArray[i].tow += SECONDS_IN_WEEK;
              m_ObsArray[i].week -= 1;
            }
            else if( m_ObsArray[i].tow >= SECONDS_IN_WEEK )
            {
              m_ObsArray[i].tow -= SECONDS_IN_WEEK;
              m_ObsArray[i].week += 1;
            }

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

            m_ObsArray[i].psr       = obsArray[i].psr;       // [m]
            m_ObsArray[i].adr       = -1.0*obsArray[i].adr;  // [cycles]
            m_ObsArray[i].doppler   = obsArray[i].doppler;   // [Hz]
            m_ObsArray[i].cno       = obsArray[i].cno;       // [dB-Hz]
            m_ObsArray[i].locktime  = obsArray[i].locktime;  // [s]
    
            if( obsArray[i].psrstd < 0.5 )
              m_ObsArray[i].stdev_psr     = 0.5f;
            else
              m_ObsArray[i].stdev_psr     = obsArray[i].psrstd; 

            if( obsArray[i].adrstd < 0.01 )
              m_ObsArray[i].stdev_adr     = 0.01f; // these are in cycles!.              
            else
              m_ObsArray[i].stdev_adr     = obsArray[i].adrstd; // these are in cycles!.

            m_ObsArray[i].stdev_doppler = 0.09f; // Hz

            m_ObsArray[i].psr_smoothed      = 0.0;
            m_ObsArray[i].psr_predicted     = 0.0;
            m_ObsArray[i].doppler_predicted = 0.0;
            
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
            {
              GNSS_ERROR_MSG( "m_EphAlmArray.IsEphemerisAvailable() returned false." );
              return false;
            }

            m_ObsArray[i].flags.isEphemerisValid      = isAvailable;

            m_ObsArray[i].flags.isAlmanacValid          = 0; // not yet known
            m_ObsArray[i].flags.isAboveElevationMask    = 0; // not yet known
            m_ObsArray[i].flags.isAboveCNoMask          = 0; // not yet known
            m_ObsArray[i].flags.isAboveLockTimeMask     = 0; // not yet known
            m_ObsArray[i].flags.isNotUserRejected       = 1; // assume not rejected
            m_ObsArray[i].flags.isNotPsrRejected        = 1; // assume not rejected
            m_ObsArray[i].flags.isNotAdrRejected        = 1; // assume not rejected
            m_ObsArray[i].flags.isNotDopplerRejected    = 1; // assume not rejected
            m_ObsArray[i].flags.isNoCycleSlipDetected   = 1; // assume no slip
            m_ObsArray[i].flags.isPsrUsedInSolution     = 0; // not yet known
            m_ObsArray[i].flags.isDopplerUsedInSolution = 0; // not yet known
            m_ObsArray[i].flags.isAdrUsedInSolution     = 0; // not yet known

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
        GNSS_ERROR_MSG( "Unexpected default case reached." );
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
          m_ObsArray[i].sd_ambiguity = m_prev_ObsArray[j].sd_ambiguity;
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
      m_pvt,
      latitudeRads,
      longitudeRads,
      height,
      clk,
      std_lat,
      std_lon,
      std_hgt,
      std_clk );
    if( !result )
    {
      GNSS_ERROR_MSG( "UpdatePositionAndRxClock returned false." );
      return false;
    }

    result = UpdateVelocityAndClockDrift(
      m_pvt,
      vn,
      ve,
      vup,
      clkdrift,
      std_vn,
      std_ve,
      std_vup,
      std_clkdrift );
    if( !result )
    {
      GNSS_ERROR_MSG( "UpdateVelocityAndClockDrift returned false." );
      return false;
    }

    if( std_lat == 0.0 && std_lon == 0.0 && std_hgt == 0.0 )
    {
      m_pvt.std_lat = 1e-06;
      m_pvt.std_lon = 1e-06;
      m_pvt.std_hgt = 1e-06;      
      m_pvt.isPositionFixed = 1;
    }

    m_pvt.isHeightConstrained = 0;
    m_pvt.isClockConstrained = 0;
    m_pvt.isSolutionBasedOnEphemeris = 0;

    m_pvt.undulation = undulation;

    // Set the previous pvt as well.
    m_prev_pvt = m_pvt;

    // Set the least squares specific pvt as well.
    m_pvt_lsq = m_pvt;
    
    return true;
  }


  bool GNSS_RxData::UpdatePositionAndRxClock( 
    GNSS_structPVT& pvt,         //!< The position, velocity, and time struct. Usually either rxData.m_pvt or rxData.m_pvt_lsq.
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
    pvt.latitude   = latitudeRads;
    pvt.longitude  = longitudeRads;
    pvt.height     = height;
    
    pvt.latitudeDegs  = latitudeRads*RAD2DEG;
    pvt.longitudeDegs = longitudeRads*RAD2DEG;

    result = GetDMS( 
      pvt.latitudeDegs, 
      pvt.lat_dms.degrees, 
      pvt.lat_dms.minutes, 
      pvt.lat_dms.seconds, 
      (char*)m_pvt.lat_dms.dms_str, 24 );
    if( result == false )
    {
      GNSS_ERROR_MSG( "GetDMS returned false." );
      return false;
    }
    
    result = GetDMS( 
      pvt.longitudeDegs, 
      pvt.lon_dms.degrees, 
      pvt.lon_dms.minutes, 
      pvt.lon_dms.seconds, 
      (char*)pvt.lon_dms.dms_str, 24 );
    if( result == false )
    {
      GNSS_ERROR_MSG( "GetDMS returned false." );
      return false;
    }

    if( GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      pvt.latitude,
      pvt.longitude,
      pvt.height,
      &pvt.x,
      &pvt.y,
      &pvt.z ) == FALSE )
    {
      GNSS_ERROR_MSG( "GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates returned FALSE." );
      return false;
    }
    pvt.clockOffset = clk;

    pvt.std_lat = std_lat;
    pvt.std_lon = std_lon;
    pvt.std_hgt = std_hgt;
    pvt.std_clk = std_clk;
    
    return true;
  }

  bool GNSS_RxData::SetDatumPVT( 
    const double latitudeRads,
    const double longitudeRads,
    const double height 
    )
  {
    bool result;
    m_datum_pvt.latitude   = latitudeRads;
    m_datum_pvt.longitude  = longitudeRads;
    m_datum_pvt.height     = height;
    
    m_datum_pvt.latitudeDegs  = latitudeRads*RAD2DEG;
    m_datum_pvt.longitudeDegs = longitudeRads*RAD2DEG;

    result = GetDMS( 
      m_datum_pvt.latitudeDegs, 
      m_datum_pvt.lat_dms.degrees, 
      m_datum_pvt.lat_dms.minutes, 
      m_datum_pvt.lat_dms.seconds, 
      (char*)m_pvt.lat_dms.dms_str, 24 );
    if( result == false )
    {
      GNSS_ERROR_MSG( "GetDMS returned false." );
      return false;
    }

    result = GetDMS( 
      m_datum_pvt.longitudeDegs, 
      m_datum_pvt.lon_dms.degrees, 
      m_datum_pvt.lon_dms.minutes, 
      m_datum_pvt.lon_dms.seconds, 
      (char*)m_datum_pvt.lon_dms.dms_str, 24 );
    if( result == false )
    {
      GNSS_ERROR_MSG( "GetDMS returned false." );
      return false;
    }

    if( GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      m_datum_pvt.latitude,
      m_datum_pvt.longitude,
      m_datum_pvt.height,
      &m_datum_pvt.x,
      &m_datum_pvt.y,
      &m_datum_pvt.z ) == FALSE )
    {
      GNSS_ERROR_MSG( "GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates returned FALSE." );
      return false;
    }
   
    return true;
  }

  bool GNSS_RxData::UpdateVelocityAndClockDrift( 
    GNSS_structPVT& pvt,         //!< The position, velocity, and time struct. Usually either rxData.m_pvt or rxData.m_pvt_lsq.
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
    pvt.vn  = vn;
    pvt.ve  = ve;
    pvt.vup = vup;

    GEODESY_RotateVectorFromLocalGeodeticFrameToEarthFixedFrame(
      pvt.latitude,
      pvt.longitude,
      pvt.vn,
      pvt.ve,
      pvt.vup,
      &pvt.vx,
      &pvt.vy,
      &pvt.vz );
    
    pvt.clockDrift = clkdrift;

    pvt.std_vn  = std_vn;
    pvt.std_ve  = std_ve;
    pvt.std_vup = std_vup;
    pvt.std_clkdrift  = std_clkdrift;

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
    {
      GNSS_ERROR_MSG( "sprintf returned unexpected result" );
      return false;
    }

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
    double cmc = 0.0;
    double cmc_prev = 0.0;
    double cmc_diff = 0.0;
    char msg[256];

    static double max_dif = 0.0;

    t_prev = m_prev_pvt.time.gps_week*SECONDS_IN_WEEK + m_prev_pvt.time.gps_tow;
    t = m_pvt.time.gps_week*SECONDS_IN_WEEK + m_pvt.time.gps_tow;
    dt = t - t_prev;
    if( dt == 0.0 )
    {
      // nothing to check.
      return true; 
    }
    if( dt < 0.0 )
    {    
      GNSS_ERROR_MSG( "if( dt < 0.0 )" );
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
          m_ObsArray[i].freqType == GNSS_GPSL1 &&
          m_ObsArray[i].freqType == m_prev_ObsArray[j].freqType &&
          m_ObsArray[i].flags.isNoCycleSlipDetected ) // no cycle slip already detected
        {
          // channel matching is not performed.
          // This should be done if this a real time receiver data.
          // However, since the data source is generally not known for
          // post processing software, it is not performed here.
          if( m_ObsArray[i].flags.isPhaseLocked && m_prev_ObsArray[j].flags.isPhaseLocked &&
            m_ObsArray[i].flags.isDopplerValid && m_prev_ObsArray[j].flags.isDopplerValid &&
            m_ObsArray[i].flags.isAdrValid && m_prev_ObsArray[j].flags.isAdrValid &&
            m_ObsArray[i].flags.isParityValid && m_prev_ObsArray[j].flags.isParityValid )            
          {
            mean_doppler = (m_ObsArray[i].doppler + m_prev_ObsArray[j].doppler)/2.0;

            predicted_phase = m_prev_ObsArray[j].adr - mean_doppler * dt; // GDM_BEWARE Doppler sign convention
            if( m_clockJumpDetected )
            {
              predicted_phase += m_clockJump/GPS_WAVELENGTHL1;
            }

            phase_diff = predicted_phase - m_ObsArray[i].adr;

            if( fabs(phase_diff) > max_dif )
              max_dif = fabs(phase_diff);
            if( fabs(phase_diff) > nrThresholdCycles*dt )
            {
              // Compare using code - carrier.
              cmc_prev = m_prev_ObsArray[j].psr - m_prev_ObsArray[j].adr*GPS_WAVELENGTHL1;
              cmc = m_ObsArray[i].psr - m_ObsArray[i].adr*GPS_WAVELENGTHL1;
              cmc_diff = cmc - cmc_prev;

              if( cmc_diff < 5.0 )
              {
                m_ObsArray[i].flags.isNoCycleSlipDetected = 1; // No cycle slip detected.
              }
              else
              {
                sprintf( msg, "%.1Lf %d GPS L1 cycle slip detected\n PRN %d Phase rate method difference= %.1Lf cycles, Time difference code-carrier = %.1Lf m",  m_pvt.time.gps_tow, m_pvt.time.gps_week, m_ObsArray[i].id, phase_diff, cmc_diff );
                GNSS_ERROR_MSG( msg );
                m_ObsArray[i].flags.isNoCycleSlipDetected = 0; // Indicate a cycle slip has occured.
              }
            }
            else
            {
              m_ObsArray[i].flags.isNoCycleSlipDetected = 1; // No cycle slip detected.
            }
            break;
          }        
          else
          {
            if( m_ObsArray[i].flags.isPhaseLocked && m_prev_ObsArray[j].flags.isPhaseLocked &&
              m_ObsArray[i].flags.isPsrValid && m_prev_ObsArray[j].flags.isPsrValid &&
              m_ObsArray[i].flags.isCodeLocked && m_prev_ObsArray[j].flags.isCodeLocked &&
              m_ObsArray[i].flags.isAdrValid && m_prev_ObsArray[j].flags.isAdrValid &&
              m_ObsArray[i].flags.isParityValid && m_prev_ObsArray[j].flags.isParityValid )
            {
              // Compare using code - carrier.
              cmc_prev = m_prev_ObsArray[j].psr - m_prev_ObsArray[j].adr*GPS_WAVELENGTHL1;
              cmc = m_ObsArray[i].psr - m_ObsArray[i].adr*GPS_WAVELENGTHL1;
              cmc_diff = cmc - cmc_prev;

              if( fabs(cmc_diff) > 5.0 )
              {
                sprintf( msg, "%.1Lf %d GPS L1 cycle slip detected\n PRN %d Time difference code-carrier = %.1Lf m",  m_pvt.time.gps_tow, m_pvt.time.gps_week, m_ObsArray[i].id, cmc_diff );
                GNSS_ERROR_MSG( msg );
                m_ObsArray[i].flags.isNoCycleSlipDetected = 0; // Indicate a cycle slip has occured.
              }
              else
              {
                m_ObsArray[i].flags.isNoCycleSlipDetected = 1; // No cycle slip detected.
              }
            }
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
    {
      GNSS_ERROR_MSG( "if( filepath == NULL )" );
      return false;
    }

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
      X[j][i] = m_ObsArray[i].flags.isPsrUsedInSolution; j++;   //!< Indicates if some part (pseudorange) of this channel's measurement was used in the position solution.
      X[j][i] = m_ObsArray[i].flags.isDopplerUsedInSolution; j++;   //!< Indicates if some part (Doppler) of this channel's measurement was used in the velocity solution.
      X[j][i] = m_ObsArray[i].flags.isAdrUsedInSolution; j++;   //!< Indicates if the the ADR is used in the solution.
      X[j][i] = m_ObsArray[i].flags.isDifferentialPsrAvailable; j++;     //!< Indicates if a matching pseudrange observation is available from another receiver.
      X[j][i] = m_ObsArray[i].flags.isDifferentialDopplerAvailable; j++; //!< Indicates if a matching Doppler observation is available from another receiver.
      X[j][i] = m_ObsArray[i].flags.isDifferentialAdrAvailable; j++;     //!< Indicates if a matching ADR observation is available from another receiver.
      X[j][i] = m_ObsArray[i].flags.useTropoCorrection; j++;         //!< Indicates that the tropospheric correction should be applied.
      X[j][i] = m_ObsArray[i].flags.useBroadcastIonoCorrection; j++; //!< Indicates that the broadcast ionospheric correction should be applied.

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
      X[j][i] = m_ObsArray[i].sd_ambiguity; j++;      //!< The estimated integer component of the adr. This may be the single or double differenced ambiguity [].
      X[j][i] = m_ObsArray[i].doppler_predicted; j++; //!< The predicted Doppler based on user position, velocity, satellite position, velocity and clock rate [Hz].
      
      X[j][i] = m_ObsArray[i].index_differential; j++;      //!< The channel index of a matching differential observation. -1 means there is no matching channel.
      X[j][i] = m_ObsArray[i].index_time_differential; j++; //!< The channel index of a matching time differential observation. -1 means there is no matching channel.

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
      GNSS_ERROR_MSG( "if( maxBufferLength < 1024*8 )" );
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
        scount += sprintf( buffer+scount, "%2d", m_ObsArray[i].flags.isPsrUsedInSolution );

        scount += sprintf( buffer+scount, "%9.1lf", m_ObsArray[i].doppler );
        scount += sprintf( buffer+scount, "%2d", m_ObsArray[i].flags.isDopplerUsedInSolution );
        
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
    {
      GNSS_ERROR_MSG( "if( fid == NULL )" );
      return false;
    }

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
    {
      GNSS_ERROR_MSG( "if( m_RINEX_eph.eph_array == NULL )" );
      return false;
    }
     
    m_RINEX_eph.max_array_length = estimated_nr_eph;
    
    result = RINEX_DecodeGPSNavigationFile(
      m_RINEX_eph.filepath.c_str(),
      &m_RINEX_eph.iono_model,
      m_RINEX_eph.eph_array,
      m_RINEX_eph.max_array_length,
      &m_RINEX_eph.array_length
      );
    if( result == FALSE )
    {
      GNSS_ERROR_MSG( "RINEX_DecodeGPSNavigationFile returned FALSE." );
      return false;
    }

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
          {
            GNSS_ERROR_MSG( "m_EphAlmArray.GetEphemerisTOW returned false." );
            return false;
          }

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
                    {
                      GNSS_ERROR_MSG( "m_EphAlmArray.AddEphemeris returned false." );
                      return false;
                    }
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
                    {
                      GNSS_ERROR_MSG( "m_EphAlmArray.AddEphemeris returned false." );
                      return false;
                    }
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
              {
                GNSS_ERROR_MSG( "m_EphAlmArray.AddEphemeris returned false." );
                return false;
              }
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
          {
            GNSS_ERROR_MSG( "m_EphAlmArray.IsEphemerisAvailable returned false." );
            return false;
          }

          m_ObsArray[j].flags.isEphemerisValid = isAvailable;
        }
      }
    }

	return true;
  }

} // end namespace GNSS



